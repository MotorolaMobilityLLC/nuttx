/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <arch/byteorder.h>

#if CONFIG_ARCH_BOARD_APBA
#include <arch/board/factory.h>
#endif

#include <arch/chip/unipro_p2p.h>

#include <nuttx/kmalloc.h>
#include <nuttx/util.h>
#include <nuttx/wqueue.h>
#include <nuttx/version.h>

#include <nuttx/greybus/tsb_unipro.h>
#if defined(CONFIG_I2S_TUNNEL)
# include <nuttx/i2s_tunnel/i2s_unipro.h>
#endif

#include <nuttx/mhb/ipc.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/mhb/mhb_utils.h>

#include "gearbox.h"
#include "mhb_ramlog.h"
#include "mhb_server.h"
#include "sm.h"
#include "sm_timer.h"

enum svc_state {
    SVC_WAIT_FOR_AP,
    SVC_SLAVE_WAIT_FOR_UNIPRO,
    SVC_SLAVE_WAIT_FOR_CPORTS,
    SVC_SLAVE_CONNECTED,
    SVC_WAIT_FOR_UNIPRO,
    SVC_WAIT_FOR_MOD,
    SVC_CONNECTED,
    SVC_DISCONNECTED,
    SVC_TEST_MODE,

    SVC_STATE_MAX,
};

#if CONFIG_DEBUG
const static char *SVC_STATE_STRINGS[] = {
    "WAIT_FOR_AP",
    "SLAVE_WAIT_FOR_UNIPRO",
    "SLAVE_WAIT_FOR_CPORTS",
    "SLAVE_CONNECTED",
    "WAIT_FOR_UNIPRO",
    "WAIT_FOR_MOD",
    "CONNECTED",
    "DISCONNECTED",
    "TEST_MODE",
};

const static char *SVC_EVENT_STRINGS[] = {
    "MASTER_STARTED",
    "SLAVE_STARTED",
    "MODE_STARTED",
    "MOD_DETECTED",
    "LINK_TIMEOUT",
    "MOD_TIMEOUT",
    "LINK_UP",
    "LINK_DOWN",
    "GEAR_SHIFT_DONE",
    "QUEUE_STATS",
    "SEND_STATS",
    "CPORTS_DONE",
};
#endif

struct svc {
    enum svc_state state;
    svc_timer_t mod_detect_tid;
    svc_timer_t linkup_poll_tid;
    svc_timer_t stats_tid;
    uint32_t mod_detect_retry;
    struct gearbox *gearbox;
    bool stats_pending;
};

struct svc_work {
    struct work_s work;
    struct svc *svc;
    enum svc_event event;
    void *parameter0;
    void *parameter1;
    void *parameter2;
};

typedef enum svc_state (*svc_event_handler)(struct svc *svc, struct svc_work *work);

static struct svc g_svc;
extern struct mhb_unipro_stats g_unipro_stats;

#define SVC_LINKUP_POLL_INTERVAL 1000 /* ms */
#define SVC_MOD_DETECT_TIMEOUT 10000 /* ms */
#define SVC_STATS_TIMEOUT 5000 /* ms */
#define SVC_MOD_DETECT_RETRY 5

/* Helpers */
static void _svc_bitmask_to_stats(uint32_t *counter, uint32_t value)
{
    vdbg("%p %d\n", counter, value);

    bool delta = false;

    while (value) {
        if (value & 0x1) {
            (*counter)++;
            delta = true;
        }

        value >>= 1;
        counter++;
    }

    if (delta && !g_svc.stats_pending) {
        svc_send_event(SVC_EVENT_QUEUE_STATS, 0, 0, 0);
        g_svc.stats_pending = true;
    } else {
        vdbg("stats pending\n");
    }
}

/* IMPORTANT:
 * The UniPro event handler callback runs in the context of the UniPro IRQ. */
static void unipro_evt_handler(enum unipro_event evt)
{
    uint32_t rc;
    uint32_t v = 0;
    int err;

    llvdbg("event=%d\n", evt);

    switch (evt) {
    case UNIPRO_EVT_LUP_DONE:
        svc_send_event(SVC_EVENT_UNIPRO_LINK_UP, 0, 0, 0);
        break;
    case UNIPRO_EVT_LINK_LOST:
        rc = unipro_attr_local_read(TSB_DME_LINKLOSTIND, &v, 0);
        llvdbg("rc=%d, lost_ind=%x\n", rc, v);

        if (v & 0x1) {
            svc_send_event(SVC_EVENT_UNIPRO_LINK_DOWN,
                (void *)(unsigned int)0,
                (void *)(unsigned int)0,
                (void *)(unsigned int)0);
        }

        break;
    case UNIPRO_EVT_PWRMODE:
        if (g_svc.gearbox) {
            uint32_t pmi_val0 = 0;
            rc = unipro_attr_read(TSB_DME_POWERMODEIND, &pmi_val0, 0, 0);
            llvdbg("rc=%d, pmi0=%x\n", rc, pmi_val0);

            uint32_t pmi_val1 = 0;
            rc = unipro_attr_read(TSB_DME_POWERMODEIND, &pmi_val1, 0, 1);
            llvdbg("rc=%d, pmi1=%x\n", rc, pmi_val1);

            if (pmi_val0 == 2 && pmi_val1 == 4) {
                llvdbg("Powermode change successful\n");
                err = 0;
            } else {
                llvdbg("Powermode change failed\n");
                err = -1;
            }

            svc_send_event(SVC_EVENT_GEAR_SHIFT_DONE,
                (void *)(unsigned int)err,
                (void *)(unsigned int)0,
                (void *)(unsigned int)0);
        }
        break;
    case UNIPRO_EVT_PHY_ERROR:
        rc = unipro_attr_local_read(TSB_DME_ERRORPHYIND, &v, 0);
        v &= 1;
        if (v) {
            llvdbg("rc=%d, phy err=%x\n", rc, v);
            _svc_bitmask_to_stats(&g_unipro_stats.phy_lane_err, v);
        }
        break;
    case UNIPRO_EVT_PA_ERROR:
        rc = unipro_attr_local_read(TSB_DME_ERRORPAIND, &v, 0);
        v &= 0x3;
        if (v) {
            llvdbg("rc=%d, pa err=%x\n", rc, v);
            _svc_bitmask_to_stats(&g_unipro_stats.pa_lane_reset_tx, v);
        }
        break;
    case UNIPRO_EVT_D_ERROR:
        rc = unipro_attr_local_read(TSB_DME_ERRORDIND, &v, 0);
        v &= 0x7fff;
        if (v) {
            llvdbg("rc=%d, d err=%x\n", rc, v);
            _svc_bitmask_to_stats(&g_unipro_stats.d_nac_received, v);
        }
        break;
    case UNIPRO_EVT_N_ERROR:
        rc = unipro_attr_local_read(TSB_DME_ERRORNIND, &v, 0);
        v &= 0x7;
        if (v) {
            llvdbg("rc=%d, n err=%x\n", rc, v);
            _svc_bitmask_to_stats(&g_unipro_stats.n_unsupported_header_type, v);
        }
        break;
    case UNIPRO_EVT_T_ERROR:
        rc = unipro_attr_local_read(TSB_DME_ERRORTIND, &v, 0);
        v &= 0x7f;
        if (v) {
            llvdbg("rc=%d, t err=%x\n", rc, v);
            _svc_bitmask_to_stats(&g_unipro_stats.t_unsupported_header_type, v);
        }
        break;
    case UNIPRO_EVT_PAINIT_ERROR: {
        bool lost = false;

        rc = unipro_attr_local_read(TSB_DME_ERRORPHYIND, &v, 0);
        v &= 0x1;
        if (v) {
            llvdbg("rc=%d, phy err=%x\n", rc, v);
            _svc_bitmask_to_stats(&g_unipro_stats.phy_lane_err, v);
            lost = true;
        }

        rc = unipro_attr_local_read(TSB_DME_ERRORPAIND, &v, 0);
        v &= 0x3;
        if (v) {
            llvdbg("rc=%d, pa err=%x\n", rc, v);
            _svc_bitmask_to_stats(&g_unipro_stats.pa_lane_reset_tx, v);
            lost = true;
        }

        rc = unipro_attr_local_read(TSB_DME_ERRORDIND, &v, 0);
        v &= 0x7fff;
        if (v) {
            llvdbg("rc=%d, d err=%x\n", rc, v);
            _svc_bitmask_to_stats(&g_unipro_stats.d_nac_received, v);
            lost = true;
        }

        if (lost) {
            llvdbg("Link lost\n");
            svc_send_event(SVC_EVENT_UNIPRO_LINK_DOWN,
                (void *)(unsigned int)0,
                (void *)(unsigned int)0,
                (void *)(unsigned int)0);
        }

        break;
    }
    case UNIPRO_EVT_MAILBOX: {
        v = TSB_MAIL_RESET;
        rc = unipro_attr_local_read(TSB_MAILBOX, &v, 0);
        llvdbg("rc=%d, mbox_val=%d\n", rc, v);

        if (rc) {
        }

#if CONFIG_UNIPRO_P2P_APBA
        if (v == TSB_MAIL_READY_OTHER) {
            llvdbg("Mod detected\n");
            svc_send_event(SVC_EVENT_MOD_DETECTED,
                (void *)(unsigned int)0,
                (void *)(unsigned int)0,
                (void *)(unsigned int)0);
        } else {
            // TODO: Handle error case.
            llvdbg("Mod NOT detected\n");
        }
#elif CONFIG_UNIPRO_P2P_APBE
        llvdbg("Connected cport %d\n", v-1);
        svc_send_event(SVC_EVENT_CPORTS_DONE, (void *)(v-1), 0, 0);
#endif

        break;
    }
    default:
        break;
    }

    vdbg("done\n");
}

static void svc_unipro_link_timeout_handler(svc_timer_t timerid) {
    vdbg("\n");
    svc_send_event(SVC_EVENT_UNIPRO_LINK_TIMEOUT, (void *)timerid, NULL, NULL);
}

static void svc_mod_detect_timeout_handler(svc_timer_t timerid) {
    vdbg("\n");
    svc_send_event(SVC_EVENT_MOD_TIMEOUT, (void *)timerid, NULL, NULL);
}

static void svc_stats_timeout_handler(svc_timer_t timerid) {
    vdbg("\n");
    svc_send_event(SVC_EVENT_SEND_STATS, 0, 0, 0);
}

/* true - keep retrying. false - retry exhausted */
static bool handle_mod_retry(svc_timer_t tid) {

    if (tid != g_svc.mod_detect_tid) {
        vdbg("ignore tid\n");
        return true;
    }

    g_svc.mod_detect_retry++;

    if (g_svc.mod_detect_retry == SVC_MOD_DETECT_RETRY) {
        lldbg("Mod detection timeout. Giving up.\n");
        svc_delete_timer(g_svc.mod_detect_tid);
        g_svc.mod_detect_tid = SVC_TIMER_INVALID;
        unipro_stop();
        mhb_send_pm_status_not(MHB_PM_STATUS_PEER_DISCONNECTED);
    } else {
        lldbg("Mod detection timeout. Retrying (%d)\n", g_svc.mod_detect_retry);

        unipro_reset();
        if (!svc_restart_timer(tid)) {
            /* TODO: handle error case */
            dbg("ERROR: restart timer failed\n");
        }
        mhb_send_pm_status_not(MHB_PM_STATUS_PEER_RESET);
    }

    return (g_svc.mod_detect_retry < SVC_MOD_DETECT_RETRY) ? true : false;
}

inline static bool svc_event_valid(enum svc_event event) {
    return event < SVC_EVENT_MAX;
}

#if CONFIG_DEBUG
static const char * svc_event_to_string(enum svc_event event) {
    if (event >= ARRAY_SIZE(SVC_EVENT_STRINGS)) {
        return NULL;
    }

    return SVC_EVENT_STRINGS[event];
}

static const char *svc_state_to_string(enum svc_state state) {
    if (state >= ARRAY_SIZE(SVC_STATE_STRINGS)) {
        return NULL;
    }

    return SVC_STATE_STRINGS[state];
}
#endif

inline static bool svc_state_valid(enum svc_state state) {
    return state < SVC_STATE_MAX;
}

/* Event Handlers */

static enum svc_state svc_initial__master_started(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    svc_timer_init();

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_ON);

    if (unipro_p2p_is_link_up()) {
        vdbg("link already up\n");
        /* Link is already up. No need poll link status. */
        svc_send_event(SVC_EVENT_UNIPRO_LINK_UP, 0, 0, 0);
    } else {
        vdbg("link not up\n");
        /* Link is not up. Start up link status check polling */
        g_svc.linkup_poll_tid = svc_set_timer(&svc_unipro_link_timeout_handler,
                                              SVC_LINKUP_POLL_INTERVAL);
        if (g_svc.linkup_poll_tid == SVC_TIMER_INVALID) {
            /* TODO: handle error case */
            dbg("ERROR: invalid link timer\n");
        }
    }

    /* Start up a timer for overall Mod detection */
    g_svc.mod_detect_tid = svc_set_timer(&svc_mod_detect_timeout_handler,
                                         SVC_MOD_DETECT_TIMEOUT);
    if (g_svc.mod_detect_tid == SVC_TIMER_INVALID) {
        /* TODO: handle error case */
        dbg("ERROR: invalid mod timer\n");
    }

    return SVC_WAIT_FOR_UNIPRO;
}

static enum svc_state svc_initial__slave_started(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    svc_timer_init();

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_ON);

    unipro_init_with_event_handler(unipro_evt_handler);

    return SVC_SLAVE_WAIT_FOR_UNIPRO;
}

static enum svc_state svc_wf_slave_unipro__link_up(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    unipro_p2p_detect_linkloss(true);
    tsb_unipro_set_init_status(INIT_STATUS_OPERATING);
    unipro_attr_local_write(TSB_DME_ES3_SYSTEM_STATUS_14, CONFIG_VERSION, 0);
    tsb_unipro_mbox_send(TSB_MAIL_READY_OTHER);

    if (g_svc.gearbox) {
        gearbox_link_up(g_svc.gearbox);
    }

#if CONFIG_MHB_IPC_SERVER || CONFIG_MHB_IPC_CLIENT
    ipc_register_unipro();
#endif
#if defined(CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL)
    (void)i2s_unipro_tunnel_unipro_register();
#endif

    return SVC_SLAVE_WAIT_FOR_CPORTS;
}

static enum svc_state svc_wf_cports__link_down(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

#if CONFIG_RAMLOG_SYSLOG
    mhb_ramlog_disable(true /* force */);
#endif

#if defined(CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL)
    (void)i2s_unipro_tunnel_unipro_unregister();
#endif

#if CONFIG_MHB_IPC_SERVER || CONFIG_MHB_IPC_CLIENT
    ipc_unregister_unipro();
#endif

    if (g_svc.gearbox) {
        gearbox_link_down(g_svc.gearbox);
    }

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_DISCONNECTED);
    return SVC_DISCONNECTED;
}

static enum svc_state svc_wf_cports__connected(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    unsigned int cport = (unsigned int)work->parameter0;
    if (cport != CONFIG_MHB_IPC_CPORT_ID) {
        return g_svc.state;
    }

#if CONFIG_RAMLOG_SYSLOG
    mhb_ramlog_enable();
#endif

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_CONNECTED);
    mhb_send_id_not();
    return SVC_SLAVE_CONNECTED;
}

static enum svc_state svc_wf_unipro__link_up(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    /* Unipro link is up. Starting "unipro_init" stuff now */
    unipro_init_with_event_handler(unipro_evt_handler);
    unipro_p2p_detect_linkloss(true);
    tsb_unipro_set_init_status(INIT_STATUS_OPERATING);
    unipro_attr_local_write(TSB_DME_ES3_SYSTEM_STATUS_14, CONFIG_VERSION, 0);
    return SVC_WAIT_FOR_MOD;
}

static enum svc_state svc_wf_unipro__link_timeout(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    svc_timer_t tid = (svc_timer_t)work->parameter0;

    if (tid == g_svc.linkup_poll_tid) {
        if (unipro_p2p_is_link_up()) {
            vdbg("link already up\n");
            svc_delete_timer(g_svc.linkup_poll_tid);
            g_svc.linkup_poll_tid = SVC_TIMER_INVALID;
            svc_send_event(SVC_EVENT_UNIPRO_LINK_UP, 0, 0, 0);
        } else {
            vdbg("link not up\n");
            if (!svc_restart_timer(tid)) {
                /* TODO: handle error case */
                dbg("ERROR: restart timer failed\n");
            }
        }
    }
    return SVC_WAIT_FOR_UNIPRO;
}

static enum svc_state svc_wf_unipro__mod_timeout(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    svc_timer_t tid = (svc_timer_t)work->parameter0;

    return handle_mod_retry(tid) ? SVC_WAIT_FOR_UNIPRO : SVC_DISCONNECTED;
}

static enum svc_state svc_wf_mod__mod_detected(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    svc_delete_timer(g_svc.mod_detect_tid);
    g_svc.mod_detect_tid = SVC_TIMER_INVALID;

    if (g_svc.gearbox) {
        gearbox_link_up(g_svc.gearbox);
    }

#if defined(CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL)
    (void)i2s_unipro_tunnel_unipro_register();
#endif

#if CONFIG_UNIPRO_TEST_CPORTS
# ifdef CONFIG_UNIPRO_TEST_0_CPORT_ID
    lldbg("setup cport=%d\n", CONFIG_UNIPRO_TEST_0_CPORT_ID);
    unipro_p2p_setup_test_connection(CONFIG_UNIPRO_TEST_0_CPORT_ID,
        0 /* test port */, 1 /* APBA to ABPE */, 1 /* E2EFC */);
# endif
# ifdef CONFIG_UNIPRO_TEST_1_CPORT_ID
    lldbg("setup cport=%d\n", CONFIG_UNIPRO_TEST_1_CPORT_ID);
    unipro_p2p_setup_test_connection(CONFIG_UNIPRO_TEST_1_CPORT_ID,
        1 /* test port */, 0 /* APBE to APBA */, 1 /* E2EFC */);
# endif
#endif

#if CONFIG_MHB_IPC_SERVER || CONFIG_MHB_IPC_CLIENT
    ipc_register_unipro();
#endif

#if CONFIG_RAMLOG_SYSLOG
    mhb_ramlog_enable();
#endif

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_CONNECTED);
    mhb_send_id_not();
    return SVC_CONNECTED;
}

static enum svc_state svc_wf_mod__mod_timeout(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    svc_timer_t tid = (svc_timer_t)work->parameter0;

    if (handle_mod_retry(tid)) {
        vdbg("retry\n");

        /* APBE will be reset && local unipro block is restarted.
           Need to wait for unipro link up again */
        g_svc.linkup_poll_tid = svc_set_timer(&svc_unipro_link_timeout_handler,
                                              SVC_LINKUP_POLL_INTERVAL);
        if (g_svc.linkup_poll_tid == SVC_TIMER_INVALID) {
            /* TODO: handle error case */
            dbg("ERROR: invalid link timer\n");
        }
        return SVC_WAIT_FOR_UNIPRO;
    }

    /* giving up */
    return SVC_DISCONNECTED;
}

static enum svc_state svc_connected__link_down(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

#if CONFIG_RAMLOG_SYSLOG
    mhb_ramlog_disable(true /* force */);
#endif

#if defined(CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL)
    (void)i2s_unipro_tunnel_unipro_unregister();
#endif

#if CONFIG_MHB_IPC_SERVER || CONFIG_MHB_IPC_CLIENT
    ipc_unregister_unipro();
#endif

    if (g_svc.gearbox) {
        gearbox_link_down(g_svc.gearbox);
    }

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_DISCONNECTED);
    return SVC_DISCONNECTED;
}

static enum svc_state svc__link_down(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    /* Cancel timers (if running). */
    if (g_svc.linkup_poll_tid != SVC_TIMER_INVALID) {
        svc_delete_timer(g_svc.linkup_poll_tid);
        g_svc.linkup_poll_tid = SVC_TIMER_INVALID;
    }

    if (g_svc.mod_detect_tid != SVC_TIMER_INVALID) {
        svc_delete_timer(g_svc.mod_detect_tid);
        g_svc.mod_detect_tid = SVC_TIMER_INVALID;
    }

#if CONFIG_RAMLOG_SYSLOG
    mhb_ramlog_disable(true /* force */);
#endif

#if defined(CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL)
    (void)i2s_unipro_tunnel_unipro_unregister();
#endif

#if CONFIG_MHB_IPC_SERVER || CONFIG_MHB_IPC_CLIENT
    ipc_unregister_unipro();
#endif

    if (g_svc.gearbox) {
        gearbox_link_down(g_svc.gearbox);
    }

    /* Disable UniPro interrupt events. */
    unipro_p2p_detect_linkloss(false);

    /* Reset the UniPro link. */
    unipro_reset();

    g_svc.linkup_poll_tid = svc_set_timer(&svc_unipro_link_timeout_handler,
                                          SVC_LINKUP_POLL_INTERVAL);
    if (g_svc.linkup_poll_tid == SVC_TIMER_INVALID) {
        /* TODO: handle error case */
        dbg("ERROR: invalid link timer\n");
    }

    /* Start up a timer for overall Mod detection */
    g_svc.mod_detect_tid = svc_set_timer(&svc_mod_detect_timeout_handler,
                                         SVC_MOD_DETECT_TIMEOUT);
    if (g_svc.mod_detect_tid == SVC_TIMER_INVALID) {
        /* TODO: handle error case */
        dbg("ERROR: invalid mod timer\n");
    }

    return SVC_WAIT_FOR_UNIPRO;
}

static enum svc_state svc_connected__gear_shifted(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    int err = (int)work->parameter0;
    gearbox_shift_complete(g_svc.gearbox, err);
    return g_svc.state;
}

static enum svc_state svc__test_mode(struct svc *svc, struct svc_work *work) {
#if CONFIG_ARCH_BOARD_APBA
    factory_mode((uint32_t)work->parameter0);
#endif

    return SVC_TEST_MODE;
}

static enum svc_state svc__queue_stats(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    if (g_svc.stats_tid != SVC_TIMER_INVALID) {
        /* The stats timer is already set to expire.  No need to start it again. */
        vdbg("stats timer already running\n");
        return g_svc.state;
    }

    g_svc.stats_tid = svc_set_timer(&svc_stats_timeout_handler, SVC_STATS_TIMEOUT);
    if (g_svc.stats_tid == SVC_TIMER_INVALID) {
        vdbg("ERROR: Failed to send stats.\n");
    }

    return g_svc.state;
}

static enum svc_state svc__send_stats(struct svc *svc, struct svc_work *work) {
    vdbg("\n");

    if (g_svc.stats_tid != SVC_TIMER_INVALID) {
        svc_delete_timer(g_svc.stats_tid);
        g_svc.stats_tid = SVC_TIMER_INVALID;
    }

    mhb_send_unipro_stats_not();
    g_svc.stats_pending = false;

    return g_svc.state;
}

/* State Table */

static const svc_event_handler SVC_STATES[SVC_STATE_MAX][SVC_EVENT_MAX] = {
    /* SVC_WAIT_FOR_AP */
    {
        svc_initial__master_started, /* SVC_EVENT_MASTER_STARTED */
        svc_initial__slave_started,  /* SVC_EVENT_SLAVE_STARTED */
        NULL,                        /* SVC_EVENT_TEST_MODE_STARTED */
        NULL,                        /* SVC_EVENT_MOD_DETECTED */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_TIMEOUT */
        NULL,                        /* SVC_EVENT_MOD_TIMEOUT */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_UP */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_DOWN */
        NULL,                        /* SVC_EVENT_GEAR_SHIFT_DONE */
        NULL,                        /* SVC_EVENT_QUEUE_STATS */
        NULL,                        /* SVC_EVENT_SEND_STATS */
        NULL,                        /* SVC_EVENT_CPORTS_DONE */
    },
    /* SVC_SLAVE_WAIT_FOR_UNIPRO */
    {
        NULL,                        /* SVC_EVENT_MASTER_STARTED */
        NULL,                        /* SVC_EVENT_SLAVE_STARTED */
        svc__test_mode,              /* SVC_EVENT_TEST_MODE_STARTED */
        NULL,                        /* SVC_EVENT_MOD_DETECTED */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_TIMEOUT */
        NULL,                        /* SVC_EVENT_MOD_TIMEOUT */
        svc_wf_slave_unipro__link_up,/* SVC_EVENT_UNIPRO_LINK_UP */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_DOWN */
        NULL,                        /* SVC_EVENT_GEAR_SHIFT_DONE */
        svc__queue_stats,            /* SVC_EVENT_QUEUE_STATS */
        svc__send_stats,             /* SVC_EVENT_SEND_STATS */
        NULL,                        /* SVC_EVENT_CPORTS_DONE */
    },
    /* SVC_SLAVE_WAIT_FOR_CPORTS */
    {
        NULL,                        /* SVC_EVENT_MASTER_STARTED */
        NULL,                        /* SVC_EVENT_SLAVE_STARTED */
        svc__test_mode,              /* SVC_EVENT_TEST_MODE_STARTED */
        NULL,                        /* SVC_EVENT_MOD_DETECTED */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_TIMEOUT */
        NULL,                        /* SVC_EVENT_MOD_TIMEOUT */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_UP */
        svc_wf_cports__link_down,    /* SVC_EVENT_UNIPRO_LINK_DOWN */
        NULL,                        /* SVC_EVENT_GEAR_SHIFT_DONE */
        svc__queue_stats,            /* SVC_EVENT_QUEUE_STATS */
        svc__send_stats,             /* SVC_EVENT_SEND_STATS */
        svc_wf_cports__connected,    /* SVC_EVENT_CPORTS_DONE */
    },
    /* SVC_SLAVE_CONNECTED */
    {
        NULL,                        /* SVC_EVENT_MASTER_STARTED */
        NULL,                        /* SVC_EVENT_SLAVE_STARTED */
        svc__test_mode,              /* SVC_EVENT_TEST_MODE_STARTED */
        NULL,                        /* SVC_EVENT_MOD_DETECTED */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_TIMEOUT */
        NULL,                        /* SVC_EVENT_MOD_TIMEOUT */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_UP */
        svc_connected__link_down,    /* SVC_EVENT_UNIPRO_LINK_DOWN */
        svc_connected__gear_shifted, /* SVC_EVENT_GEAR_SHIFT_DONE */
        svc__queue_stats,            /* SVC_EVENT_QUEUE_STATS */
        svc__send_stats,             /* SVC_EVENT_SEND_STATS */
        NULL,                        /* SVC_EVENT_CPORTS_DONE */
    },
    /* SVC_WAIT_FOR_UNIPRO */
    {
        NULL,                        /* SVC_EVENT_MASTER_STARTED */
        NULL,                        /* SVC_EVENT_SLAVE_STARTED */
        svc__test_mode,              /* SVC_EVENT_TEST_MODE_STARTED */
        NULL,                        /* SVC_EVENT_MOD_DETECTED */
        svc_wf_unipro__link_timeout, /* SVC_EVENT_UNIPRO_LINK_TIMEOUT */
        svc_wf_unipro__mod_timeout,  /* SVC_EVENT_MOD_TIMEOUT */
        svc_wf_unipro__link_up,      /* SVC_EVENT_UNIPRO_LINK_UP */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_DOWN */
        NULL,                        /* SVC_EVENT_GEAR_SHIFT_DONE */
        svc__queue_stats,            /* SVC_EVENT_QUEUE_STATS */
        svc__send_stats,             /* SVC_EVENT_SEND_STATS */
        NULL,                        /* SVC_EVENT_CPORTS_DONE */
    },
    /* SVC_WAIT_FOR_MOD */
    {
        NULL,                        /* SVC_EVENT_MASTER_STARTED */
        NULL,                        /* SVC_EVENT_SLAVE_STARTED */
        svc__test_mode,              /* SVC_EVENT_TEST_MODE_STARTED */
        svc_wf_mod__mod_detected,    /* SVC_EVENT_MOD_DETECTED */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_TIMEOUT */
        svc_wf_mod__mod_timeout,     /* SVC_EVENT_MOD_TIMEOUT */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_UP */
        svc__link_down,              /* SVC_EVENT_UNIPRO_LINK_DOWN */
        NULL,                        /* SVC_EVENT_GEAR_SHIFT_DONE */
        svc__queue_stats,            /* SVC_EVENT_QUEUE_STATS */
        svc__send_stats,             /* SVC_EVENT_SEND_STATS */
        NULL,                        /* SVC_EVENT_CPORTS_DONE */
    },
    /* SVC_CONNECTED */
    {
        NULL,                        /* SVC_EVENT_MASTER_STARTED */
        NULL,                        /* SVC_EVENT_SLAVE_STARTED */
        svc__test_mode,              /* SVC_EVENT_TEST_MODE_STARTED */
        NULL,                        /* SVC_EVENT_MOD_DETECTED */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_TIMEOUT */
        NULL,                        /* SVC_EVENT_MOD_TIMEOUT */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_UP */
        svc__link_down,              /* SVC_EVENT_UNIPRO_LINK_DOWN */
        svc_connected__gear_shifted, /* SVC_EVENT_GEAR_SHIFT_DONE */
        svc__queue_stats,            /* SVC_EVENT_QUEUE_STATS */
        svc__send_stats,             /* SVC_EVENT_SEND_STATS */
        NULL,                        /* SVC_EVENT_CPORTS_DONE */
    },
    /* SVC_DISCONNECTED */
    {
        NULL,                        /* SVC_EVENT_MASTER_STARTED */
        NULL,                        /* SVC_EVENT_SLAVE_STARTED */
        svc__test_mode,              /* SVC_EVENT_TEST_MODE_STARTED */
        NULL,                        /* SVC_EVENT_MOD_DETECTED */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_TIMEOUT */
        NULL,                        /* SVC_EVENT_MOD_TIMEOUT */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_UP */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_DOWN */
        NULL,                        /* SVC_EVENT_GEAR_SHIFT_DONE */
        svc__queue_stats,            /* SVC_EVENT_QUEUE_STATS */
        svc__send_stats,             /* SVC_EVENT_SEND_STATS */
        NULL,                        /* SVC_EVENT_CPORTS_DONE */
    },
    /* SVC_TEST_MODE */
    {
        NULL,                        /* SVC_EVENT_MASTER_STARTED */
        NULL,                        /* SVC_EVENT_SLAVE_STARTED */
        svc__test_mode,              /* SVC_EVENT_TEST_MODE_STARTED */
        NULL,                        /* SVC_EVENT_MOD_DETECTED */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_TIMEOUT */
        NULL,                        /* SVC_EVENT_MOD_TIMEOUT */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_UP */
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_DOWN */
        NULL,                        /* SVC_EVENT_GEAR_SHIFT_DONE */
        NULL,                        /* SVC_EVENT_QUEUE_STATS */
        NULL,                        /* SVC_EVENT_SEND_STATS */
        NULL,                        /* SVC_EVENT_CPORTS_DONE */
    },
};

static void svc_handle_event(struct svc *svc, struct svc_work *work) {
    if (!svc || !work) {
        return;
    }

    const enum svc_event event = work->event;

    if (!svc_event_valid(event)) {
        lldbg("ERROR: invalid event, %d\n", event);
        return;
    }

    enum svc_state state = svc->state;
    if (!svc_state_valid(state)) {
        lldbg("ERROR: invalid state, %d\n", state);
        return;
    }

    vdbg("state: %s (%d), event: %s (%d)\n",
         svc_state_to_string(state), state,
         svc_event_to_string(event), event);

    const svc_event_handler handler = SVC_STATES[state][event];
    if (!handler) {
        lldbg("state: %s (%d), event: %s (%d)\n",
            svc_state_to_string(state), state,
            svc_event_to_string(event), event);
        return;
    }

    const enum svc_state new_state = handler(svc, work);
    if (new_state == state) {
        lldbg("state: %s (%d), event: %s (%d)\n",
            svc_state_to_string(state), state,
            svc_event_to_string(event), event);
    } else {
        if (!svc_state_valid(new_state)) {
            lldbg("ERROR: handler, %p, returned an invalid state, %d\n",
                handler, new_state);
            return;
        }

        lldbg("state: %s (%d), event: %s (%d), new state: %s (%d)\n",
            svc_state_to_string(state), state,
            svc_event_to_string(event), event,
            svc_state_to_string(new_state), new_state);

        svc->state = new_state;
    }
}

static void svc_work_callback(FAR void *arg) {
    struct svc_work *svc_work = (struct svc_work *)arg;
    if (!svc_work) {
        lldbg("ERROR: invalid work\n");
        return;
    }

    vdbg("event: %s (%d), %p %p %p\n",
         svc_event_to_string(svc_work->event), svc_work->event,
         svc_work->parameter0,
         svc_work->parameter1,
         svc_work->parameter2);

    svc_handle_event(svc_work->svc, svc_work);

    kmm_free(svc_work);
}

int svc_send_event(enum svc_event event, void *parameter0, void *parameter1, void *parameter2) {
    llvdbg("event: %s (%d), %p %p %p\n",
         svc_event_to_string(event), event,
         parameter0,
         parameter1,
         parameter2);

    struct svc_work *svc_work = (struct svc_work *)kmm_zalloc(sizeof(*svc_work));
    if (!svc_work) {
        lldbg("ERROR: failed to alloc work: errno=%d\n", get_errno());
        return -1;
    }

    svc_work->svc = &g_svc;
    svc_work->event = event;
    svc_work->parameter0 = parameter0;
    svc_work->parameter1 = parameter1;
    svc_work->parameter2 = parameter2;

    int ret = work_queue(HPWORK, &svc_work->work, svc_work_callback, svc_work, 0 /* delay */);
    if (ret) {
        lldbg("ERROR: failed to queue work: errno=%d\n", get_errno());
        return ret;
    }

    return ret;
}

int svc_init(void) {
    vdbg("\n");

    g_svc.state = SVC_WAIT_FOR_AP;
    g_svc.mod_detect_tid = SVC_TIMER_INVALID;
    g_svc.linkup_poll_tid = SVC_TIMER_INVALID;
    g_svc.mod_detect_retry = 0;

#if CONFIG_MODS_MHB_SERVER_GEARBOX
    g_svc.gearbox = gearbox_initialize();
#endif

    mhb_uart_init();
    mhb_unipro_init(g_svc.gearbox);

    return 0;
}
