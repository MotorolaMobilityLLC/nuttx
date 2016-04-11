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

#include <arch/chip/unipro_p2p.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#include <nuttx/greybus/tsb_unipro.h>
#include <nuttx/i2s_tunnel/i2s_unipro.h>
#include <nuttx/mhb/mhb_protocol.h>

#include <apps/greybus-utils/utils.h>
#include <apps/ice/factory.h>
#include <apps/ice/ipc.h>
#include <apps/ice/utils.h>

#include "mhb_server.h"
#include "sm.h"
#include "sm_timer.h"

const static char *SVC_EVENT_STRINGS[] = {
    TO_STR(SVC_EVENT_MASTER_STARTED),
    TO_STR(SVC_EVENT_SLAVE_STARTED),
    TO_STR(SVC_EVENT_TEST_MODE_STARTED),
    TO_STR(SVC_EVENT_MOD_DETECTED),
    TO_STR(SVC_EVENT_UNIPRO_LINK_TIMEOUT),
    TO_STR(SVC_EVENT_MOD_TIMEOUT),
    TO_STR(SVC_EVENT_UNIPRO_LINK_UP),
    TO_STR(SVC_EVENT_UNIPRO_LINK_DOWN),
};

enum svc_state {
    SVC_WAIT_FOR_AP,
    SVC_SLAVE_WAIT_FOR_UNIPRO,
    SVC_WAIT_FOR_UNIPRO,
    SVC_WAIT_FOR_MOD,
    SVC_CONNECTED,
    SVC_DISCONNECTED,
    SVC_TEST_MODE,

    SVC_STATE_MAX,
};

const static char *SVC_STATE_STRINGS[] = {
    TO_STR(SVC_WAIT_FOR_AP),
    TO_STR(SVC_SLAVE_WAIT_FOR_UNIPRO),
    TO_STR(SVC_WAIT_FOR_UNIPRO),
    TO_STR(SVC_WAIT_FOR_MOD),
    TO_STR(SVC_CONNECTED),
    TO_STR(SVC_DISCONNECTED),
    TO_STR(SVC_TEST_MODE),
};

struct svc {
    enum svc_state state;
    svc_timer_t mod_detect_tid;
    svc_timer_t linkup_poll_tid;
    uint32_t mod_detect_retry;
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

#define SVC_LINKUP_POLL_INTERVAL 1000 /* ms */
#define SVC_MOD_DETECT_TIMEOUT 10000 /* ms */
#define SVC_MOD_DETECT_RETRY 5

/* Helpers */
static void _pb_unipro_evt_handler(enum unipro_event event)
{
    switch (event) {
    case UNIPRO_EVT_LUP_DONE:
        lldbg("cb start\n");
        svc_send_event(SVC_EVENT_UNIPRO_LINK_UP, 0, 0, 0);
        lldbg("cb done\n");
        break;
    default:
        break;
    }
}

static void svc_unipro_link_timeout_handler(svc_timer_t timerid) {
    svc_send_event(SVC_EVENT_UNIPRO_LINK_TIMEOUT, (void *)timerid, NULL, NULL);
}

static void svc_mod_detect_timeout_handler(svc_timer_t timerid) {
    svc_send_event(SVC_EVENT_MOD_TIMEOUT, (void *)timerid, NULL, NULL);
}

/* true - keep retrying. false - retry exhausted */
static bool handle_mod_retry(svc_timer_t tid) {

    if (tid != g_svc.mod_detect_tid) {
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
        }
        mhb_send_pm_status_not(MHB_PM_STATUS_PEER_RESET);
    }

    return (g_svc.mod_detect_retry < SVC_MOD_DETECT_RETRY) ? true : false;
}

inline static bool svc_event_valid(enum svc_event event) {
    return event < SVC_EVENT_MAX;
}

static const char * svc_event_to_string(enum svc_event event) {
    if (event >= ARRAY_SIZE(SVC_EVENT_STRINGS)) {
        return NULL;
    }

    return SVC_EVENT_STRINGS[event];
}

inline static bool svc_state_valid(enum svc_state state) {
    return state < SVC_STATE_MAX;
}

static const char *svc_state_to_string(enum svc_state state) {
    if (state >= ARRAY_SIZE(SVC_STATE_STRINGS)) {
        return NULL;
    }

    return SVC_STATE_STRINGS[state];
}

/* Event Handlers */

static enum svc_state svc_initial__master_started(struct svc *svc, struct svc_work *work) {
    svc_timer_init();

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_ON);

    if (unipro_p2p_is_link_up()) {
        /* Link is already up. No need poll link status. */
        svc_send_event(SVC_EVENT_UNIPRO_LINK_UP, 0, 0, 0);
    } else {
        /* Link is not up. Start up link status check polling */
        g_svc.linkup_poll_tid = svc_set_timer(&svc_unipro_link_timeout_handler,
                                              SVC_LINKUP_POLL_INTERVAL);
        if (g_svc.linkup_poll_tid == SVC_TIMER_INVALID) {
            /* TODO: handle error case */
        }
    }

    /* Start up a timer for overall Mod detection */
    g_svc.mod_detect_tid = svc_set_timer(&svc_mod_detect_timeout_handler,
                                         SVC_MOD_DETECT_TIMEOUT);
    if (g_svc.mod_detect_tid == SVC_TIMER_INVALID) {
        /* TODO: handle error case */
    }

    return SVC_WAIT_FOR_UNIPRO;
}

static enum svc_state svc_initial__slave_started(struct svc *svc, struct svc_work *work) {
    svc_timer_init();

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_ON);

    unipro_init_with_event_handler(_pb_unipro_evt_handler);

    return SVC_SLAVE_WAIT_FOR_UNIPRO;
}

static enum svc_state svc_wf_slave_unipro__link_up(struct svc *svc, struct svc_work *work) {
    tsb_unipro_set_init_status(INIT_STATUS_OPERATING);
    tsb_unipro_mbox_send(TSB_MAIL_READY_OTHER);

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_CONNECTED);

#if CONFIG_ICE_IPC_SERVER || CONFIG_ICE_IPC_CLIENT
    ipc_register_unipro();
#endif
#if defined(CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL)
    (void)i2s_unipro_tunnel_unipro_register();
#endif

    return SVC_CONNECTED;
}

static enum svc_state svc_wf_unipro__link_up(struct svc *svc, struct svc_work *work) {
    /* Unipro link is up. Starting "unipro_init" stuff now */
    unipro_init();
    unipro_p2p_detect_linkloss();
    return SVC_WAIT_FOR_MOD;
}

static enum svc_state svc_wf_unipro__link_timeout(struct svc *svc, struct svc_work *work) {
    svc_timer_t tid = (svc_timer_t)work->parameter0;

    if (tid == g_svc.linkup_poll_tid) {
        if (unipro_p2p_is_link_up()) {
            svc_delete_timer(g_svc.linkup_poll_tid);
            g_svc.linkup_poll_tid = SVC_TIMER_INVALID;
            svc_send_event(SVC_EVENT_UNIPRO_LINK_UP, 0, 0, 0);
        } else {
            if (!svc_restart_timer(tid)) {
                /* TODO: handle error case */
            }
        }
    }
    return SVC_WAIT_FOR_UNIPRO;
}

static enum svc_state svc_wf_unipro__mod_timeout(struct svc *svc, struct svc_work *work) {
    svc_timer_t tid = (svc_timer_t)work->parameter0;

    return handle_mod_retry(tid) ? SVC_WAIT_FOR_UNIPRO : SVC_DISCONNECTED;
}

static enum svc_state svc_wf_mod__mod_detected(struct svc *svc, struct svc_work *work) {
    svc_delete_timer(g_svc.mod_detect_tid);
    g_svc.mod_detect_tid = SVC_TIMER_INVALID;

#if CONFIG_ICE_IPC_SERVER || CONFIG_ICE_IPC_CLIENT
    ipc_register_unipro();
#endif
#if defined(CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL)
    (void)i2s_unipro_tunnel_unipro_register();
#endif
#if CONFIG_ICE_IPC_SERVER
    lldbg("setup cport=%d\n", CONFIG_ICE_IPC_CPORT_ID);
    unipro_p2p_setup_connection(CONFIG_ICE_IPC_CPORT_ID);
#endif
#if CONFIG_UNIPRO_TEST_0_CPORT_ID
    lldbg("setup cport=%d\n", CONFIG_UNIPRO_TEST_0_CPORT_ID);
    unipro_p2p_setup_test_connection(CONFIG_UNIPRO_TEST_0_CPORT_ID,
        0 /* test port */, 1 /* APBA to ABPE */, 1 /* E2EFC */);
#endif
#if CONFIG_UNIPRO_TEST_1_CPORT_ID
    lldbg("setup cport=%d\n", CONFIG_UNIPRO_TEST_1_CPORT_ID);
    unipro_p2p_setup_test_connection(CONFIG_UNIPRO_TEST_1_CPORT_ID,
        1 /* test port */, 0 /* APBE to APBA */, 1 /* E2EFC */);
#endif

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_CONNECTED);
    return SVC_CONNECTED;
}

static enum svc_state svc_wf_mod__mod_timeout(struct svc *svc, struct svc_work *work) {
    svc_timer_t tid = (svc_timer_t)work->parameter0;

    if (handle_mod_retry(tid)) {
        /* APBE will be reset && local unipro block is restarted.
           Need to wait for unipro link up again */
        g_svc.linkup_poll_tid = svc_set_timer(&svc_unipro_link_timeout_handler,
                                              SVC_LINKUP_POLL_INTERVAL);
        if (g_svc.linkup_poll_tid == SVC_TIMER_INVALID) {
            /* TODO: handle error case */
        }
        return SVC_WAIT_FOR_UNIPRO;
    }

    /* giving up */
    return SVC_DISCONNECTED;
}

static enum svc_state svc_connected__link_down(struct svc *svc, struct svc_work *work) {
    if (work->parameter0) {
        struct p2p_link_err_reason* reason = (struct p2p_link_err_reason *)work->parameter0;
        if (reason->link_lost_ind) {
            lldbg("Link Lost - 0x%x\n", reason->link_lost_ind);
        }
        if (reason->phy_ind) {
            lldbg("Phy Error - 0x%x\n", reason->phy_ind);
        }
        if (reason->pa_ind) {
            lldbg("PA Error - 0x%x\n", reason->pa_ind);
        }
        if (reason->d_ind) {
            lldbg("D Error - 0x%x\n", reason->d_ind);
        }
        if (reason->n_ind) {
            lldbg("N Error - 0x%x\n", reason->n_ind);
        }
        if (reason->t_ind) {
            lldbg("T Error - 0x%x\n", reason->t_ind);
        }
    }

#if CONFIG_ICE_IPC_SERVER || CONFIG_ICE_IPC_CLIENT
    ipc_unregister_unipro();
#endif

    mhb_send_pm_status_not(MHB_PM_STATUS_PEER_DISCONNECTED);
    return SVC_DISCONNECTED;
}

static enum svc_state svc__test_mode(struct svc *svc, struct svc_work *work) {
#if CONFIG_ICE_FACTORY
    factory_mode((uint32_t)work->parameter0);
#endif

    return SVC_TEST_MODE;
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
        NULL,                        /* SVC_EVENT_UNIPRO_LINK_DOWN */
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
        svc_connected__link_down,    /* SVC_EVENT_UNIPRO_LINK_DOWN */
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

    const svc_event_handler handler = SVC_STATES[state][event];
    if (!handler) {
        lldbg("state: %s (%d), event: %s (%d) handler: %p\n",
            svc_state_to_string(state), state,
            svc_event_to_string(event), event,
            handler);
        return;
    }

    const enum svc_state new_state = handler(svc, work);
    if (new_state == state) {
        lldbg("state: %s (%d), event: %s (%d) handler: %p\n",
            svc_state_to_string(state), state,
            svc_event_to_string(event), event,
            handler);
    } else {
        if (!svc_state_valid(new_state)) {
            lldbg("ERROR: handler, %p, returned an invalid state, %d\n",
                handler, new_state);
            return;
        }

        lldbg("state: %s (%d), event: %s (%d), handler=%p, new state: %s (%d)\n",
            svc_state_to_string(state), state,
            svc_event_to_string(event), event,
            handler,
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

    svc_handle_event(svc_work->svc, svc_work);

    kmm_free(svc_work);
}

int svc_send_event(enum svc_event event, void *parameter0, void *parameter1, void *parameter2) {
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
    g_svc.state = SVC_WAIT_FOR_AP;
    g_svc.mod_detect_tid = SVC_TIMER_INVALID;
    g_svc.linkup_poll_tid = SVC_TIMER_INVALID;
    g_svc.mod_detect_retry = 0;

    mhb_uart_init();
    mhb_unipro_init();

    return 0;
}
