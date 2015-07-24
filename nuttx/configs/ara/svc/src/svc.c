/*
 * Copyright (c) 2015 Google Inc.
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

#define DBG_COMP DBG_SVC

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/greybus/unipro.h>

#include <apps/greybus-utils/utils.h>

#include <arch/board/board.h>

#include <sys/wait.h>
#include <apps/nsh.h>

#include "string.h"
#include "ara_board.h"
#include "up_debug.h"
#include "interface.h"
#include "tsb_switch.h"
#include "svc.h"
#include "vreg.h"

#define SVCD_PRIORITY      (60)
#define SVCD_STACK_SIZE    (2048)

static struct svc the_svc;
struct svc *svc = &the_svc;

/**
 * @brief "Poke" a bridge's mailbox. The mailbox is a DME register in the
 * vendor-defined space present on Toshiba bridges. This is used as a notification
 * to the bridge that a connection has been established and that it can enable
 * transmission of E2EFC credits.
 *
 * Value 'x' means that cport 'x - 1' is ready. When the bridge responds, it
 * clears its own mailbox to notify the SVC that it has received the
 * notification.
 */
static int svc_mailbox_poke(uint8_t intf_id, uint8_t cport) {
    size_t retries = 1000;
    uint32_t val;
    int rc;

    rc = switch_dme_peer_set(svc->sw, interface_get_portid_by_id(intf_id),
                             TSB_MAILBOX, 0, cport + 1);
    if (rc) {
        dbg_error("Failed to notify intf %u\n", intf_id);
    }

    while (retries--) {
        rc = switch_dme_peer_get(svc->sw, 0, TSB_MAILBOX, 0, &val);
        if (!rc && !val) {
            return 0;
        }
    }

    return -ETIMEDOUT;
}

/**
 * @brief Assign a device id given an interface id
 */
int svc_intf_device_id(uint8_t intf_id, uint8_t dev_id) {
    struct tsb_switch *sw = svc->sw;
    int rc;

    rc = switch_if_dev_id_set(sw, interface_get_portid_by_id(intf_id), dev_id);
    if (rc) {
        return rc;
    }

    return interface_set_devid_by_id(intf_id, dev_id);
}

/**
 * @brief Create a UniPro connection
 */
int svc_connection_create(uint8_t intf1_id, uint16_t cport1_id,
                          uint8_t intf2_id, uint16_t cport2_id, u8 tc, u8 flags) {
    struct tsb_switch *sw = svc->sw;
    struct unipro_connection c;
    int rc;

    memset(&c, 0x0, sizeof c);

    c.port_id0      = interface_get_portid_by_id(intf1_id);
    c.device_id0    = interface_get_devid_by_id(intf1_id);
    c.cport_id0     = cport1_id;

    c.port_id1      = interface_get_portid_by_id(intf2_id);
    c.device_id1    = interface_get_devid_by_id(intf2_id);
    c.cport_id1     = cport2_id;

    c.tc            = tc;
    c.flags         = flags;

    rc = switch_connection_create(sw, &c);
    if (rc) {
        return rc;
    }

    if (flags & CPORT_FLAGS_E2EFC) {
        /*
         * Poke bridge mailboxes and wait for them to enable E2EFC.
         * @jira{ENG-376}
         */
        rc = svc_mailbox_poke(intf1_id, cport1_id);
        if (rc) {
            dbg_error("Failed to notify intf %u\n", intf1_id);
            return rc;
        }
        rc = svc_mailbox_poke(intf2_id, cport2_id);
        if (rc) {
            dbg_error("Failed to notify intf %u\n", intf2_id);
            return rc;
        }
    }

    return 0;
}

/**
 * @brief Create a bidirectional route through the switch
 */
int svc_route_create(uint8_t intf1_id, uint8_t dev1_id,
                     uint8_t intf2_id, uint8_t dev2_id) {
    struct tsb_switch *sw = svc->sw;
    int rc;

    rc = switch_setup_routing_table(sw,
                                    dev1_id,
                                    interface_get_portid_by_id(intf1_id),
                                    dev2_id,
                                    interface_get_portid_by_id(intf2_id));
    if (rc) {
        dbg_error("Failed to create route [%u:%u]<->[%u:%u]\n");
        return rc;
    }

    return 0;
}

/* state helpers */
#define svcd_state_running() (svc->state == SVC_STATE_RUNNING)
#define svcd_state_stopped() (svc->state == SVC_STATE_STOPPED)
static inline void svcd_set_state(enum svc_state state){
    svc->state = state;
}

static int svcd_startup(void) {
    struct ara_board_info *info;
    struct tsb_switch *sw;
    int i, rc;

    /*
     * Board-specific initialization, all boards must define this.
     */
    info = board_init();
    if (!info) {
        dbg_error("%s: No board information provided.\n", __func__);
        goto error0;
    }
    svc->board_info = info;
    rc = interface_early_init(info->interfaces,
                              info->nr_interfaces, info->nr_spring_interfaces);
    if (rc < 0) {
        dbg_error("%s: Failed to power off interfaces\n", __func__);
        goto error0;
    }

    /* Init Switch */
    sw = switch_init(&info->sw_data);
    if (!sw) {
        dbg_error("%s: Failed to initialize switch.\n", __func__);
        goto error1;
    }
    svc->sw = sw;

    /* Power on all provided interfaces */
    if (!info->interfaces) {
        dbg_error("%s: No interface information provided\n", __func__);
        goto error2;
    }

    rc = interface_init(info->interfaces,
                        info->nr_interfaces, info->nr_spring_interfaces);
    if (rc < 0) {
        dbg_error("%s: Failed to initialize interfaces\n", __func__);
        goto error2;
    }

    /*
     * FIXME remove when system bootstrap sequence is implemented.
     *
     * HACK: until the system bootstrap sequence is finished, we can't
     * synchronize with the bridges' own initialization sequence. This
     * is breaking GPB2 setup on BDB2B. Add a magic delay until the
     * system bootstrap sequence is implemented.
     */
    up_mdelay(300);

    /*
     * Enable the switch IRQ
     *
     * Note: the IRQ must be enabled after all NCP commands have been sent
     * for the switch and Unipro devices initialization.
     */
    rc = switch_irq_enable(sw, true);
    if (rc && (rc != -EOPNOTSUPP)) {
        goto error3;
    }

    /* Enable interrupts for all Unipro ports */
    for (i = 0; i < SWITCH_PORT_MAX; i++)
        switch_port_irq_enable(sw, i, true);

    return 0;

error3:
    interface_exit();
error2:
    switch_exit(sw);
    svc->sw = NULL;
error1:
    board_exit();
error0:
    return -1;
}

static int svcd_cleanup(void) {
    interface_exit();

    switch_exit(svc->sw);
    svc->sw = NULL;

    board_exit();
    svc->board_info = NULL;

    return 0;
}


static int svcd_main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    int rc = 0;

    pthread_mutex_lock(&svc->lock);
    rc = svcd_startup();
    if (rc < 0) {
        goto done;
    }

    while (!svc->stop) {
        pthread_cond_wait(&svc->cv, &svc->lock);
        /* check to see if we were told to stop */
        if (svc->stop) {
            dbg_verbose("svc stop requested\n");
            break;
        }
    };

    rc = svcd_cleanup();

done:
    svcd_set_state(SVC_STATE_STOPPED);
    pthread_mutex_unlock(&svc->lock);
    return rc;
}

/*
 * System entrypoint. CONFIG_USER_ENTRYPOINT should point to this function.
 */
int svc_init(int argc, char **argv) {
    int rc;

    svc->sw = NULL;
    svc->board_info = NULL;
    svc->svcd_pid = 0;
    svc->stop = 0;
    pthread_mutex_init(&svc->lock, NULL);
    pthread_cond_init(&svc->cv, NULL);
    svcd_set_state(SVC_STATE_STOPPED);

    rc = svcd_start();
    if (rc) {
        return rc;
    }

    /*
     * Now start the shell.
     */
    return nsh_main(argc, argv);
}

int svcd_start(void) {
    int rc;

    pthread_mutex_lock(&svc->lock);
    dbg_info("starting svcd\n");
    if (!svcd_state_stopped()) {
        dbg_info("svcd already started\n");
        pthread_mutex_unlock(&svc->lock);
        return -EBUSY;
    }

    rc = task_create("svcd", SVCD_PRIORITY, SVCD_STACK_SIZE, svcd_main, NULL);
    if (rc == ERROR) {
        dbg_error("failed to start svcd\n");
        return rc;
    }
    svc->svcd_pid = rc;

    svc->stop = 0;
    svcd_set_state(SVC_STATE_RUNNING);
    pthread_mutex_unlock(&svc->lock);

    return 0;
}

void svcd_stop(void) {
    int status;
    int rc;
    pid_t pid_tmp;

    pthread_mutex_lock(&svc->lock);
    dbg_verbose("stopping svcd\n");

    pid_tmp = svc->svcd_pid;

    if (!svcd_state_running()) {
        dbg_info("svcd not running\n");
        pthread_mutex_unlock(&svc->lock);
        return;
    }

    /* signal main thread to stop */
    svc->stop = 1;
    pthread_cond_signal(&svc->cv);
    pthread_mutex_unlock(&svc->lock);

    /* wait for the svcd to stop */
    rc = waitpid(pid_tmp, &status, 0);
    if (rc != pid_tmp) {
        dbg_error("failed to stop svcd\n");
    } else {
        dbg_info("svcd stopped\n");
    }
}

int svc_connect_interfaces(struct interface *iface1, uint16_t cportid1,
                           struct interface *iface2, uint16_t cportid2,
                           uint8_t tc, uint8_t flags) {
    int rc;
    uint8_t devids[2];
    struct tsb_switch *sw = svc->sw;
    struct unipro_connection con;

    if (!iface1 || !iface2) {
        return -EINVAL;
    }

    pthread_mutex_lock(&svc->lock);

    /* Retrieve the interface structures and device IDs for the interfaces. */
    rc = switch_if_dev_id_get(sw, iface1->switch_portid, &devids[0]);
    if (rc) {
        goto error_exit;
    }

    rc = switch_if_dev_id_get(sw, iface2->switch_portid, &devids[1]);
    if (rc) {
        goto error_exit;
    }

    /* Create the route between the two devices. */
    rc = switch_setup_routing_table(sw,
                                    devids[0], iface1->switch_portid,
                                    devids[1], iface2->switch_portid);
    if (rc) {
        dbg_error("Failed to create route: [d=%u,p=%u]<->[d=%u,p=%u]\n",
                  devids[0], iface1->switch_portid,
                  devids[1], iface2->switch_portid);
        goto error_exit;
    }
    /* Create the connection between the two devices. */
    con.port_id0   = iface1->switch_portid;
    con.device_id0 = devids[0];
    con.cport_id0  = cportid1;
    con.port_id1   = iface2->switch_portid;
    con.device_id1 = devids[1];
    con.cport_id1  = cportid2;
    con.tc         = tc;
    con.flags      = flags;
    rc = switch_connection_create(sw, &con);
    if (rc) {
        dbg_error("Failed to create [p=%u,d=%u,c=%u]<->[p=%u,d=%u,c=%u] TC: %u Flags: 0x%x\n",
                  con.port_id0, con.device_id0, con.cport_id0,
                  con.port_id1, con.device_id1, con.cport_id1,
                  con.tc, con.flags);
    }

 error_exit:
    pthread_mutex_unlock(&svc->lock);
    return rc;
}
