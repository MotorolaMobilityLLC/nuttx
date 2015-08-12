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
#include <nuttx/greybus/greybus.h>
#include <nuttx/unipro/unipro.h>

#include <apps/greybus-utils/utils.h>

#include <arch/board/board.h>

#include <sys/wait.h>
#include <apps/nsh.h>

#include "string.h"
#include "ara_board.h"
#include "up_debug.h"
#include "interface.h"
#include "tsb_switch.h"
#include "tsb_switch_driver_es2.h"
#include "svc.h"
#include "vreg.h"
#include "gb_svc.h"

#define SVCD_PRIORITY      (60)
#define SVCD_STACK_SIZE    (2048)
#define SVC_PROTOCOL_CPORT_ID    (4)

static struct svc the_svc;
struct svc *svc = &the_svc;

#define SVC_EVENT_TYPE_READY_AP       0x1
#define SVC_EVENT_TYPE_READY_OTHER    0x2

struct svc_event_ready_other {
    uint8_t port;
};

struct svc_event {
    int type;
    struct list_head events;
    union {
        struct svc_event_ready_other ready_other;
    } data;
};

static struct list_head svc_events;

static struct svc_event *svc_event_create(int type) {
    struct svc_event *event;

    event = malloc(sizeof(*event));
    if (!event) {
        return NULL;
    }

    event->type = type;
    list_init(&event->events);
    return event;
}

static inline void svc_event_destroy(struct svc_event *event) {
    free(event);
}

static int event_cb(struct tsb_switch_event *ev);
static int event_mailbox(struct tsb_switch_event *ev);

static struct tsb_switch_event_listener evl = {
    .cb = event_cb,
};

/**
 * @brief "Acknowledge" a write to the switch's mailbox.  The mailbox is a DME
 * register in the vendor-defined space present on Toshiba bridges. This is used
 * as a notification to the bridge that the SVC has read what the bridge wrote
 * to the switch's mailbox.
 */
static int svc_mailbox_ack(uint8_t portid) {
    int rc;
    uint32_t val;

    rc = switch_dme_set(svc->sw, portid, TSB_MAILBOX, 0, 0);
    if (rc) {
        dbg_error("Failed to ack to port %u\n", portid);
        return rc;
    }
    rc = switch_dme_get(svc->sw, portid, TSB_MAILBOX, 0, &val);
    if (rc) {
        dbg_error("Failed to reread mailbox ack on port %u\n", portid);
        return rc;
    }

    return 0;
}


static int event_cb(struct tsb_switch_event *ev) {

    switch (ev->type) {
    case TSB_SWITCH_EVENT_MAILBOX:
        event_mailbox(ev);
        svc_mailbox_ack(ev->mbox.port);
        break;
    }
    return 0;
}

static int event_mailbox(struct tsb_switch_event *ev) {
    struct svc_event *svc_ev;
    int rc;
    dbg_info("event received: type: %u port: %u val: %u\n", ev->type, ev->mbox.port, ev->mbox.val);
    pthread_mutex_lock(&svc->lock);
    switch (ev->mbox.val) {
    case TSB_MAIL_READY_AP:
        rc = interface_get_id_by_portid(ev->mbox.port);
        if (rc < 0) {
            dbg_error("Unknown module on port %u: %d\n", ev->mbox.port, rc);
            break;
        }

        svc->ap_intf_id = rc;
        break;

    case TSB_MAIL_READY_OTHER:
        svc_ev = svc_event_create(SVC_EVENT_TYPE_READY_OTHER);
        if (!svc_ev) {
            dbg_error("Couldn't create event\n");
        }
        svc_ev->data.ready_other.port = ev->mbox.port;
        list_add(&svc_events, &svc_ev->events);
        break;
    default:
        dbg_error("unexpected mailbox value: %u port: %u", ev->mbox.val, ev->mbox.port)
    }
    pthread_cond_signal(&svc->cv);
    pthread_mutex_unlock(&svc->lock);

    return 0;
}

static int svc_event_init(void) {
    list_init(&svc_events);
    switch_event_register_listener(svc->sw, &evl);
    return 0;
}

static struct unipro_driver svc_greybus_driver = {
    .name = "svcd-greybus",
    .rx_handler = greybus_rx_handler,
};

static int svc_listen(unsigned int cport) {
    return unipro_driver_register(&svc_greybus_driver, cport);
}

static struct gb_transport_backend svc_backend = {
    .init   = unipro_init,
    .send   = unipro_send,
    .listen = svc_listen,
};

static int svc_gb_init(void) {
    gb_init(&svc_backend);
    gb_svc_register(SVC_PROTOCOL_CPORT_ID);
    return gb_listen(SVC_PROTOCOL_CPORT_ID);
}

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
    uint32_t portid = interface_get_portid_by_id(intf_id);

    rc = switch_dme_peer_set(svc->sw, portid,
                             TSB_MAILBOX, 0, cport + 1);
    if (rc) {
        dbg_error("Failed to notify intf %u\n", intf_id);
    }

    while (retries--) {
        rc = switch_dme_peer_get(svc->sw, portid, TSB_MAILBOX, 0, &val);
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

/**
 * @brief Handle AP module boot
 */
static int svc_handle_ap(void) {
    struct unipro_connection svc_conn;
    uint8_t ap_port_id = interface_get_portid_by_id(svc->ap_intf_id);
    int rc;

    dbg_info("Creating initial SVC connection\n");

    /* Assign a device ID. The AP is always GB_SVC_DEVICE_ID = 1 */
    rc = svc_intf_device_id(svc->ap_intf_id, GB_SVC_DEVICE_ID);
    if (rc) {
        dbg_error("Failed to assign device id to AP: %d\n", rc);
    }

    rc = switch_setup_routing_table(svc->sw, SWITCH_DEVICE_ID, SWITCH_PORT_ID,
                                    GB_SVC_DEVICE_ID, ap_port_id);
    if (rc) {
        dbg_error("Failed to set initial SVC route: %d\n", rc);
    }

    svc_conn.port_id0      = SWITCH_PORT_ID;
    svc_conn.device_id0    = SWITCH_DEVICE_ID;
    svc_conn.cport_id0     = SVC_PROTOCOL_CPORT_ID;
    svc_conn.port_id1      = ap_port_id;
    svc_conn.device_id1    = GB_SVC_DEVICE_ID;
    svc_conn.cport_id1     = GB_SVC_CPORT_ID;
    svc_conn.tc            = 0;
    svc_conn.flags         = CPORT_FLAGS_E2EFC | CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N;
    rc = switch_connection_create(svc->sw, &svc_conn);
    if (rc) {
        dbg_error("Failed to create initial SVC connection: %d\n", rc);
    }

    /*
     * Poke the AP module so that it can transmit FCTs
     */
    rc = svc_mailbox_poke(svc->ap_intf_id, GB_SVC_CPORT_ID);
    if (rc) {
        dbg_error("AP did not respond to poke. Timed out.\n");
        return rc;
    }

    /*
     * Now turn on E2EFC on the switch so that it can transmit FCTs.
     */
    rc = tsb_switch_es2_fct_enable(svc->sw);
    if (rc) {
        dbg_error("Failed to enable FCT on switch.\n");
        return rc;
    }

    /*
     * Now start the SVC protocol handshake.
     */
    gb_svc_protocol_version();
    /*
     * Tell the AP what kind of endo it's in, and the AP's intf_id.
     */
    gb_svc_hello(svc->ap_intf_id);

    /*
     * FIXME: Hotplug events should be sent upon receiving a linkup
     * notification. For now, send a fake one for intf_id 2.
     */
    return 0;
}

static int svc_handle_module_ready(uint8_t portid) {
    int rc, intf_id;
    uint32_t unipro_mfg_id, unipro_prod_id, ara_vend_id, ara_prod_id;

    dbg_info("Hotplug event received for port: %u\n", portid);
    intf_id  = interface_get_id_by_portid(portid);
    if (intf_id < 0) {
        return intf_id;
    }

    rc = switch_dme_peer_get(svc->sw, portid, DME_DDBL1_MANUFACTURERID, 0, &unipro_mfg_id);
    if (rc) {
        dbg_error("Failed to read manufacturer id: %d\n", rc);
        return rc;
    }

    rc = switch_dme_peer_get(svc->sw, portid, DME_DDBL1_PRODUCTID, 0, &unipro_prod_id);
    if (rc) {
        dbg_error("Failed to read product id: %d\n", rc);
        return rc;
    }

    /*
     * Ara vendor id and product ID attributes don't exist on ES2 silicon.
     * These are unused for now.
     */
    ara_vend_id = 0xfeedface;
    ara_prod_id = 0xdeadbeef;

    return gb_svc_intf_hotplug(intf_id, unipro_mfg_id, unipro_prod_id,
                               ara_vend_id, ara_prod_id);
}

/**
 * @brief Main event loop processing routine
 */
static int svc_handle_events(void) {
    struct list_head *node, *next;
    struct svc_event *event;

    list_foreach_safe(&svc_events, node, next) {
        event = list_entry(node, struct svc_event, events);
        switch (event->type) {
        case SVC_EVENT_TYPE_READY_OTHER:
            svc_handle_module_ready(event->data.ready_other.port);
            break;
        default:
            dbg_error("Unknown event: %d\n", event->type);
        }

        svc_event_destroy(event);
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

    /* Initialize event system */
    rc = svc_event_init();
    if (rc) {
        goto error3;
    }

    /* Register svc protocol greybus driver*/
    rc = svc_gb_init();
    if (rc) {
        dbg_error("%s: Failed to initialize SVC protocol\n", __func__);
        goto error3;
    }

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

        if (svc->ap_intf_id && !svc->ap_initialized) {
            if (svc_handle_ap()) {
                break;
            }

            dbg_info("AP initialized on interface  %u\n", svc->ap_intf_id);
            svc->ap_initialized = 1;
        }

        if (svc->ap_initialized) {
            svc_handle_events();
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
