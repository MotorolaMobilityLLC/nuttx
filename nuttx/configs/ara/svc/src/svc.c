/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 */

#define DBG_COMP DBG_SVC

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>

#include <arch/board/board.h>

#include "string.h"
#include "ara_board.h"
#include "up_debug.h"
#include "interface.h"
#include "tsb_switch.h"
#include "svc.h"

struct svc {
    struct tsb_switch *sw;
};

static struct svc the_svc;

/*
 * Static connections table
 *
 * The routing and connections setup is made of two tables:
 * 1) The interface to deviceID mapping table. Every interface is given by
 *    its name (provided in the board file). The deviceID to associate to
 *    the interface is freely chosen.
 *    Cf. the console command 'power' for the interfaces naming.
 *    The physical port to the switch is retrieved from the board file
 *    information, there is no need to supply it in the connection tables.
 *
 * 2) The connections table, given by the deviceID and the CPort to setup, on
 *    both local and remote ends of the link. The routing will be setup in a
 *    bidirectional way.
 *
 * Spring interfaces placement on BDB1B/2A:
 *
 *                             8
 *                            (9)
 *          7     5  6
 *    3  2     4        1
 */
struct svc_interface_device_id {
    char *interface_name;       // Interface name
    uint8_t device_id;          // DeviceID
    uint8_t port_id;            // PortID
    bool found;
};

struct svc_connection {
    uint8_t local_device_id;    // Local DeviceID
    uint8_t local_cport;        // Local CPort
    uint8_t peer_device_id;     // Peer DeviceID
    uint8_t peer_cport;         // Peer CPort
};

/*
 * Default routes used on BDB1B demo
 */
#define DEV_ID_APB1             (1)
#define DEV_ID_APB2             (2)
#define DEMO_GPIO_APB1_CPORT    (0)
#define DEMO_GPIO_APB2_CPORT    (5)
#define DEMO_I2C_APB1_CPORT     (1)
#define DEMO_I2C_APB2_CPORT     (4)

/* Interface name to deviceID mapping table */
static struct svc_interface_device_id devid[] = {
    { "apb1", DEV_ID_APB1 },
    { "apb2", DEV_ID_APB2 },
};

/* Connections table */
static struct svc_connection conn[] = {
    // APB1, CPort 0 <-> APB2, CPort 5, for GPIO
    { DEV_ID_APB1, DEMO_GPIO_APB1_CPORT, DEV_ID_APB2, DEMO_GPIO_APB2_CPORT },
    // APB1, CPort 1 <-> APB2, CPort 4, for I2C
    { DEV_ID_APB1, DEMO_I2C_APB1_CPORT, DEV_ID_APB2, DEMO_I2C_APB2_CPORT },
};


static int setup_default_routes(struct tsb_switch *sw) {
    int i, j, rc;
    uint8_t port_id_local, port_id_peer;
    bool port_id_local_found, port_id_peer_found;
    struct interface *iface;

    /*
     * Setup hard-coded default routes from the routing and
     * connection tables
     */

    /* Setup Port <-> deviceID and configure the Switch routing table */
    for (i = 0; i < ARRAY_SIZE(devid); i++) {
        devid[i].found = false;
        /* Retrieve the portID from the interface name */
        interface_foreach(iface, j) {
            if (!strcmp(iface->name, devid[i].interface_name)) {
                devid[i].port_id = iface->switch_portid;
                devid[i].found = true;

                dbg_info("Setting deviceID %d to interface %s (portID %d)\n",
                         devid[i].device_id, devid[i].interface_name,
                         devid[i].port_id);

                rc = switch_if_dev_id_set(sw, devid[i].port_id,
                                          devid[i].device_id);
                if (rc) {
                    dbg_error("Failed to assign deviceID %u to interface %s\n",
                              devid[i].device_id, devid[i].interface_name);
                    continue;
                }
            }
        }
    }

    /* Connections setup */
    for (i = 0; i < ARRAY_SIZE(conn); i++) {
        /* Look up local and peer portIDs for the given deviceIDs */
        port_id_local = port_id_peer = 0;
        port_id_local_found = port_id_peer_found = false;
        for (j = 0; j < ARRAY_SIZE(devid); j++) {
            if (!devid[j].found)
                continue;

            if (devid[j].device_id == conn[i].local_device_id) {
                port_id_local = devid[j].port_id;
                port_id_local_found = true;
            }
            if (devid[j].device_id == conn[i].peer_device_id) {
                port_id_peer = devid[j].port_id;
                port_id_peer_found = true;
            }
        }

        /* If found, create the requested connection */
        if (port_id_local_found && port_id_peer_found) {
            dbg_info("Creating connection [%u:%u]->[%u:%u]\n",
                     port_id_local, conn[i].local_cport,
                     port_id_peer, conn[i].peer_cport);
            rc = switch_connection_std_create(sw,
                                              port_id_local,
                                              conn[i].local_cport,
                                              port_id_peer,
                                              conn[i].peer_cport);
            if (rc) {
                dbg_error("Failed to create connection [%u:%u]->[%u:%u]\n",
                          port_id_local, conn[i].local_cport,
                          port_id_peer, conn[i].peer_cport);
                return -1;
            }
        } else {
            dbg_error("Cannot find portIDs for deviceIDs %d and %d\n",
                      port_id_local, port_id_peer);
        }
    }

    switch_dump_routing_table(sw);

    return 0;
}


int svc_init(void) {
    struct ara_board_info *info;
    struct tsb_switch *sw;
    int rc;

    dbg_info("Initializing SVC\n");
    /*
     * Board-specific initialization, all boards must define this.
     */
    info = board_init();
    if (!info) {
        dbg_error("%s: No board information provided.\n", __func__);
        goto error0;
    }

    /* Power on all provided interfaces */
    if (!info->interfaces) {
        dbg_error("%s: No interface information provided\n", __func__);
        goto error1;
    }

    rc = interface_init(info->interfaces, info->nr_interfaces);
    if (rc < 0) {
        dbg_error("%s: Failed to initialize interfaces\n", __func__);
        goto error1;
    }

    sw = switch_init(info->sw_drv,
                     info->sw_1p1,
                     info->sw_1p8,
                     info->sw_reset,
                     info->sw_irq);
    if (!sw) {
        dbg_error("%s: Failed to initialize switch.\n", __func__);
        goto error2;
    }
    the_svc.sw = sw;

    rc = setup_default_routes(sw);
    if (rc) {
        dbg_error("%s: Failed to set default routes\n", __func__);
    }

    return 0;

error2:
    interface_exit();
error1:
    board_exit();
error0:
    return -1;
}

void svc_exit(void) {
    if (the_svc.sw) {
        switch_exit(the_svc.sw);
        the_svc.sw = NULL;
    }

    interface_exit();
    board_exit();
}

