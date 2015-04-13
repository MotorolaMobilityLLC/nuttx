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

#include <arch/board/board.h>

#include "string.h"
#include "ara_board.h"
#include "up_debug.h"
#include "interface.h"
#include "tsb_switch.h"
#include "svc.h"

static struct svc the_svc;
struct svc *ara_svc = &the_svc;

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
    uint8_t device_id_0;        // DeviceID 0
    uint8_t cport_0;            // CPort 0
    uint8_t device_id_1;        // DeviceID 1
    uint8_t cport_1;            // CPort 1
};

/*
 * Default routes used on BDB1B demo
 */
#define DEV_ID_APB1             (1)
#define DEV_ID_APB2             (2)
#define DEV_ID_SPRING6          (8)
#define DEMO_GPIO_APB1_CPORT    (0)
#define DEMO_GPIO_APB2_CPORT    (5)
#define DEMO_I2C_APB1_CPORT     (1)
#define DEMO_I2C_APB2_CPORT     (4)
#define DEMO_DSI_APB1_CPORT     (16)
#define DEMO_DSI_APB2_CPORT     (16)

/* Interface name to deviceID mapping table */
static struct svc_interface_device_id devid[] = {
    { "apb1", DEV_ID_APB1 },
    { "apb2", DEV_ID_APB2 },
    { "spring6", DEV_ID_SPRING6 },
};

/* Connections table */
static struct svc_connection conn[] = {
#if defined(CONFIG_SVC_ROUTE_DEFAULT)
    // APB1, CPort 0 <-> APB2, CPort 5, for GPIO
    { DEV_ID_APB1, DEMO_GPIO_APB1_CPORT, DEV_ID_APB2, DEMO_GPIO_APB2_CPORT },
    // APB1, CPort 1 <-> APB2, CPort 4, for I2C
    { DEV_ID_APB1, DEMO_I2C_APB1_CPORT, DEV_ID_APB2, DEMO_I2C_APB2_CPORT },
    // APB1, CPort 16 <-> APB2, CPort 16, for DSI
    { DEV_ID_APB1, DEMO_DSI_APB1_CPORT, DEV_ID_APB2, DEMO_DSI_APB2_CPORT },
#elif defined(CONFIG_SVC_ROUTE_SPRING6_APB2)
    // SPRING6, CPort 0 <-> APB2, CPort 5, for GPIO
    { DEV_ID_SPRING6, DEMO_GPIO_APB1_CPORT, DEV_ID_APB2, DEMO_GPIO_APB2_CPORT },
    // SPRING6, CPort 1 <-> APB2, CPort 4, for I2C
    { DEV_ID_SPRING6, DEMO_I2C_APB1_CPORT, DEV_ID_APB2, DEMO_I2C_APB2_CPORT },
    // SPRING6, CPort 16 <-> APB2, CPort 16, for DSI
    { DEV_ID_SPRING6, DEMO_DSI_APB1_CPORT, DEV_ID_APB2, DEMO_DSI_APB2_CPORT },
#endif
};


static int setup_default_routes(struct tsb_switch *sw) {
    int i, j, rc;
    uint8_t port_id_0, port_id_1;
    bool port_id_0_found, port_id_1_found;
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
        port_id_0 = port_id_1 = 0;
        port_id_0_found = port_id_1_found = false;
        for (j = 0; j < ARRAY_SIZE(devid); j++) {
            if (!devid[j].found)
                continue;

            if (devid[j].device_id == conn[i].device_id_0) {
                port_id_0 = devid[j].port_id;
                port_id_0_found = true;
            }
            if (devid[j].device_id == conn[i].device_id_1) {
                port_id_1 = devid[j].port_id;
                port_id_1_found = true;
            }
        }

        /* If found, create the requested connection */
        if (port_id_0_found && port_id_1_found) {
            dbg_info("Creating connection [%u:%u]->[%u:%u]\n",
                     port_id_0, conn[i].cport_0,
                     port_id_1, conn[i].cport_1);

            /* Update Switch routing table */
            rc = switch_setup_routing_table(sw,
                                            conn[i].device_id_0, port_id_0,
                                            conn[i].device_id_1, port_id_1);
            if (rc) {
                dbg_error("Failed to setup routing table [%u:%u]<->[%u:%u]\n",
                          conn[i].device_id_0, port_id_0,
                          conn[i].device_id_1, port_id_1);
                return -1;
            }

            /* Create connection */
            rc = switch_connection_std_create(sw,
                                              port_id_0,
                                              conn[i].cport_0,
                                              port_id_1,
                                              conn[i].cport_1);
            if (rc) {
                dbg_error("Failed to create connection [%u:%u]<->[%u:%u]\n",
                          port_id_0, conn[i].cport_0,
                          port_id_1, conn[i].cport_1);
                return -1;
            }
        } else {
            dbg_error("Cannot find portIDs for deviceIDs %d and %d\n",
                      port_id_0, port_id_1);
        }
    }

    switch_dump_routing_table(sw);

    return 0;
}


int svc_init(void) {
    struct ara_board_info *info;
    struct tsb_switch *sw;
    int i, rc;

    dbg_info("Initializing SVC\n");

    // Allocate and zero the sw struct
    sw = zalloc(sizeof(struct tsb_switch));
    if (!sw)
        return -ENOMEM;

    /*
     * Board-specific initialization, all boards must define this.
     */
    info = board_init(sw);
    if (!info) {
        dbg_error("%s: No board information provided.\n", __func__);
        goto error0;
    }
    the_svc.board_info = info;

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

    /* Init Switch */
    sw = switch_init(sw,
                     info->sw_1p1,
                     info->sw_1p8,
                     info->sw_reset,
                     info->sw_irq);
    if (!sw) {
        dbg_error("%s: Failed to initialize switch.\n", __func__);
        goto error2;
    }
    the_svc.sw = sw;

    /* Set up default routes */
    rc = setup_default_routes(sw);
    if (rc) {
        dbg_error("%s: Failed to set default routes\n", __func__);
    }

    /*
     * Enable the switch IRQ
     *
     * Note: the IRQ must be enabled after all NCP commands have been sent
     * for the switch and Unipro devices initialization.
     */
    rc = switch_irq_enable(sw, true);
    if (rc && (rc != -EOPNOTSUPP)) {
        goto error2;
    }

    /* Enable interrupts for all Unipro ports */
    for (i = 0; i < SWITCH_PORT_MAX; i++)
        switch_port_irq_enable(sw, i, true);

    return 0;

error2:
    switch_irq_enable(sw, false);
    interface_exit();
error1:
    board_exit(sw);
error0:
    free(sw);
    return -1;
}

void svc_exit(void) {
    if (the_svc.sw)
        switch_exit(the_svc.sw);

    interface_exit();

    if (the_svc.sw)
        board_exit(the_svc.sw);

    free(the_svc.sw);
    the_svc.sw = NULL;
    the_svc.board_info = NULL;
}

