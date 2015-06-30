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

/* state helpers */
#define svcd_state_running() (svc->state == SVC_STATE_RUNNING)
#define svcd_state_stopped() (svc->state == SVC_STATE_STOPPED)
static inline void svcd_set_state(enum svc_state state){
    svc->state = state;
}


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

/*
 * Keep the Device IDs here and the Device IDs descriptions in
 * apps/greybus-utils/Kconfig and apps/greybus-utils/Makefile in sync.
 */
#define DEV_ID_APB1              (1)
#define DEV_ID_APB2              (2)
#define DEV_ID_APB3              (3)
#define DEV_ID_GPB1              (4)
#define DEV_ID_GPB2              (5)
#define DEV_ID_SPRING1           (6)
#define DEV_ID_SPRING2           (7)
#define DEV_ID_SPRING3           (8)
#define DEV_ID_SPRING4           (9)
#define DEV_ID_SPRING5           (10)
#define DEV_ID_SPRING6           (11)
#define DEV_ID_SPRING7           (12)
#define DEV_ID_SPRING8           (13)

#define DSI_APB1_CPORT           (16)
#define DSI_APB2_CPORT           (16)

/* Interface name to deviceID mapping table */
static struct svc_interface_device_id devid[] = {
    { "apb1", DEV_ID_APB1 },
    { "apb2", DEV_ID_APB2 },
    { "apb3", DEV_ID_APB3 },
    { "gpb1", DEV_ID_GPB1 },
    { "gpb2", DEV_ID_GPB2 },
    { "spring1", DEV_ID_SPRING1 },
    { "spring2", DEV_ID_SPRING2 },
    { "spring3", DEV_ID_SPRING3 },
    { "spring4", DEV_ID_SPRING4 },
    { "spring5", DEV_ID_SPRING5 },
    { "spring6", DEV_ID_SPRING6 },
    { "spring7", DEV_ID_SPRING7 },
    { "spring8", DEV_ID_SPRING8 },
};

/* Connections table */
static struct unipro_connection *conn;

static int setup_routes_from_manifest(void)
{
    struct list_head *cports, *iter;
    struct gb_cport *gb_cport;
    uint8_t device_id0;
    int hd_cport_max;
    int hd_cport = 0;

    cports = get_manifest_cports();
    hd_cport_max = list_count(cports) + 1;
    conn = zalloc(hd_cport_max * sizeof(*conn));
    if (!conn) {
        return 0;
    }

#if defined(CONFIG_SVC_ROUTE_DEFAULT)
    device_id0 = DEV_ID_APB1;
#elif defined(CONFIG_SVC_ROUTE_SPRING6_APB2)
    device_id0 = DEV_ID_SPRING6;
#endif

    list_foreach(cports, iter) {
        gb_cport = list_entry(iter, struct gb_cport, list);
        conn[hd_cport].device_id0 = device_id0;
        conn[hd_cport].cport_id0  = hd_cport;
        conn[hd_cport].device_id1 = gb_cport->device_id;
        conn[hd_cport].cport_id1  = gb_cport->id;
        conn[hd_cport].flags      = CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N;
        hd_cport++;
    }

    conn[hd_cport].device_id0 = device_id0;
    conn[hd_cport].cport_id0  = DSI_APB1_CPORT;
    conn[hd_cport].device_id1 = DEV_ID_APB2;
    conn[hd_cport].cport_id1  = DSI_APB2_CPORT;
    conn[hd_cport].flags      = CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N;
    hd_cport++;

    return hd_cport;
}

#define IID_LENGTH 7
static void manifest_enable(unsigned char *manifest_file,
                            int device_id, int manifest_number)
{
    char iid[IID_LENGTH];

    snprintf(iid, IID_LENGTH, "IID-%d", manifest_number + 1);
    enable_manifest(iid, manifest_file, device_id);
}

static void manifest_disable(unsigned char *manifest_file,
                            int device_id, int manifest_number)
{
    char iid[IID_LENGTH];

    snprintf(iid, IID_LENGTH, "IID-%d", manifest_number + 1);
    disable_manifest(iid, manifest_file, device_id);
}

static int setup_default_routes(struct tsb_switch *sw) {
    int i, j, rc;
    int conn_size;
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

                rc = switch_if_dev_id_set(sw, devid[i].port_id,
                                          devid[i].device_id);
                if (rc) {
                    dbg_error("Failed to assign deviceID %u to interface %s\n",
                              devid[i].device_id, devid[i].interface_name);
                    continue;
                } else {
                    dbg_info("Set deviceID %d to interface %s (portID %d)\n",
                             devid[i].device_id, devid[i].interface_name,
                             devid[i].port_id);
                }
            }
        }
    }

    foreach_manifest(manifest_enable);
    conn_size = setup_routes_from_manifest();
    if (!conn_size)
        return -1;

    /* Connections setup */
    for (i = 0; i < conn_size; i++) {
        /* Look up local and peer portIDs for the given deviceIDs */
        port_id_0 = port_id_1 = 0;
        port_id_0_found = port_id_1_found = false;
        for (j = 0; j < ARRAY_SIZE(devid); j++) {
            if (!devid[j].found)
                continue;

            if (devid[j].device_id == conn[i].device_id0) {
                conn[i].port_id0 = port_id_0 = devid[j].port_id;
                port_id_0_found = true;
            }
            if (devid[j].device_id == conn[i].device_id1) {
                conn[i].port_id1 = port_id_1 = devid[j].port_id;
                port_id_1_found = true;
            }
        }

        /* If found, create the requested connection */
        if (port_id_0_found && port_id_1_found) {
            /* Update Switch routing table */
            rc = switch_setup_routing_table(sw,
                                            conn[i].device_id0, port_id_0,
                                            conn[i].device_id1, port_id_1);
            if (rc) {
                dbg_error("Failed to setup routing table [%u:%u]<->[%u:%u]\n",
                          conn[i].device_id0, port_id_0,
                          conn[i].device_id1, port_id_1);
                return -1;
            }

            /* Create connection */
            rc = switch_connection_create(sw, &conn[i]);
            if (rc) {
                dbg_error("Failed to create connection [%u:%u]<->[%u:%u]\n",
                          port_id_0, conn[i].cport_id0,
                          port_id_1, conn[i].cport_id1);
                return -1;
            }
        } else {
            dbg_error("Cannot find portIDs for deviceIDs %d and %d\n",
                      port_id_0, port_id_1);
        }
    }
    free(conn);
    foreach_manifest(manifest_disable);

    switch_dump_routing_table(sw);

    return 0;
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

