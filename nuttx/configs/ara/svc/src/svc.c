/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 */

#define DBG_COMP DBG_SVC

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

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
 * Default routes used on BDB1B demo
 */
#define PORT_ID_APB1            (0)
#define PORT_ID_APB2            (1)
#define DEV_ID_APB1             (1)
#define DEV_ID_APB2             (2)
#define DEMO_GPIO_APB1_CPORT    (0)
#define DEMO_GPIO_APB2_CPORT    (5)
#define DEMO_I2C_APB1_CPORT     (1)
#define DEMO_I2C_APB2_CPORT     (4)

static int setup_default_routes(struct tsb_switch *sw) {
    int rc;

    /* Setup hard-coded default routes for greybus testing */
    rc = switch_if_dev_id_set(sw, PORT_ID_APB1, DEV_ID_APB1);
    if (rc) {
        dbg_error("Failed to assign device id %u to port %u\n",
                  DEV_ID_APB1,
                  PORT_ID_APB1);
        return -1;
    }
    rc = switch_if_dev_id_set(sw, PORT_ID_APB2, DEV_ID_APB2);
    if (rc) {
        dbg_error("Failed to assign device id %u to port %u\n",
                  DEV_ID_APB2,
                  PORT_ID_APB2);
        return -1;
    }
    rc = switch_connection_std_create(sw,
                                      PORT_ID_APB1,
                                      DEMO_GPIO_APB1_CPORT,
                                      PORT_ID_APB2,
                                      DEMO_GPIO_APB2_CPORT);
    if (rc) {
        dbg_error("Failed to create connection [%u:%u]->[%u:%u]\n",
                  PORT_ID_APB1, DEMO_GPIO_APB1_CPORT,
                  PORT_ID_APB2, DEMO_GPIO_APB2_CPORT);
        return -1;
    }
    rc = switch_connection_std_create(sw,
                                      PORT_ID_APB1,
                                      DEMO_I2C_APB1_CPORT,
                                      PORT_ID_APB2,
                                      DEMO_I2C_APB2_CPORT);
    if (rc) {
        dbg_error("Failed to create connection [%u:%u]->[%u:%u]\n",
                  PORT_ID_APB1, DEMO_I2C_APB1_CPORT,
                  PORT_ID_APB2, DEMO_I2C_APB2_CPORT);
        return -1;
    }
    switch_dump_routing_table(sw);

    return 0;
}


int svc_init(void) {
    struct ara_board_info *info;
    struct tsb_switch *sw;
    int rc;

    dbg_set_config(DBG_I2C | DBG_SVC | DBG_SWITCH | DBG_UI, DBG_INFO);

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

