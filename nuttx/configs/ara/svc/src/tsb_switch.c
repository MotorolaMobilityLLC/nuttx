/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 * @author: Perry Hung
 */

#define DBG_COMP    DBG_SWITCH
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <errno.h>
#include <string.h>

#include <nuttx/util.h>
#include "stm32.h"

#include "up_debug.h"
#include "tsb_switch.h"

/* Switch power supply times (1V1, 1V8), in us */
#define POWER_SWITCH_ON_STABILISATION_TIME_US  (50000)
#define POWER_SWITCH_OFF_STABILISATION_TIME_US (10000)
#define SWITCH_SETTLE_TIME_S                   (1)

/*
 * CportID and peerCPortID for L4 access by the SVC
 *
 * L4 CPortID: 0 for SVC -> CC, 2 for CC -> SVC
 * Peer CPortID: 3 for SVC -> CC, 2 for CC -> SVC
 */
#define L4_CPORT_SVC_TO_CC          0
#define L4_CPORT_CC_TO_SVC          2
#define L4_PEERCPORT_SVC_TO_CC      3
#define L4_PEERCPORT_CC_TO_SVC      2

/* NCP_SwitchIDSetReq: DIS and IRT fields values */
#define CPORT_ENABLE                0
#define CPORT_DISABLE               1
#define IRT_DISABLE                 0
#define IRT_ENABLE                  1

static inline void dev_ids_update(struct tsb_switch *sw,
                                  uint8_t port_id,
                                  uint8_t dev_id) {
    sw->dev_ids[port_id] = dev_id;
}

static inline uint8_t dev_ids_port_to_dev(struct tsb_switch *sw,
                                          uint8_t port_id) {
    return sw->dev_ids[port_id];
}

static void dev_ids_destroy(struct tsb_switch *sw) {
    memset(sw->dev_ids, 0, sizeof(sw->dev_ids));
}

static void switch_power_on_reset(struct tsb_switch *sw) {
    /*
     * Hold all the lines low while we turn on the power rails.
     */
    stm32_configgpio(sw->vreg_1p1);
    stm32_configgpio(sw->vreg_1p8);
    stm32_configgpio(sw->reset);
    up_udelay(POWER_SWITCH_OFF_STABILISATION_TIME_US);

    /* First 1V1, wait for stabilisation */
    stm32_gpiowrite(sw->vreg_1p1, true);
    up_udelay(POWER_SWITCH_ON_STABILISATION_TIME_US);
    /* Then 1V8, wait for stabilisation */
    stm32_gpiowrite(sw->vreg_1p8, true);
    up_udelay(POWER_SWITCH_OFF_STABILISATION_TIME_US);

    /* release reset */
    stm32_gpiowrite(sw->reset, true);
    sleep(SWITCH_SETTLE_TIME_S);
}

static void switch_power_off(struct tsb_switch *sw) {
    stm32_gpiowrite(sw->vreg_1p1, false);
    stm32_gpiowrite(sw->vreg_1p8, false);
    stm32_gpiowrite(sw->reset, false);
}


/* Switch communication link init and de-init */
static int switch_init_comm(struct tsb_switch *sw)
{
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->init_comm) {
        return -EOPNOTSUPP;
    }
    return drv->init_comm(drv);
}

/*
 * Unipro NCP commands
 */
int switch_dme_set(struct tsb_switch *sw,
                          uint8_t portid,
                          uint16_t attrid,
                          uint16_t select_index,
                          uint32_t attr_value) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->set) {
        return -EOPNOTSUPP;
    }
    return drv->set(drv, portid, attrid, select_index, attr_value);
}

int switch_dme_get(struct tsb_switch *sw,
                          uint8_t portid,
                          uint16_t attrid,
                          uint16_t select_index,
                          uint32_t *attr_value) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->get) {
        return -EOPNOTSUPP;
    }
    return drv->get(drv, portid, attrid, select_index, attr_value);
}

int switch_dme_peer_set(struct tsb_switch *sw,
                               uint8_t portid,
                               uint16_t attrid,
                               uint16_t select_index,
                               uint32_t attr_value) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->peer_set) {
        return -EOPNOTSUPP;
    }
    return drv->peer_set(drv, portid, attrid, select_index, attr_value);
}

int switch_dme_peer_get(struct tsb_switch *sw,
                               uint8_t portid,
                               uint16_t attrid,
                               uint16_t select_index,
                               uint32_t *attr_value) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->peer_get) {
        return -EOPNOTSUPP;
    }
    return drv->peer_get(drv, portid, attrid, select_index, attr_value);
}

/*
 * Routing table configuration commands
 */
static int switch_lut_set(struct tsb_switch *sw,
                          uint8_t addr,
                          uint8_t dst_portid) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->lut_set) {
        return -EOPNOTSUPP;
    }
    return drv->lut_set(drv, addr, dst_portid);
}

static int switch_lut_get(struct tsb_switch *sw,
                          uint8_t addr,
                          uint8_t *dst_portid) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->lut_get) {
        return -EOPNOTSUPP;
    }
    return drv->lut_get(drv, addr, dst_portid);
}

static int switch_dev_id_mask_get(struct tsb_switch *sw,
                                  uint8_t *dst) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->dev_id_mask_get) {
        return -EOPNOTSUPP;
    }
    return drv->dev_id_mask_get(drv, dst);
}

static int switch_dev_id_mask_set(struct tsb_switch *sw,
                                  uint8_t *mask) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->dev_id_mask_set) {
        return -EOPNOTSUPP;
    }
    return drv->dev_id_mask_set(drv, mask);
}

/*
 * Switch internal configuration commands
 */
static int switch_internal_getattr(struct tsb_switch *sw,
                                   uint16_t attrid,
                                   uint32_t *val) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->switch_attr_get) {
        return -EOPNOTSUPP;
    }
    return drv->switch_attr_get(drv, attrid, val);
}

static int switch_internal_setattr(struct tsb_switch *sw,
                                   uint16_t attrid,
                                   uint32_t val) {
    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->switch_attr_set) {
        return -EOPNOTSUPP;
    }
    return drv->switch_attr_set(drv, attrid, val);
}

static int switch_internal_set_id(struct tsb_switch *sw,
                                  uint8_t cportid,
                                  uint8_t peercportid,
                                  uint8_t dis,
                                  uint8_t irt) {

    struct tsb_switch_driver *drv = sw->drv;
    if (!drv->switch_id_set) {
        return -EOPNOTSUPP;
    }
    return drv->switch_id_set(drv, cportid, peercportid, dis, irt);
}

static int switch_cport_connect(struct tsb_switch *sw,
                                uint8_t port_id,
                                uint8_t cport_id,
                                uint8_t peer_port_id,
                                uint8_t peer_cport_id,
                                uint8_t tc,
                                uint8_t flags) {

    int rc = 0;
    uint8_t peer_dev_id = dev_ids_port_to_dev(sw, peer_port_id);

    /* Disable connection */
    rc = switch_dme_peer_set(sw, port_id, T_CONNECTIONSTATE, cport_id, 0x0);
    if (rc) {
        return rc;
    }

    switch_dme_peer_set(sw, port_id, T_CPORTFLAGS, cport_id, flags);
    if (rc) {
        return rc;
    }

    switch_dme_peer_set(sw, port_id, T_TRAFFICCLASS, cport_id, tc);
    if (rc) {
        return rc;
    }

    /* Set peer devid/cportid for this cport */
    switch_dme_peer_set(sw, port_id, T_PEERCPORTID, cport_id, peer_cport_id);
    if (rc) {
        return rc;
    }

    switch_dme_peer_set(sw, port_id, T_PEERDEVICEID, cport_id, peer_dev_id);
    if (rc) {
        return rc;
    }

    /* Enable connection */
    rc = switch_dme_peer_set(sw, port_id, T_CONNECTIONSTATE, cport_id, 1);
    if (rc) {
        return rc;
    }

    return 0;
}

static int switch_cport_disconnect(struct tsb_switch *sw,
                                   uint8_t port_id,
                                   uint8_t cport_id) {
    return switch_dme_peer_set(sw, port_id, T_CONNECTIONSTATE, cport_id, 0x0);
}

static int switch_link_power_set_default(struct tsb_switch *sw, uint8_t port_id) {
    int rc;
    uint32_t val;
    unsigned int i;
    struct pwr_mode_cmds {
        uint16_t attr;
        uint8_t select_index;
        uint32_t val;
    } pwr_mode_cmds[] = {
        {PA_TXGEAR, NCP_SELINDEX_NULL, PA_GEAR},
        {PA_TXTERMINATION, NCP_SELINDEX_NULL, 0x1},
        {PA_HSSERIES, NCP_SELINDEX_NULL, 0x1},
        {PA_ACTIVETXDATALANES, NCP_SELINDEX_NULL, PA_ACTIVE_TX_DATA_LANES_NR},
        {PA_RXGEAR, NCP_SELINDEX_NULL, PA_GEAR},
        {PA_RXTERMINATION, NCP_SELINDEX_NULL, 0x1},
        {PA_ACTIVERXDATALANES, NCP_SELINDEX_NULL, PA_ACTIVE_RX_DATA_LANES_NR},
        {PA_PWRMODEUSERDATA0, NCP_SELINDEX_NULL, 0x1FFF},
        {PA_PWRMODE, NCP_SELINDEX_NULL, PA_FASTMODE_RXTX},
    };

    for (i = 0; i < ARRAY_SIZE(pwr_mode_cmds); i++) {
        rc = switch_dme_set(sw,
                            port_id,
                            pwr_mode_cmds[i].attr,
                            pwr_mode_cmds[i].select_index,
                            pwr_mode_cmds[i].val);
        if (rc) {
            return rc;
        }
    }

    do {
        /* Wait until the power mode change completes */
        switch_dme_get(sw, port_id, DME_POWERMODEIND, NCP_SELINDEX_NULL, &val);
    } while (val != DME_POWERMODEIND_SUCCESS);

    /* Set TSB_MaxSegmentConfig */
    rc = switch_dme_peer_set(sw,
                         port_id,
                         TSB_MAXSEGMENTCONFIG,
                         NCP_SELINDEX_NULL,
                         MAX_SEGMENT_CONFIG);
    if (rc) {
        return rc;
    }

    return 0;
}

/**
 * @brief Assign a device id to a given port id
 */
int switch_if_dev_id_set(struct tsb_switch *sw,
                         uint8_t port_id,
                         uint8_t dev_id) {
    int rc;

    if (port_id >= SWITCH_PORT_MAX) {
        return -EINVAL;
    }

    rc = switch_dme_peer_set(sw, port_id, N_DEVICEID, NCP_SELINDEX_NULL, dev_id);
    if (rc) {
        return rc;
    }

    rc = switch_dme_peer_set(sw, port_id, N_DEVICEID_VALID, NCP_SELINDEX_NULL, 1);
    if (rc) {
        /* do what on failure? */
        return rc;
    }

    rc = switch_lut_set(sw, dev_id, port_id);
    if (rc) {
        /* is it worth undoing deviceid_valid on failure...? */
        return rc;
    }

    /* update the table */
    dev_ids_update(sw, port_id, dev_id);

    return 0;
}

/**
 * @brief Create a connection between two cports
 */
int switch_connection_create(struct tsb_switch *sw,
                             uint8_t port_id1,
                             uint8_t cport_id1,
                             uint8_t port_id2,
                             uint8_t cport_id2,
                             uint8_t tc,
                             uint8_t flags) {
    int rc;

    dbg_verbose("Creating connection: [%u:%u]<->[%u:%u] TC: %u Flags: %x\n",
                port_id1,
                cport_id1,
                port_id2,
                cport_id2,
                tc,
                flags);
    rc = switch_cport_connect(sw,
                              port_id1,
                              cport_id1,
                              port_id2,
                              cport_id2,
                              tc,
                              flags);
    if (rc) {
        return rc;
    }

    rc = switch_cport_connect(sw,
                              port_id2,
                              cport_id2,
                              port_id1,
                              cport_id1,
                              tc,
                              flags);
    if (rc) {
        goto err0;
    }

    rc = switch_link_power_set_default(sw, port_id1);
    if (rc) {
        goto err1;
    }

    rc = switch_link_power_set_default(sw, port_id2);
    if (rc) {
        goto err1;
    }

    return 0;

err1:
    switch_cport_disconnect(sw, port_id2, cport_id2);
err0:
    switch_cport_disconnect(sw, port_id1, cport_id1);
    dbg_verbose("%s: Connection setup failed. [%u:%u]<->[%u:%u] TC: %u Flags: %x rc: %u\n",
                __func__,
                port_id1,
                cport_id1,
                port_id2,
                cport_id2,
                tc,
                flags,
                rc);
    return rc;
}

static int switch_detect_devices(struct tsb_switch *sw,
                                 uint32_t *link_status)
{
    uint32_t attr_value = 0;
    int i, j;
    uint32_t attr_to_read[] = {
        /* DME_DDBL1 */
        /*  Revision,       expected 0x0010 */
        DME_DDBL1_REVISION,
        /*  Level,          expected 0x0003 */
        DME_DDBL1_LEVEL,
        /*  deviceClass,    expected 0x0000 */
        DME_DDBL1_DEVICECLASS,
        /*  ManufactureID,  expected 0x0126 */
        DME_DDBL1_MANUFACTUREID,
        /*  productID,      expected 0x1000 */
        DME_DDBL1_PRODUCTID,
        /*  length,         expected 0x0008 */
        DME_DDBL1_LENGTH,
        /* DME_DDBL2 VID and PID */
        DME_DDBL2_VID,
        DME_DDBL2_PID,
        /* Data lines */
        PA_CONNECTEDTXDATALANES,
        PA_CONNECTEDRXDATALANES
    };

    /* Read switch link status */
    if (switch_internal_getattr(sw, SWSTA, link_status)) {
        dbg_error("Switch read status failed\n");
        return -1;
    }

    dbg_info("%s: Link status: 0x%x\n", __func__, *link_status);

    /* Get attributes from connected devices */
    for (i = 0; i < SWITCH_PORT_MAX; i++) {
        if (*link_status & (1 << i)) {
            for (j = 0; j < ARRAY_SIZE(attr_to_read); j++) {
                if (switch_dme_peer_get(sw, i, attr_to_read[j],
                                        NCP_SELINDEX_NULL, &attr_value)) {
                    dbg_error("%s: Failed to read attr(0x%x) from portID %d\n",
                              __func__, attr_to_read[j], i);
                } else {
                    dbg_verbose("%s: portID %d: attr(0x%x)=0x%x\n",
                                __func__, i, attr_to_read[j], attr_value);
                }
            }
        }
   }

   return 0;
}

/**
 * @brief Dump routing table to low level console
 */
void switch_dump_routing_table(struct tsb_switch *sw) {
    int i, j, idx, rc;
    uint8_t p = 0, valid_bitmask[16];
    char msg[64];

    rc = switch_dev_id_mask_get(sw, valid_bitmask);
    if (rc) {
        dbg_error("%s() Failed to retrieve routing table.\n", __func__);
        return;
    }

    dbg_info("%s(): Routing table\n", __func__);
    dbg_info("======================================================\n");

    /* Replace invalid entries with 'XX' */
#define CHECK_VALID_ENTRY(entry) \
    (valid_bitmask[15 - ((entry) / 8)] & (1 << ((entry)) % 8))

    for (i = 0; i < 8; i++) {
        /* Build a line with the offset, 8 entries, a '|' then 8 entries */
        idx = 0;
        rc = sprintf(msg, "%3d: ", i * 16);
        if (rc <= 0)
            goto out;
        else
            idx += rc;

        for (j = 0; j < 16; j++) {
            if (CHECK_VALID_ENTRY(i * 16 + j)) {
                switch_lut_get(sw, i * 16 + j, &p);
                rc = sprintf(msg + idx, "%2u ", p);
                if (rc <= 0)
                    goto out;
                else
                    idx += rc;
            } else {
                rc = sprintf(msg + idx, "XX ");
                if (rc <= 0)
                    goto out;
                else
                    idx += rc;
            }

            if (j == 7) {
                rc = sprintf(msg + idx, "| ");
                if (rc <= 0)
                    goto out;
                else
                    idx += rc;
            }
        }

        rc = sprintf(msg + idx, "\n");
        if (rc <= 0)
            goto out;
        else
            idx += rc;
        msg[idx] = 0;

        /* Output the line */
        dbg_info("%s", msg);
    }

out:
    dbg_info("======================================================\n");
}

/**
 * @brief Initialize the switch and set default SVC<->Switch route
 * @param drv switch driver
 * @param vreg_1p1 gpio for 1p1 regulator
 * @param vreg_1p8 gpio for 1p8 regulator
 * @param reset gpio for reset line
 * @param irq gpio for switch irq
 */
struct tsb_switch *switch_init(struct tsb_switch_driver *drv,
                               unsigned int vreg_1p1,
                               unsigned int vreg_1p8,
                               unsigned int reset,
                               unsigned int irq) {
    struct tsb_switch *sw = NULL;
    unsigned int attr_value, link_status;
    int rc;

    dbg_verbose("%s: Initializing switch\n", __func__);
    if (!drv) {
        goto error;
    }

    if (!vreg_1p1 || !vreg_1p8 || !reset || !irq) {
        goto error;
    }

    sw = malloc(sizeof(struct tsb_switch));
    if (!sw) {
        goto error;
    }
    memset(sw, 0, sizeof *sw);

    sw->drv = drv;
    sw->vreg_1p1 = vreg_1p1;
    sw->vreg_1p8 = vreg_1p8;
    sw->reset = reset;
    sw->irq = irq;              // unused for now

    switch_power_on_reset(sw);

    rc = switch_init_comm(sw);
    if (rc && (rc != -EOPNOTSUPP)) {
        goto error;
    }

    /*
     * Sanity check
     */
    if (switch_internal_getattr(sw, SWVER, &attr_value)) {
        dbg_error("Switch probe failed\n");
        goto error;
    }

    /*
     * Set initial SVC deviceId to SWITCH_DEVICE_ID and setup
     * routes for internal L4 access from the SVC
     */
    rc = switch_internal_set_id(sw,
                                L4_CPORT_SVC_TO_CC,
                                L4_PEERCPORT_SVC_TO_CC,
                                CPORT_ENABLE,
                                IRT_ENABLE);
    if (rc) {
        goto error;
    }
    rc = switch_internal_set_id(sw,
                                L4_CPORT_CC_TO_SVC,
                                L4_PEERCPORT_CC_TO_SVC,
                                CPORT_ENABLE,
                                IRT_DISABLE);
    if (rc) {
        goto error;
    }

    rc = switch_detect_devices(sw, &link_status);
    if (rc) {
        goto error;
    }

    return sw;

error:
    if (sw) {
        switch_exit(sw);
    }
    dbg_error("%s: Failed to initialize switch.\n", __func__);

    return NULL;
}


/**
 * @brief Power down and disable the switch
 */
void switch_exit(struct tsb_switch *sw) {
    dbg_verbose("%s: Disabling switch\n", __func__);
    dev_ids_destroy(sw);
    switch_power_off(sw);
    free(sw);
}
