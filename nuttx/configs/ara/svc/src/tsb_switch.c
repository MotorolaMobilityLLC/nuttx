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

/**
 * @author: Perry Hung
 * @author: Jean Pihet
 */

#define DBG_COMP    DBG_SWITCH
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/greybus/unipro.h>
#include <errno.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include <nuttx/util.h>

#include "stm32.h"
#include "up_debug.h"
#include "tsb_switch.h"
#include "vreg.h"
#include "tsb_switch_driver_es1.h"
#include "tsb_switch_driver_es2.h"

#define IRQ_WORKER_DEFPRIO          50
#define IRQ_WORKER_STACKSIZE        2048

#define SWITCH_SETTLE_TIME_S        (1)

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

/* Default link configuration values.
 *
 * Bring everything up in non-auto HS-G1. The links which carry
 * display traffic need this, so apply it everywhere.
 *
 * TODO: these should be replaced with auto PWM-G1 (i.e. USE_HS=0,
 *       FLAGS=UNIPRO_LINK_CFGF_AUTO) when the AP and SVC know enough
 *       control protocol to do the right thing on all links.
 */
#define LINK_DEFAULT_USE_HS_GEAR    1
#define LINK_DEFAULT_HS_GEAR        1
#define LINK_DEFAULT_PWM_GEAR       1
#define LINK_DEFAULT_HS_NLANES      2
#define LINK_DEFAULT_PWM_NLANES     LINK_DEFAULT_HS_NLANES
#define LINK_DEFAULT_FLAGS          0

/* Default CPort configuration values. */
#define CPORT_DEFAULT_TOKENVALUE           32
#define CPORT_DEFAULT_T_PROTOCOLID         0
#define CPORT_DEFAULT_TSB_MAXSEGMENTCONFIG 0x118

/* MaskId entry update */
#define SET_VALID_ENTRY(entry) \
    id_mask[15 - ((entry) / 8)] |= (1 << ((entry)) % 8)

static inline void dev_ids_update(struct tsb_switch *sw,
                                  uint8_t port_id,
                                  uint8_t dev_id) {
    sw->dev_ids[port_id] = dev_id;
}

static inline uint8_t dev_ids_port_to_dev(struct tsb_switch *sw,
                                          uint8_t port_id) {
    if (port_id >= SWITCH_PORT_MAX)
         return INVALID_PORT;

    return sw->dev_ids[port_id];
}

static inline uint8_t dev_ids_dev_to_port(struct tsb_switch *sw,
                                          uint8_t dev_id) {
    int i;

    for (i = 0; i < SWITCH_PORT_MAX; i++) {
        if (sw->dev_ids[i] == dev_id) {
             return i;
        }
    }

    return INVALID_PORT;
}

static void dev_ids_destroy(struct tsb_switch *sw) {
    memset(sw->dev_ids, INVALID_PORT, sizeof(sw->dev_ids));
}

static void switch_power_on_reset(struct tsb_switch *sw)
{
    /* Enable the switch power supplies regulator */
    vreg_get(sw->pdata->vreg);

    /* Release reset */
    stm32_gpiowrite(sw->pdata->gpio_reset, true);
    sleep(SWITCH_SETTLE_TIME_S);
}

static void switch_power_off(struct tsb_switch *sw)
{
    /* Release the switch power supplies regulator */
    vreg_put(sw->pdata->vreg);

    /* Assert reset */
    stm32_gpiowrite(sw->pdata->gpio_reset, false);
}



/* Switch communication link init and de-init */
static int switch_init_comm(struct tsb_switch *sw)
{
    if (!sw->ops->init_comm) {
        return -EOPNOTSUPP;
    }
    return sw->ops->init_comm(sw);
}

/*
 * Unipro NCP commands
 */
int switch_dme_set(struct tsb_switch *sw,
                          uint8_t portid,
                          uint16_t attrid,
                          uint16_t select_index,
                          uint32_t attr_value) {
    if (!sw->ops->set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->set(sw, portid, attrid, select_index, attr_value);
}

int switch_dme_get(struct tsb_switch *sw,
                          uint8_t portid,
                          uint16_t attrid,
                          uint16_t select_index,
                          uint32_t *attr_value) {
    if (!sw->ops->get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->get(sw, portid, attrid, select_index, attr_value);
}

int switch_dme_peer_set(struct tsb_switch *sw,
                               uint8_t portid,
                               uint16_t attrid,
                               uint16_t select_index,
                               uint32_t attr_value) {
    if (!sw->ops->peer_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->peer_set(sw, portid, attrid, select_index, attr_value);
}

int switch_dme_peer_get(struct tsb_switch *sw,
                               uint8_t portid,
                               uint16_t attrid,
                               uint16_t select_index,
                               uint32_t *attr_value) {
    if (!sw->ops->peer_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->peer_get(sw, portid, attrid, select_index, attr_value);
}

int switch_port_irq_enable(struct tsb_switch *sw,
                           uint8_t portid,
                           bool enable) {
    if (!sw->ops->port_irq_enable) {
        return -EOPNOTSUPP;
    }
    return sw->ops->port_irq_enable(sw, portid, enable);
}

/*
 * Routing table configuration commands
 */
int switch_lut_set(struct tsb_switch *sw,
                   uint8_t unipro_portid,
                   uint8_t addr,
                   uint8_t dst_portid) {
    if (!sw->ops->lut_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->lut_set(sw, unipro_portid, addr, dst_portid);
}

int switch_lut_get(struct tsb_switch *sw,
                   uint8_t unipro_portid,
                   uint8_t addr,
                   uint8_t *dst_portid) {
    if (!sw->ops->lut_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->lut_get(sw, unipro_portid, addr, dst_portid);
}

int switch_dump_routing_table(struct tsb_switch *sw) {
    if (!sw->ops->dump_routing_table) {
        return -EOPNOTSUPP;
    }
    return sw->ops->dump_routing_table(sw);
}

int switch_sys_ctrl_set(struct tsb_switch *sw,
                        uint16_t sc_addr,
                        uint32_t val) {
    if (!sw->ops->sys_ctrl_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->sys_ctrl_set(sw, sc_addr, val);
}

int switch_sys_ctrl_get(struct tsb_switch *sw,
                        uint16_t sc_addr,
                        uint32_t *val) {
    if (!sw->ops->sys_ctrl_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->sys_ctrl_get(sw, sc_addr, val);
}

int switch_dev_id_mask_get(struct tsb_switch *sw,
                           uint8_t unipro_portid,
                           uint8_t *dst) {
    if (!sw->ops->dev_id_mask_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->dev_id_mask_get(sw, unipro_portid, dst);
}

int switch_dev_id_mask_set(struct tsb_switch *sw,
                           uint8_t unipro_portid,
                           uint8_t *mask) {
    if (!sw->ops->dev_id_mask_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->dev_id_mask_set(sw, unipro_portid, mask);
}

/*
 * Switch internal configuration commands
 */
int switch_internal_getattr(struct tsb_switch *sw,
                            uint16_t attrid,
                            uint32_t *val) {
    if (!sw->ops->switch_attr_get) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_attr_get(sw, attrid, val);
}

int switch_internal_setattr(struct tsb_switch *sw,
                            uint16_t attrid,
                            uint32_t val) {
    if (!sw->ops->switch_attr_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_attr_set(sw, attrid, val);
}

static int switch_internal_set_id(struct tsb_switch *sw,
                                  uint8_t cportid,
                                  uint8_t peercportid,
                                  uint8_t dis,
                                  uint8_t irt) {
    if (!sw->ops->switch_id_set) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_id_set(sw, cportid, peercportid, dis, irt);
}

int switch_irq_enable(struct tsb_switch *sw,
                      bool enable) {
    if (!sw->ops->switch_irq_enable) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_irq_enable(sw, enable);
}

int switch_irq_handler(struct tsb_switch *sw) {
    if (!sw->ops->switch_irq_handler) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_irq_handler(sw);
}

static int switch_set_port_l4attr(struct tsb_switch *sw,
                                  uint8_t portid,
                                  uint16_t attrid,
                                  uint16_t selector,
                                  uint32_t val) {
    int rc;

    if (portid == SWITCH_PORT_ID) {
        rc = switch_dme_set(sw, portid, attrid, selector, val);
    } else {
        rc = switch_dme_peer_set(sw, portid, attrid, selector, val);
    }

    return rc;
}

static int switch_get_port_l4attr(struct tsb_switch *sw,
                                  uint8_t portid,
                                  uint16_t attrid,
                                  uint16_t selector,
                                  uint32_t *val) {
    int rc;

    if (portid == SWITCH_PORT_ID) {
        rc = switch_dme_get(sw, portid, attrid, selector, val);
    } else {
        rc = switch_dme_peer_get(sw, portid, attrid, selector, val);
    }

    return rc;
}


static int switch_set_pair_attr(struct tsb_switch *sw,
                                struct unipro_connection *c,
                                uint16_t attrid,
                                uint32_t val0,
                                uint32_t val1) {
    int rc;
    rc = switch_set_port_l4attr(sw,
            c->port_id0,
            attrid,
            c->cport_id0,
            val0);
    if (rc) {
        return rc;
    }

    rc = switch_set_port_l4attr(sw,
            c->port_id1,
            attrid,
            c->cport_id1,
            val1);
    if (rc) {
        return rc;
    }

    return 0;
}

static int switch_cport_connect(struct tsb_switch *sw,
                                struct unipro_connection *c) {
    int e2efc_enabled = (!!(c->flags & CPORT_FLAGS_E2EFC) == 1);
    int csd_enabled = (!!(c->flags & CPORT_FLAGS_CSD_N) == 0);
    int rc = 0;

    /* Disable any existing connection(s). */
    rc = switch_set_pair_attr(sw, c, T_CONNECTIONSTATE, 0, 0);
    if (rc) {
        return rc;
    }

    /*
     * Point each device at the other.
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              T_PEERDEVICEID,
                              c->device_id1,
                              c->device_id0);
    if (rc) {
        return rc;
    }

    /*
     * Point each CPort at the other.
     */
    rc = switch_set_pair_attr(sw, c, T_PEERCPORTID, c->cport_id1, c->cport_id0);
    if (rc) {
        return rc;
    }

    /*
     * Match up traffic classes.
     */
    rc = switch_set_pair_attr(sw, c, T_TRAFFICCLASS, c->tc, c->tc);
    if (rc) {
        return rc;
    }

    /*
     * Make sure the protocol IDs are equal. (We don't use them otherwise.)
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              T_PROTOCOLID,
                              CPORT_DEFAULT_T_PROTOCOLID,
                              CPORT_DEFAULT_T_PROTOCOLID);
    if (rc) {
        return rc;
    }

    /*
     * Set default TxTokenValue and RxTokenValue values.
     *
     * IMPORTANT: TX and RX token values must be equal if E2EFC is
     * enabled, so don't change them to different values unless you
     * also patch up the E2EFC case, below.
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              T_TXTOKENVALUE,
                              CPORT_DEFAULT_TOKENVALUE,
                              CPORT_DEFAULT_TOKENVALUE);
    if (rc) {
        return rc;
    }

    rc = switch_set_pair_attr(sw,
                              c,
                              T_RXTOKENVALUE,
                              CPORT_DEFAULT_TOKENVALUE,
                              CPORT_DEFAULT_TOKENVALUE);
    if (rc) {
        return rc;
    }


    /*
     * Set CPort flags.
     *
     * (E2EFC needs to be the same on both sides, which is handled by
     * having a single flags value for now.)
     */
    rc = switch_set_pair_attr(sw, c, T_CPORTFLAGS, c->flags, c->flags);
    if (rc) {
        return rc;
    }

    /*
     * If E2EFC is enabled, or E2EFC is disabled and CSD is enabled,
     * then each CPort's T_PeerBufferSpace must equal the peer CPort's
     * T_LocalBufferSpace.
     */
    if (e2efc_enabled || (!e2efc_enabled && csd_enabled)) {
        uint32_t cport0_local = 0;
        uint32_t cport1_local = 0;
        rc = switch_get_port_l4attr(sw,
                c->port_id0,
                T_LOCALBUFFERSPACE,
                c->cport_id0,
                &cport0_local);
        if (rc) {
            return rc;
        }

        rc = switch_get_port_l4attr(sw,
                c->port_id1,
                T_LOCALBUFFERSPACE,
                c->cport_id1,
                &cport1_local);
        if (rc) {
            return rc;
        }

        rc = switch_set_pair_attr(sw,
                                  c,
                                  T_LOCALBUFFERSPACE,
                                  cport0_local,
                                  cport1_local);
        if (rc) {
            return rc;
        }
    }

    /*
     * Ensure the CPorts aren't in test mode.
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              T_CPORTMODE,
                              CPORT_MODE_APPLICATION,
                              CPORT_MODE_APPLICATION);
    if (rc) {
        return rc;
    }

    /*
     * Clear out the credits to send on each side.
     */
    rc = switch_set_pair_attr(sw, c, T_CREDITSTOSEND, 0, 0);
    if (rc) {
        return rc;
    }

    /*
     * XXX Toshiba-specific TSB_MaxSegmentConfig (move to bridge ASIC code.)
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              TSB_MAXSEGMENTCONFIG,
                              CPORT_DEFAULT_TSB_MAXSEGMENTCONFIG,
                              CPORT_DEFAULT_TSB_MAXSEGMENTCONFIG);
    if (rc) {
        return rc;
    }
    /*
     * Establish the connections!
     */
    rc = switch_set_pair_attr(sw, c, T_CONNECTIONSTATE, 1, 1);
    if (rc) {
        return rc;
    }

    return rc;
}

static int switch_cport_disconnect(struct tsb_switch *sw,
                                   uint8_t port_id0,
                                   uint8_t cport_id0,
                                   uint8_t port_id1,
                                   uint8_t cport_id1) {
    int rc0, rc1;
    rc0 = switch_dme_peer_set(sw, port_id0, T_CONNECTIONSTATE,
                             cport_id0, 0x0);
    rc1 = switch_dme_peer_set(sw, port_id1, T_CONNECTIONSTATE,
                              cport_id1, 0x0);
    return rc0 || rc1;
}

#if !CONFIG_ARCH_BOARD_ARA_BDB2A_SVC
static int switch_link_power_set_default(struct tsb_switch *sw,
                                         uint8_t port_id) {
    int rc;

    if (LINK_DEFAULT_USE_HS_GEAR) {
        rc = switch_configure_link_hs(sw,
                                      port_id,
                                      LINK_DEFAULT_HS_GEAR,
                                      LINK_DEFAULT_HS_NLANES,
                                      LINK_DEFAULT_FLAGS);
    } else {
        rc = switch_configure_link_pwm(sw,
                                       port_id,
                                       LINK_DEFAULT_PWM_GEAR,
                                       LINK_DEFAULT_PWM_NLANES,
                                       LINK_DEFAULT_FLAGS);
    }

    return rc;
}
#endif

/**
 * @brief Assign a device id to a given port id
 */
int switch_if_dev_id_set(struct tsb_switch *sw,
                         uint8_t port_id,
                         uint8_t dev_id) {
    int rc;

    if (port_id >= SWITCH_UNIPORT_MAX) {
        return -EINVAL;
    }

    rc = switch_dme_peer_set(sw, port_id, N_DEVICEID,
                             UNIPRO_SELINDEX_NULL, dev_id);
    if (rc) {
        return rc;
    }

    rc = switch_dme_peer_set(sw, port_id, N_DEVICEID_VALID,
                             UNIPRO_SELINDEX_NULL, 1);
    if (rc) {
        /* do what on failure? */
        return rc;
    }

    /* update the table */
    dev_ids_update(sw, port_id, dev_id);

    return 0;
}

/**
 * @brief Setup network routing table
 *
 * Setup of the deviceID Mask tables (if supported) and the Switch LUTs,
 * bidirectionally between the source and destination Switch ports.
 */
int switch_setup_routing_table(struct tsb_switch *sw,
                               uint8_t device_id_0,
                               uint8_t port_id_0,
                               uint8_t device_id_1,
                               uint8_t port_id_1) {

    int rc;
    uint8_t id_mask[16];

    dbg_verbose("Setup routing table [%u:%u]<->[%u:%u]\n",
                device_id_0, port_id_0, device_id_1, port_id_1);

    // Set MaskId for devices 0->1
    rc = switch_dev_id_mask_get(sw,
                                port_id_0,
                                id_mask);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to get MaskId for port %d\n", port_id_0);
        return rc;
    }

    SET_VALID_ENTRY(device_id_1);

    rc = switch_dev_id_mask_set(sw,
                                port_id_0,
                                id_mask);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to set MaskId for port %d\n", port_id_0);
        return rc;
    }

    // Set MaskId for devices 1->0
    rc = switch_dev_id_mask_get(sw,
                                port_id_1,
                                id_mask);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to get MaskId for port %d\n", port_id_1);
        return rc;
    }

    SET_VALID_ENTRY(device_id_0);

    rc = switch_dev_id_mask_set(sw,
                                port_id_1,
                                id_mask);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to set MaskId for port %d\n", port_id_1);
        return rc;
    }

    // Setup routing table for devices 0->1
    rc = switch_lut_set(sw, port_id_0, device_id_1, port_id_1);
    if (rc) {
        dbg_error("Failed to set Lut for source port %d, disabling\n",
                  port_id_0);
        /* Undo deviceid_valid on failure */
        switch_dme_peer_set(sw, port_id_0, N_DEVICEID_VALID,
                            UNIPRO_SELINDEX_NULL, 0);
        return rc;
    }

    // Setup routing table for devices 1->0
    rc = switch_lut_set(sw, port_id_1, device_id_0, port_id_0);
    if (rc) {
        dbg_error("Failed to set Lut for source port %d, disabling\n",
                  port_id_1);
        /* Undo deviceid_valid on failure */
        switch_dme_peer_set(sw, port_id_1, N_DEVICEID_VALID,
                            UNIPRO_SELINDEX_NULL, 0);
        return rc;
    }

    return 0;
}

/**
 * @brief Create a connection between two cports
 */
int switch_connection_create(struct tsb_switch *sw,
                             struct unipro_connection *c) {
    int rc;

    if (!c) {
        rc = -EINVAL;
        goto err0;
    }

    dbg_info("Creating connection: "
             "[p=%u,d=%u,c=%u]<->[p=%u,d=%u,c=%u] "
             "TC: %u Flags: 0x%x\n",
             c->port_id0,
             c->device_id0,
             c->cport_id0,
             c->port_id1,
             c->device_id1,
             c->cport_id1,
             c->tc,
             c->flags);

    /*
     * UniPro HS gears have issues on BDB2A with the default M-PHY
     * settings. Turn this off on that target for now, but leave them
     * in for ES1 targets (e.g., the display tunnel on ES1 demos needs
     * the default setting of HS-G1 to work properly).
     */
#if !CONFIG_ARCH_BOARD_ARA_BDB2A_SVC
    rc = switch_link_power_set_default(sw, c->port_id0);
    if (rc) {
        goto err1;
    }

    rc = switch_link_power_set_default(sw, c->port_id1);
    if (rc) {
        goto err1;
    }
#endif

    rc = switch_cport_connect(sw, c);
    if (rc) {
        switch_cport_disconnect(sw,
                                c->port_id0,
                                c->cport_id0,
                                c->port_id1,
                                c->cport_id1);
        dbg_error("%s: couldn't create connection: %d\n", __func__, rc);
        goto err0;
    }

    return 0;

#if !CONFIG_ARCH_BOARD_ARA_BDB2A_SVC
err1:
    dbg_error("%s: couldn't set link power mode to default state.\n",
              __func__);
    switch_cport_disconnect(sw,
                            c->port_id0,
                            c->cport_id0,
                            c->port_id1,
                            c->cport_id1);
#endif
err0:
    dbg_error("%s: Connection setup failed. "
              "[p=%u,d=%u,c=%u]<->[p=%u,d=%u,c=%u] "
              "TC: %u Flags: %x rc: %d\n",
               __func__,
               c->port_id0,
               c->device_id0,
               c->cport_id0,
               c->port_id1,
               c->device_id1,
               c->cport_id1,
               c->tc,
               c->flags,
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
        DME_DDBL1_MANUFACTURERID,
        /*  productID,      expected 0x1000 */
        DME_DDBL1_PRODUCTID,
        /*  length,         expected 0x0008 */
        DME_DDBL1_LENGTH,
        /* DME_DDBL2 VID and PID */
        TSB_DME_DDBL2_A,
        TSB_DME_DDBL2_B,
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
    for (i = 0; i < SWITCH_UNIPORT_MAX; i++) {
        if (*link_status & (1 << i)) {
            for (j = 0; j < ARRAY_SIZE(attr_to_read); j++) {
                if (switch_dme_peer_get(sw, i, attr_to_read[j],
                                        UNIPRO_SELINDEX_NULL, &attr_value)) {
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

/*
 * Determine if a link power mode ought to be taken to slow or slow
 * auto mode before reconfiguration in an HS gear, series B.
 */
static bool switch_ok_for_series_change(enum unipro_pwr_mode mode) {
    switch (mode) {
    case UNIPRO_FAST_MODE:  /* fall through */
    case UNIPRO_FASTAUTO_MODE:
        return false;
    case UNIPRO_SLOW_MODE:
    case UNIPRO_SLOWAUTO_MODE:
        return true;
    default:
        dbg_warn("%s(): unexpected/invalid power mode: 0x%x\n",
                 __func__, mode);
        return false;
    }
}

/*
 * Prepare a link for a change to its M-PHY RATE series.
 */
static int switch_prep_for_series_change(struct tsb_switch *sw,
                                         uint8_t port_id,
                                         const struct unipro_link_cfg *cfg,
                                         uint32_t *pwr_mode) {
    int rc;
    const struct unipro_pwr_cfg *tx = &cfg->upro_tx_cfg;
    const struct unipro_pwr_cfg *rx = &cfg->upro_rx_cfg;
    bool tx_unchanged = (tx->upro_mode == UNIPRO_MODE_UNCHANGED);
    bool rx_unchanged = (rx->upro_mode == UNIPRO_MODE_UNCHANGED);
    uint32_t cur_pwr_mode;
    enum unipro_pwr_mode cur_tx_pwr_mode;
    enum unipro_pwr_mode cur_rx_pwr_mode;

    /* Sanity checks. */
    if (cfg->upro_hs_ser == UNIPRO_HS_SERIES_UNCHANGED) {
        dbg_error("%s(): invoked when no HS series change is required\n",
                  __func__);
        return -EINVAL;
    }
    if (!(tx->upro_mode == UNIPRO_FAST_MODE ||
          tx->upro_mode == UNIPRO_FASTAUTO_MODE ||
          rx->upro_mode == UNIPRO_FAST_MODE ||
          rx->upro_mode == UNIPRO_FASTAUTO_MODE)) {
        dbg_error("%s(): invoked for non-fast RX/TX power modes %d/%d\n",
                  __func__, tx->upro_mode, rx->upro_mode);
        return -EINVAL;
    }

    /* Grab the current power mode. */
    rc = switch_dme_get(sw, port_id, PA_PWRMODE, UNIPRO_SELINDEX_NULL,
                        &cur_pwr_mode);
    if (rc) {
        dbg_error("%s(): can't check current power mode state\n", __func__);
        return rc;
    }

    /* It's illegal to leave power modes unchanged when setting
     * PA_HSSeries, but we can set the mode to the same value. Use
     * that to respect the caller's intent to leave a mode
     * unchanged. */
    if (tx_unchanged || rx_unchanged) {
        if (tx_unchanged) {
            *pwr_mode &= ~0x0f;
            *pwr_mode |= cur_pwr_mode & 0x0f;
        }
        if (rx_unchanged) {
            *pwr_mode &= ~0xf0;
            *pwr_mode |= cur_pwr_mode & 0xf0;
        }
    }

    /* If both directions of the current mode are ready for the
     * change; there's nothing more to do.
     *
     * Otherwise, the "least common denominator" preparation is to
     * bring the entire link to PWM-G1, with one active lane in each
     * direction. */
    cur_tx_pwr_mode = (enum unipro_pwr_mode)(cur_pwr_mode & 0xf);
    cur_rx_pwr_mode = (enum unipro_pwr_mode)((cur_pwr_mode >> 4) & 0xf);
    if (switch_ok_for_series_change(cur_tx_pwr_mode) &&
        switch_ok_for_series_change(cur_rx_pwr_mode)) {
        return 0;
    } else {
        return switch_configure_link_pwm(sw, port_id, 1, 1, 0);
    }
}

/*
 * Sanity check an HS or PWM configuration.
 */
static int switch_active_cfg_is_sane(const struct unipro_pwr_cfg *pcfg,
                                     unsigned max_nlanes) {
    if (pcfg->upro_nlanes > max_nlanes) {
        dbg_error("%s(): attempt to use %u lanes, support at most %u\n",
                  __func__, pcfg->upro_nlanes, max_nlanes);
        return 0;
    }
    switch (pcfg->upro_mode) {
    case UNIPRO_FAST_MODE:      /* fall through */
    case UNIPRO_FASTAUTO_MODE:
        if (pcfg->upro_gear == 0 || pcfg->upro_gear > 3) {
            dbg_error("%s(): invalid HS gear %u\n", __func__, pcfg->upro_gear);
            return 0;
        }
        break;
    case UNIPRO_SLOW_MODE:      /* fall through */
    case UNIPRO_SLOWAUTO_MODE:
        if (pcfg->upro_gear == 0 || pcfg->upro_gear > 7) {
            dbg_error("%s(): invalid PWM gear %u\n", __func__, pcfg->upro_gear);
            return 0;
        }
        break;
    default:
        dbg_error("%s(): unexpected mode %u\n", __func__, pcfg->upro_mode);
        return 0;
    }
    return 1;
}

static int switch_configure_link_tx(struct tsb_switch *sw,
                                    uint8_t port_id,
                                    const struct unipro_pwr_cfg *tx,
                                    uint32_t tx_term) {
    int rc = 0;
    /* If it needs changing, apply the TX side of the new link
     * configuration. */
    if (tx->upro_mode != UNIPRO_MODE_UNCHANGED) {
        rc = switch_dme_set(sw, port_id, PA_TXGEAR,
                            UNIPRO_SELINDEX_NULL, tx->upro_gear) ||
            switch_dme_set(sw, port_id, PA_TXTERMINATION,
                           UNIPRO_SELINDEX_NULL, tx_term) ||
            switch_dme_set(sw, port_id, PA_ACTIVETXDATALANES,
                           UNIPRO_SELINDEX_NULL, tx->upro_nlanes);
    }
    return rc;
}

static int switch_configure_link_rx(struct tsb_switch *sw,
                                    uint8_t port_id,
                                    const struct unipro_pwr_cfg *rx,
                                    uint32_t rx_term) {
    int rc = 0;
    /* If it needs changing, apply the RX side of the new link
     * configuration.
     */
    if (rx->upro_mode != UNIPRO_MODE_UNCHANGED) {
        rc = switch_dme_set(sw, port_id, PA_RXGEAR,
                            UNIPRO_SELINDEX_NULL, rx->upro_gear) ||
            switch_dme_set(sw, port_id, PA_RXTERMINATION,
                           UNIPRO_SELINDEX_NULL, rx_term) ||
            switch_dme_set(sw, port_id, PA_ACTIVERXDATALANES,
                           UNIPRO_SELINDEX_NULL, rx->upro_nlanes);
    }
    return rc;
}

static int switch_configure_link_user_data
        (struct tsb_switch *sw,
         uint8_t port_id,
         const struct unipro_pwr_user_data *udata) {
    int rc = 0;
    const uint32_t flags = udata->flags;
    if (flags & UPRO_PWRF_FC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA0,
                            UNIPRO_SELINDEX_NULL,
                            udata->upro_pwr_fc0_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_TC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA1,
                            UNIPRO_SELINDEX_NULL,
                            udata->upro_pwr_tc0_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_AFC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA2,
                            UNIPRO_SELINDEX_NULL,
                            udata->upro_pwr_afc0_req_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_FC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA3,
                            UNIPRO_SELINDEX_NULL,
                            udata->upro_pwr_fc1_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_TC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA4,
                            UNIPRO_SELINDEX_NULL,
                            udata->upro_pwr_tc1_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & UPRO_PWRF_AFC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            PA_PWRMODEUSERDATA5,
                            UNIPRO_SELINDEX_NULL,
                            udata->upro_pwr_afc1_req_timeout);
    }
    return rc;
}

static int switch_configure_link_tsbdata
        (struct tsb_switch *sw,
         uint8_t port_id,
         const struct tsb_local_l2_timer_cfg *tcfg) {
    int rc = 0;
    const unsigned int flags = tcfg->tsb_flags;
    if (flags & TSB_LOCALL2F_FC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_FC0PROTECTIONTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_fc0_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_TC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_TC0REPLAYTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_tc0_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_AFC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_AFC0REQTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_afc0_req_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_FC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_FC1PROTECTIONTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_fc1_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_TC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_TC1REPLAYTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_tc1_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_AFC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_AFC1REQTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_afc1_req_timeout);
    }
    return rc;
}

static int switch_apply_power_mode(struct tsb_switch *sw,
                                   uint8_t port_id,
                                   uint32_t pwr_mode) {
    const int max_tries = 100;
    int rc;
    uint32_t val;
    int i;

    dbg_insane("%s(): enter, port=%u, pwr_mode=0x%x\n", __func__, port_id,
               pwr_mode);

    /*
     * Clear out the results of a previous failed or timed-out power
     * mode change, if any.
     */
    rc = switch_dme_get(sw, port_id, TSB_DME_POWERMODEIND,
                        UNIPRO_SELINDEX_NULL, &val);
    if (!rc) {
        dbg_verbose("%s(): previous TSB_DME_POWERMODEIND=0x%x\n",
                    __func__, val);
    } else {
        dbg_error("%s(): can't clear TSB_DME_POWERMODEIND: %d\n",
                  __func__, rc);
    }

    rc = switch_dme_set(sw, port_id, PA_PWRMODE, UNIPRO_SELINDEX_NULL,
                        pwr_mode);
    if (rc) {
        dbg_error("%s(): can't set PA_PWRMODE (0x%x) to 0x%x: %d\n",
                  __func__, PA_PWRMODE, pwr_mode, rc);
        if (rc == UNIPRO_CONFIGRESULT_BUSY) {
            /*
             * This is a healthy margin above what the spec says we
             * should give the link to recover from a previously
             * unsuccessful change.
             */
            dbg_warn("%s(): waiting 200 ms for link recovery\n", __func__);
            usleep(200 * 1000);
        }
        goto out;
    }
    for (i = 0; i < max_tries; i++) {
        /*
         * Wait until the power mode change completes.
         *
         * FIXME other error handling (UniPro specification 5.7.12.5).
         */
        rc = switch_dme_get(sw, port_id, TSB_DME_POWERMODEIND,
                            UNIPRO_SELINDEX_NULL, &val);
        if (rc) {
            dbg_error("%s: failed to read power mode indication: %d\n",
                      __func__,
                      rc);
            goto out;
        }

        switch (val) {
        case TSB_DME_POWERMODEIND_NONE:
            /* This happens sometimes, and seems harmless. */
            break;
        case TSB_DME_POWERMODEIND_OK:
            dbg_warn("%s: TSB_DME_POWERMODEIND=0x%x (can't happen!)\n",
                     __func__, val);
            break;
        case TSB_DME_POWERMODEIND_LOCAL:
            /* ... and done */
            break;
        case TSB_DME_POWERMODEIND_REMOTE:
            dbg_info("%s: TSB_DME_POWERMODEIND=0x%x (remote)\n",
                     __func__, val);
            break;
        case TSB_DME_POWERMODEIND_BUSY:
            dbg_warn("%s: TSB_DME_POWERMODEIND=0x%x (busy)\n",
                     __func__, val);
            rc = val;
            goto out;
        case TSB_DME_POWERMODEIND_CAP_ERR:
            dbg_error("%s: TSB_DME_POWERMODEIND=0x%x (capability err)\n",
                      __func__, val);
            rc = val;
            goto out;
        case TSB_DME_POWERMODEIND_FATAL_ERR:
            dbg_error("%s: TSB_DME_POWERMODEIND=0x%x (fatal error)\n",
                      __func__, val);
            rc = val;
            goto out;
        default:
            dbg_error("%s: TSB_DME_POWERMODEIND=0x%x (invalid value)\n",
                      __func__, val);
            rc = val;
            goto out;

        }
        if (val == TSB_DME_POWERMODEIND_LOCAL) {
            break;
        }
    }
    if (val != TSB_DME_POWERMODEIND_LOCAL) {
        rc = -ETIMEDOUT;
        goto out;
    }

    dbg_insane("%s(): testing link state with peer DME access\n", __func__);
    rc = switch_dme_peer_get(sw,
                             port_id,
                             T_CONNECTIONSTATE,
                             UNIPRO_SELINDEX_NULL,
                             &val);

 out:
    dbg_insane("%s(): exit, rc=%d\n", __func__, rc);
    return rc;
}

/**
 * @brief Low-level UniPro link configuration routine.
 *
 * This supports separate reconfiguration of each direction of the
 * UniPro link to different power modes, and allows for hibernation
 * and off modes.
 *
 * Higher level convenience functions to set both directions of a link
 * to the same HS or PWM gear are available.
 *
 * @param sw Switch handle
 * @param port_id Port whose link to reconfigure
 * @param cfg UniPro power configuration to apply
 * @param tcg Toshiba extensions to UniPro power config to apply,
 *            This may be NULL if no extensions are needed.
 *
 * @see switch_configure_link_hs()
 * @see switch_configure_link_pwm()
 */
int switch_configure_link(struct tsb_switch *sw,
                          uint8_t port_id,
                          const struct unipro_link_cfg *cfg,
                          const struct tsb_link_cfg *tcfg) {
    int rc = 0;
    const struct unipro_pwr_cfg *tx = &cfg->upro_tx_cfg;
    uint32_t tx_term = !!(cfg->flags & UPRO_LINKF_TX_TERMINATION);
    const struct unipro_pwr_cfg *rx = &cfg->upro_rx_cfg;
    uint32_t rx_term = !!(cfg->flags & UPRO_LINKF_RX_TERMINATION);
    uint32_t scrambling = !!(cfg->flags & UPRO_LINKF_SCRAMBLING);
    const struct unipro_pwr_user_data *udata = &cfg->upro_user;
    uint32_t pwr_mode = tx->upro_mode | (rx->upro_mode << 4);

    dbg_verbose("%s(): port=%d\n", __func__, port_id);

    /* FIXME ADD JIRA support hibernation and link off modes. */
    if (tx->upro_mode == UNIPRO_HIBERNATE_MODE ||
        tx->upro_mode == UNIPRO_OFF_MODE ||
        rx->upro_mode == UNIPRO_HIBERNATE_MODE ||
        rx->upro_mode == UNIPRO_OFF_MODE) {
        rc = -EOPNOTSUPP;
        goto out;
    }

    /* Sanity-check the configuration. */
    if (!(switch_active_cfg_is_sane(tx, PA_CONN_TX_DATA_LANES_NR) &&
          switch_active_cfg_is_sane(rx, PA_CONN_RX_DATA_LANES_NR))) {
        rc = -EINVAL;
        goto out;
    }

    /* Changes to a link's HS series require special preparation, and
     * involve restrictions on the power mode to apply next.
     *
     * Handle that properly before setting PA_HSSERIES. */
    if (cfg->upro_hs_ser != UNIPRO_HS_SERIES_UNCHANGED) {
        rc = switch_prep_for_series_change(sw, port_id, cfg, &pwr_mode);
        if (rc) {
            goto out;
        }
        rc = switch_dme_set(sw, port_id, PA_HSSERIES, UNIPRO_SELINDEX_NULL,
                            cfg->upro_hs_ser);
        if (rc) {
            dbg_error("%s(): can't change PA_HSSeries: %d\n",
                      __func__, rc);
            goto out;
        }
    }

    /* Apply TX and RX link reconfiguration as needed. */
    rc = switch_configure_link_tx(sw, port_id, tx, tx_term) ||
        switch_configure_link_rx(sw, port_id, rx, rx_term);
    if (rc) {
        goto out;
    }

    /* Handle scrambling. */
    rc = switch_dme_set(sw, port_id, PA_SCRAMBLING,
                        UNIPRO_SELINDEX_NULL, scrambling);
    if (rc) {
        goto out;
    }

    /* Set any DME user data we understand. */
    rc = switch_configure_link_user_data(sw, port_id, udata);
    if (rc) {
        goto out;
    }

    /* Handle Toshiba extensions to the link configuration procedure. */
    if (tcfg) {
        rc = switch_configure_link_tsbdata(sw, port_id, &tcfg->tsb_l2tim_cfg);
    }
    if (rc) {
        goto out;
    }

    /* Kick off the actual power mode change, and see what happens. */
    rc = switch_apply_power_mode(sw, port_id, pwr_mode);
 out:
    dbg_insane("%s(): exit, rc=%d\n", __func__, rc);
    return rc;
}

/* Post a message to the IRQ worker */
int switch_post_irq(struct tsb_switch *sw)
{
    sem_post(&sw->sw_irq_lock);

    return 0;
}

/* IRQ worker */
static int switch_irq_pending_worker(int argc, char *argv[])
{
    struct tsb_switch *sw = (struct tsb_switch *) strtol(argv[1], NULL, 16);

    if (!sw) {
        dbg_error("%s: no Switch context\n", __func__);
        return ERROR;
    }

    while (!sw->sw_irq_worker_exit) {

        sem_wait(&sw->sw_irq_lock);
        if (sw->sw_irq_worker_exit)
            break;

        /* Calls the low level handler to clear the interrupt source */
        switch_irq_handler(sw);
    }

    return 0;
}

/* IRQ worker creation */
static int create_switch_irq_worker(struct tsb_switch *sw)
{
    const char* argv[2];
    char buf[16];
    int ret;

    sprintf(buf, "%p", sw);
    argv[0] = buf;
    argv[1] = NULL;

    ret = task_create("switch_irq_worker",
                      IRQ_WORKER_DEFPRIO, IRQ_WORKER_STACKSIZE,
                      switch_irq_pending_worker,
                      (char * const*) argv);
    if (ret == ERROR) {
        dbg_error("%s: Failed to create IRQ worker\n", __func__);
        return ERROR;
    }

    sw->worker_id = ret;

    return 0;
}

/* IRQ worker destruction */
static int destroy_switch_irq_worker(struct tsb_switch *sw)
{
    int ret = 0, status;

    sw->sw_irq_worker_exit = true;
    sem_post(&sw->sw_irq_lock);

    ret = waitpid(sw->worker_id, &status, 0);
    if (ret < 0)
        dbg_warn("%s: waitpid failed with ret=%d\n", __func__, ret);

    return ret;
}

/**
 * @brief Initialize the switch and set default SVC<->Switch route
 * @param sw pdata platform-specific data for this switch
 * @returns pointer to initialized switch instance on success, NULL on error
 */
struct tsb_switch *switch_init(struct tsb_switch_data *pdata) {
    struct tsb_switch *sw ;
    unsigned int attr_value, link_status;
    int rc;

    dbg_verbose("%s: Initializing switch\n", __func__);

    if (!pdata) {
        return NULL;
    }

    sw = zalloc(sizeof(struct tsb_switch));
    if (!sw) {
        return NULL;
    }

    sw->pdata = pdata;
    sem_init(&sw->sw_irq_lock, 0, 0);
    sw->sw_irq_worker_exit = false;

    list_init(&sw->listeners);

    switch_power_on_reset(sw);

    switch (sw->pdata->rev) {
    case SWITCH_REV_ES1:
        if (tsb_switch_es1_init(sw, sw->pdata->bus)) {
            goto error;
        }
        break;
    case SWITCH_REV_ES2:
        // Initialize the SPI port
        if (tsb_switch_es2_init(sw, sw->pdata->bus)) {
            goto error;
        }
        break;
    default:
        dbg_error("Unsupported switch revision: %u\n", sw->pdata->rev);
        goto error;
    };

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

    // Init port <-> deviceID mapping table
    dev_ids_destroy(sw);
    dev_ids_update(sw, SWITCH_PORT_ID, SWITCH_DEVICE_ID);

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

    // Create Switch interrupt handling worker
    rc = create_switch_irq_worker(sw);
    if (rc) {
        dbg_error("%s: Failed to create Switch IRQ worker\n", __func__);
        goto error;
    }

    // Detect Unipro devices
    rc = switch_detect_devices(sw, &link_status);
    if (rc) {
        goto error;
    }

    return sw;

error:
    switch_exit(sw);
    dbg_error("%s: Failed to initialize switch.\n", __func__);

    return NULL;
}


/**
 * @brief Power down and disable the switch
 */
void switch_exit(struct tsb_switch *sw) {
    dbg_verbose("%s: Disabling switch\n", __func__);
    switch_irq_enable(sw, false);
    destroy_switch_irq_worker(sw);
    dev_ids_destroy(sw);

    switch (sw->pdata->rev) {
    case SWITCH_REV_ES1:
        tsb_switch_es1_exit(sw);
        break;
    case SWITCH_REV_ES2:
        tsb_switch_es2_exit(sw);
        break;
    default:
        dbg_error("Unsupported switch revision: %u\n", sw->pdata->rev);
        break;
    };

    switch_power_off(sw);
    free(sw);
}

int switch_event_register_listener(struct tsb_switch *sw,
                                   struct tsb_switch_event_listener *l) {
    if (!sw || !sw) {
        return -EINVAL;
    }

    list_init(&l->entry);
    list_add(&sw->listeners, &l->entry);
    return 0;
}


int tsb_switch_event_notify(struct tsb_switch *sw,
                            struct tsb_switch_event *event) {

    struct list_head *node, *next;
    struct tsb_switch_event_listener *l;

    list_foreach_safe(&sw->listeners, node, next) {
        l = list_entry(node, struct tsb_switch_event_listener, entry);
        if (l->cb) {
            l->cb(event);
        }
    }

    return 0;
}
