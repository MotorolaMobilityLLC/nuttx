/****************************************************************************
 * configs/bdb/svc/src/up_switch.c
 * BDB SVC support for the Unipro switch
 *
 * Copyright (C) 2014 Google, Inc.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <nuttx/i2c.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_gpio.h"
#include "up_i2c.h"
#include "up_internal.h"
#include "up_power.h"
#include "up_switch.h"
#include "stm32.h"

#include "bdb-internal.h"

#undef lldbg
#define lldbg printk

/* Debug support, to be moved to the appropriate debug code */
static inline void dbg_print_buf(uint8_t *buf, unsigned int size)
{
    unsigned int i;

    printk("buf[%d]=", size);
    for (i = 0; i < size; i++)
        printk(" 0x%02x", buf[i]);
    printk("\n");
}

/* portId to destDeviceId_Enc mapping table */
uint8_t deviceid_table[NR_INTERFACES];

static void deviceid_table_init(void)
{
    int i;

    for (i = 0; i < NR_INTERFACES; i++)
        deviceid_table[i] = INVALID_ID;
}

/* Update table with portId to deviceId mapping */
static void deviceid_table_update(uint8_t portid, uint8_t deviceid)
{
    if (portid < NR_INTERFACES)
        deviceid_table[portid] = deviceid;
}

/* Return destDeviceId_Enc for portId in the mapping table */
static uint8_t deviceid_table_get_deviceid(uint8_t portid)
{
    if (portid < NR_INTERFACES)
        return deviceid_table[portid];

    return INVALID_ID;
}

/* Look up in the mapping table for an entry that maps to deviceId */
static uint8_t deviceid_table_get_portid(uint8_t deviceid)
{
    int i;

    for (i = 0; i < NR_INTERFACES; i++)
        if (deviceid_table[i] == deviceid)
            return i;

    return INVALID_ID;
}

/* Reset the switch before initiating any communication to it */
void switch_reset(void)
{
    printk("%s()\n", __func__);

    /* Assert reset, wait 1ms, de-assert reset */
    stm32_gpiowrite(GPIO_SW_RST_40uS, false);
    usleep(1000);
    stm32_gpiowrite(GPIO_SW_RST_40uS, true);
}

/* Access to the switch routing table */
static int switch_lut_setreq(uint8_t lut_address, uint8_t dest_portid)
{
    uint8_t msg[] = {
        SWITCH_DEVICE_ID,
        lut_address,
        NCP_LUTSETREQ,
        NCP_RESERVED,
        dest_portid
    };
    uint8_t buffer[2];

    printk("%s(): lutAddress=%d, destPortId=%d\n", __func__, lut_address,
           dest_portid);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for FunctionID */
    if (buffer[1] != NCP_LUTSETCNF) {
        printk("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    printk("%s(): ret=0x%02x, lut(%d)=0x%02x\n", __func__,
           buffer[0], lut_address, dest_portid);

    /* Return resultCode */
    return buffer[0];
}

static int switch_lut_getreq(uint8_t lut_address, uint8_t *dest_portid)
{
    uint8_t msg[] = {
        SWITCH_DEVICE_ID,
        lut_address,
        NCP_LUTGETREQ
    };
    uint8_t buffer[4];

    //printk("%s(): lutAddress: %d\n", __func__, lut_address);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for FunctionID */
    if (buffer[1] != NCP_LUTGETCNF) {
        printk("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    *dest_portid = buffer[3];

    //printk("%s(): ret=0x%02x, lut(%d)=0x%02x\n", __func__,
    //       buffer[0], lut_address, *dest_portid);

    /* Return resultCode */
    return buffer[0];
}

/* Access to attribute of an external Unipro port */
int switch_peer_setreq(uint8_t portid, uint16_t attrid,
                       uint16_t select_index, uint32_t attr_value)
{
    uint8_t msg[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_PEERSETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
        ((attr_value >> 24) & 0xff),
        ((attr_value >> 16) & 0xff),
        ((attr_value >> 8) & 0xff),
        (attr_value & 0xff)
    };
    uint8_t buffer[4];

    printk("%s(): attrId=0x%04x, selectIndex=%d, attrValue=0x%08x\n",
           __func__, attrid, select_index, attr_value);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for portID */
    if (buffer[0] != portid) {
        printk("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Check for FunctionID */
    if (buffer[1] != NCP_PEERSETCNF) {
        printk("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    printk("%s(): ret=0x%02x, attr(0x%04x,%d)=0x%04x\n", __func__,
           buffer[3], attrid, select_index, attr_value);

    /* Return resultCode */
    return buffer[3];
}

int switch_peer_getreq(uint8_t portid, uint16_t attrid,
                       uint16_t select_index, uint32_t *attr_value)
{
    uint8_t msg[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_PEERGETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };
    uint8_t buffer[8];

    printk("%s(): portId=%d, attrId=0x%04x, selectIndex=%d\n",
           __func__, portid, attrid, select_index);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for portID */
    if (buffer[0] != portid) {
        printk("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Check for FunctionID */
    if (buffer[1] != NCP_PEERGETCNF) {
        printk("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    /* Retrieve the attribute value from the uint8_t reads */
    *attr_value = buf_to_uint32(&buffer[4]);

    printk("%s(): ret=0x%02x, attr(0x%04x,%d)=0x%04x\n", __func__,
           buffer[3], attrid, select_index, *attr_value);

    /* Return resultCode */
    return buffer[3];
}

/* Access to the switch attribute */
int switch_get_attribute(uint16_t attrid, uint32_t *attr_value)
{
    uint8_t msg[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_SWITCHATTRGETREQ,
        (attrid >> 8),
        (attrid & 0xff),
    };
    uint8_t buffer[6];

    printk("%s() attrId=%d\n", __func__, attrid);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for FunctionID */
    if (buffer[1] != NCP_SWITCHATTRGETCNF) {
        printk("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Retrieve the attribute value from the uint8_t reads */
    *attr_value = buf_to_uint32(&buffer[2]);

    printk("%s(): ret=0x%02x, attr(0x%04x)=0x%08x\n", __func__,
           buffer[0], attrid, *attr_value);

    /* Return resultCode */
    return buffer[0];
}

/*
 * Allow reconfiguration of several Switch internal settings with
 * a single access
 */
static int switch_id_setreq(uint8_t cportid, uint8_t peercportid, uint8_t dis,
                            uint8_t irt)
{
    uint8_t msg[] = {
        SWITCH_DEVICE_ID,
        (dis << 2) | (irt << 0),
        NCP_SWITCHIDSETREQ,
        SWITCH_DEVICE_ID,
        cportid,        // L4 CPortID
        SWITCH_DEVICE_ID,
        peercportid,
        NCP_RESERVED,
        PORT_ID_SWITCH      // Source portID
    };
    uint8_t buffer[2];

    printk("%s(): switchDeviceId=0x%01x, cPortId=0x%01x -> peerCPortId=0x%01x\n",
           __func__, SWITCH_DEVICE_ID, cportid, peercportid);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for FunctionID */
    if (buffer[1] != NCP_SWITCHIDSETCNF) {
        printk("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    printk("%s(): ret=0x%02x, switchDeviceId=0x%01x, cPortId=0x%01x -> peerCPortId=0x%01x\n",
           __func__, buffer[0], SWITCH_DEVICE_ID, cportid, peercportid);

    /* Return resultCode */
    return buffer[0];
}

/* Access to attribute of an internal Unipro port */
int switch_set_req(uint8_t portid, uint16_t attrid,
                   uint16_t select_index, uint32_t attr_value)
{
    uint8_t msg[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_SETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
        ((attr_value >> 24) & 0xff),
        ((attr_value >> 16) & 0xff),
        ((attr_value >> 8) & 0xff),
        (attr_value & 0xff)
    };
    uint8_t buffer[4];

    printk("%s(): portId=%d, attrId=0x%02x, selectIndex=%d\n",
           __func__, portid, attrid, select_index);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for portID */
    if (buffer[0] != portid) {
        printk("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Check for FunctionID */
    if (buffer[1] != NCP_SETCNF) {
        printk("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    printk("%s(): ret=0x%02x, attr(0x%04x,%d)=0x%04x\n", __func__,
           buffer[3], attrid, select_index, attr_value);

    /* Return resultCode */
    return buffer[3];
}

int switch_get_req(uint8_t portid, uint16_t attrid,
                   uint16_t select_index, uint32_t *attr_value)
{
    uint8_t msg[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_GETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };
    uint8_t buffer[8];

    printk("%s(): portId=%d, attrId=0x%02x\n", __func__, portid, attrid);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for portID */
    if (buffer[0] != portid) {
        printk("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Check for FunctionID */
    if (buffer[1] != NCP_GETCNF) {
        printk("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    /* Retrieve the attribute value from the uint8_t reads */
    *attr_value = buf_to_uint32(&buffer[4]);

    printk("%s(): ret=0x%02x, attr(0x%04x)=0x%04x\n", __func__,
           buffer[3], attrid, *attr_value);

    /* Return resultCode */
    return buffer[3];
}

/**
 * @brief Get the valid bitmask for routing table entries
 * @param bitmask destination on where to store the result
 */
static int switch_deviceidmask_getreq(uint8_t *bitmask) {
    uint8_t msg[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_GETDEVICEIDMASKREQ
    };
    uint8_t buffer[18];

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    memcpy(bitmask, buffer + 2, 16);

    return 0;
}

/* Dump routing table */

/**
 * @brief Dump routing table to low level console
 */
static void switch_dump_routing_table(void)
{
    int i, j;
    uint8_t p[8];
    uint8_t valid_bitmask[16];

    printk("%s(): Routing table\n"
            "======================================================\n", __func__);

    /* Replace invalid entries with 'XX' */
#define CHECK_VALID_ENTRY(entry) \
    (valid_bitmask[15 - ((entry) / 8)] & (1 << ((entry)) % 8))

    switch_deviceidmask_getreq(valid_bitmask);

    for (i = 0; i < 8; i++) {
        printk("%3d: ", i * 16);
        for (j = 0; j < 8; j++) {
            if (CHECK_VALID_ENTRY(i * 16 + j)) {
                switch_lut_getreq(i * 16 + j, &p[j]);
                printk("%2u ", p[j]);
            } else {
                printk("XX ");
            }
        }
        printk("| ");
        for (j = 0; j < 8; j++) {
            if (CHECK_VALID_ENTRY((i * 16) + 8 + j)) {
                switch_lut_getreq((i * 16) + 8 + j, &p[j]);
                printk("%2u ", p[j]);
            } else {
                printk("XX ");
            }
        }
        printk("\n");
    }
    printk("======================================================\n", __func__);
}

/* Configure the routing and parameters to allow communication to a device */
static int switch_set_route(uint8_t deviceid)
{
    uint8_t portid;
    uint32_t attribute_value;

    portid = deviceid_table_get_portid(deviceid);
    if (portid == INVALID_ID)
        return ERROR;

    /* Alter routing table */
    switch_lut_setreq(deviceid, portid);

    /* CPort config of the peer device */
    /*  Disable connection */
    switch_peer_setreq(portid, T_CONNECTIONSTATE, NCP_SELINDEX_NULL, 0x0);
    /*  Set CSD_n and CSV_n flags */
    switch_peer_setreq(portid, T_CPORTFLAGS, NCP_SELINDEX_NULL, 0x6);
    /*  TC0 */
    switch_peer_setreq(portid, T_TRAFFICCLASS, NCP_SELINDEX_NULL, 0x0);
    /*  CPortID 0 */
    switch_peer_setreq(portid, T_PEERCPORTID, NCP_SELINDEX_NULL, 0x0);
    /*  PeerDeviceID */
    switch_peer_getreq(portid, T_PEERDEVICEID, NCP_SELINDEX_NULL,
                       &attribute_value);
    switch_peer_setreq(portid, T_PEERDEVICEID, NCP_SELINDEX_NULL, deviceid);
    /*  Enable connection */
    switch_peer_setreq(portid, T_CONNECTIONSTATE, NCP_SELINDEX_NULL, 0x1);
    /*  TOSHIBA Specific Register Access Control, testing only */
    switch_peer_getreq(portid, T_REGACCCTRL_TESTONLY, NCP_SELINDEX_NULL,
                       &attribute_value);
    /*  COM REFCLKFREQ SEL */
    switch_peer_getreq(portid, COM_REFCLKFREQ_SEL, NCP_SELINDEX_NULL,
                       &attribute_value);
    switch_peer_setreq(portid, T_REGACCCTRL_TESTONLY, NCP_SELINDEX_NULL,
                       0x1);
    switch_peer_setreq(portid, COM_REFCLKFREQ_SEL, NCP_SELINDEX_NULL, 0x0);
    switch_peer_setreq(portid, T_REGACCCTRL_TESTONLY, NCP_SELINDEX_NULL,
                       0x0);
    /*  Power Mode Change */
    switch_peer_setreq(portid, PA_TXGEAR, NCP_SELINDEX_NULL, 0x3);
    switch_peer_setreq(portid, PA_TXTERMINATION, NCP_SELINDEX_NULL, 0x1);
    switch_peer_setreq(portid, PA_HSSERIES, NCP_SELINDEX_NULL, 0x1);
    switch_peer_setreq(portid, PA_ACTIVETXDATALANES, NCP_SELINDEX_NULL, 0x2);
    switch_peer_setreq(portid, PA_RXGEAR, NCP_SELINDEX_NULL, 0x3);
    switch_peer_setreq(portid, PA_RXTERMINATION, NCP_SELINDEX_NULL, 0x1);
    switch_peer_setreq(portid, PA_ACTIVERXDATALANES, NCP_SELINDEX_NULL, 0x2);
    /*   DL_FC0ProtectionTimeOutVal */
    switch_peer_setreq(portid, PA_PWRMODEUSERDATA0, NCP_SELINDEX_NULL,
                       0x1FFF);

    /*  Change power mode to Fast Auto Mode for RX & TX */
    switch_set_req(portid, PA_PWRMODE, NCP_SELINDEX_NULL,
                   PA_FASTAUTOMODE_RXTX);
    /*  Check the result of power mode change */
    do {
        switch_get_req(portid, DME_POWERMODEIND, NCP_SELINDEX_NULL,
                       &attribute_value);
    } while (attribute_value);

    /*  Set TSB_MaxSegmentConfig to 280 */
    switch_peer_setreq(portid, TSB_MAXSEGMENTCONFIG, NCP_SELINDEX_NULL,
                       MAX_SEGMENT_CONFIG);

    printk("%s(): deviceId=%d, portId=%d\n", __func__, deviceid, portid);

    return 0;
}


int switch_control(int state)
{
    uint8_t deviceid;
    uint32_t attr_value, link_status;
    int i;

    printk("%s(): state %d\n", __func__, state);

    /* Init GPIO pins for debug, SVC IRQ, bridges power supplies */
    gpio_init();
    bdb_apb1_init();
    bdb_apb2_init();
    bdb_apb3_init();
    bdb_gpb1_init();
    bdb_gpb2_init();

    /* Init comm with the switch */
    if (i2c_init_comm(I2C_SW_BUS))
        return ERROR;

    /* Init device id mapping table */
    deviceid_table_init();
    deviceid_table_update(PORT_ID_SWITCH, SWITCH_DEVICE_ID);

    if (state == 0) {
        /* De-init */
        gpio_clr_debug();
        bdb_apb1_disable();
        bdb_apb2_disable();
        bdb_apb3_disable();
        bdb_gpb1_disable();
        bdb_gpb2_disable();
    } else {
        /* Init */
        gpio_set_debug();
        switch_reset();
        bdb_apb1_enable();
        bdb_apb2_enable();
        bdb_apb3_enable();
        bdb_gpb1_enable();
        bdb_gpb2_enable();

        /* Get switch version */
        switch_get_attribute(SWVER, &attr_value);

        /* Dump routing table */
        switch_dump_routing_table();

        /*
         * Set initial SVC deviceId to SWITCH_DEVICE_ID and setup
         * routes for internal L4 access from the SVC
         */
        switch_id_setreq(L4_CPORT_SVC_TO_CC, L4_PEERCPORT_SVC_TO_CC,
                         CPORT_ENABLE, IRT_ENABLE);
        switch_id_setreq(L4_CPORT_CC_TO_SVC, L4_PEERCPORT_CC_TO_SVC,
                         CPORT_ENABLE, IRT_DISABLE);

        /* Get switch version, detect devices on the link */
        usleep(500000);
        switch_get_attribute(SWSTA, &link_status);

        /* Get attributes from connected devices */
        for (i = 0; i < NR_INTERFACES; i++) {
            if (link_status & (1 << i)) {
                /* DME_DDBL1 */
                /*  Revision,       expected 0x0010 */
                switch_peer_getreq(i, DME_DDBL1_REVISION, NCP_SELINDEX_NULL,
                                   &attr_value);
                /*  Level,          expected 0x0003 */
                switch_peer_getreq(i, DME_DDBL1_LEVEL, NCP_SELINDEX_NULL,
                                   &attr_value);
                /*  deviceClass,    expected 0x0000 */
                switch_peer_getreq(i, DME_DDBL1_DEVICECLASS,
                                   NCP_SELINDEX_NULL, &attr_value);
                /*  ManufactureID,  expected 0x0126 */
                switch_peer_getreq(i, DME_DDBL1_MANUFACTUREID,
                                   NCP_SELINDEX_NULL, &attr_value);
                /*  productID,      expected 0x1000 */
                switch_peer_getreq(i, DME_DDBL1_PRODUCTID,
                                   NCP_SELINDEX_NULL, &attr_value);
                /*  length,         expected 0x0008 */
                switch_peer_getreq(i, DME_DDBL1_LENGTH, NCP_SELINDEX_NULL,
                                   &attr_value);
                /* DME_DDBL2 VID and PID */
                switch_peer_getreq(i, DME_DDBL2_VID, NCP_SELINDEX_NULL,
                                   &attr_value);
                switch_peer_getreq(i, DME_DDBL2_PID, NCP_SELINDEX_NULL,
                                   &attr_value);
            }
        }

        /*
         * Set initial destDeviceId_Enc and routes
         *
         * Board bring-up phase:
         *  destDeviceId_Enc are set to:
         *   SWITCH_DEVICE_ID = 0 for switch
         *   Use portId + 1 for the rest of the detected devices.
         *  BDB has the following devices connected:
         *   Module portId  destDeviceId_Enc
         *   Switch 14      0
         *   APB1   0       1
         *   APB2   1       2
         *   APB3   2       3
         *   GPB1   3       4
         *   GPB2   4       5
         *   -      5       -
         *   Springs 6..13  7..14
         *
         * Final ES1 phase:
         *  destDeviceId_Enc are set to:
         *   SWITCH_DEVICE_ID = 0 for switch
         *   1 for the detected AP bridge
         *  The rest of the destDeviceId_Enc are assigned by the AP
         *  upon detection
         *
         * Initial routes:
         *  .(Initial bring-up) APB1 <-> GPB1
         *  .to detected AP
         */

        /* Set destDeviceId_Enc and route for the connected devices */
        for (i = 0; i < NR_INTERFACES; i++) {
            if (link_status & (1 << i)) {
                /*
                 * Board bring-up phase style:
                 * destDeviceId_Enc is portId + 1
                 */
                switch_peer_setreq(i, N_DEVICEID, NCP_SELINDEX_NULL, i + 1);
                switch_peer_setreq(i, N_DEVICEID_VALID, NCP_SELINDEX_NULL,
                                   1);
                deviceid_table_update(i, i + 1);
            }
        }

        /* Set default routes and parameters */
        /*  BDB bring-up: APB1 <-> GPB1 */
        deviceid = deviceid_table_get_deviceid(PORT_ID_APB1);
        if (deviceid != INVALID_ID)
            switch_set_route(deviceid);
        else
            printk("%s(): Cannot get APB1 deviceId\n", __func__);

        deviceid = deviceid_table_get_deviceid(PORT_ID_GPB1);
        if (deviceid != INVALID_ID)
            switch_set_route(deviceid);
        else
            printk("%s(): Cannot get GPB1 deviceId\n", __func__);

        /* Dump routing table */
        switch_dump_routing_table();
    }

    return 0;
}
