/*
 * Copyright (c) 2014 Google Inc.
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

/****************************************************************************
 * configs/endo/svc/src/up_switch.c
 * Endo SVC support for the Unipro switch
 *
 ****************************************************************************/
#define DBG_COMP DBG_SWITCH     /* DBG_COMP macro of the component */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <nuttx/i2c.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_debug.h"
#include "up_gpio.h"
#include "up_i2c.h"
#include "up_internal.h"
#include "up_power.h"
#include "up_switch.h"
#include "stm32.h"

/* switch_control parameter */
#define SWITCH_DEINIT               0
#define SWITCH_INIT                 1
#define SWITCH_CHECK_LINKUP         2

/*
 * AP <-> LCD Demo defines
 */
/* DeviceIDs for modules */
#define DEMO_SETUP_AP_M_DEVID       10
#define DEMO_SETUP_LCD_M_DEVID      14
/* CPorts in use */
#define DEV1_SETUP_I2C_CPORT        0
#define DEV1_SETUP_GPIO_CPORT       1
#define DEV2_SETUP_I2C_CPORT        5
#define DEV2_SETUP_GPIO_CPORT       4
#define DEMO_SETUP_LCD_CPORT        16
/* Settle delays, in second */
#define SWITCH_SETTLE_INITIAL_DELAY 1
#define SWITCH_SETTLE_DELAY         1
#define BRIDGE_SETTLE_DELAY         1

/*
 * The Phy lines of the switch are swapped in the layout.
 * Define the conversion table between the physical ports and the
 * Spring (i.e. interface connector) number.
 */
uint8_t spring_port_map[NR_INTERFACES] = {
    0,          /* Spring A <-> port 0 */
    1,          /* Spring B <-> port 1. Note: no Phy lines, debug only */
    3,          /* Spring C <-> port 3 */
    2,          /* Spring D <-> port 2 */
    4,          /* Spring E <-> port 4. Note: no Phy lines, debug only */
    6,          /* Spring F <-> port 6 */
    5,          /* Spring G <-> port 5 */
    8,          /* Spring H <-> port 8 */
    7,          /* Spring I <-> port 7 */
    9,          /* Spring J <-> port 9 */
    10,         /* Spring K <-> port 10 */
    11,         /* Spring L <-> port 11 */
    12,         /* Spring M <-> port 12 */
    13,         /* Spring N <-> port 13 */
    14,         /* Switch internal <-> port 14 */
};

/* portId to destDeviceId_Enc mapping table */
uint8_t deviceid_table[NR_INTERFACES];

/*
 * Dynamic allocation of deviceId:
 * - First deviceId is 0 for the switch,
 * - start on 1 for new devices
 */
static uint8_t last_device_id;

static void deviceid_table_init(void)
{
    int i;

    for (i = 0; i < NR_INTERFACES; i++)
        deviceid_table[i] = INVALID_ID;
    last_device_id = 1;
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
    uint8_t deviceid = INVALID_ID;

    if (portid < NR_INTERFACES)
        deviceid = deviceid_table[portid];

    if (deviceid == INVALID_ID)
        dbg_warn("%s(): Could not get device ID for port %u\n",
                 __func__, portid);

    return deviceid;
}

/* Look up in the mapping table for an entry that maps to deviceId */
static uint8_t deviceid_table_get_portid(uint8_t deviceid)
{
    int i;

    for (i = 0; i < NR_INTERFACES; i++)
        if (deviceid_table[i] == deviceid)
            return i;

    dbg_warn("%s(): Could not find portId for device %u\n",
             __func__, deviceid);

    return INVALID_ID;
}

/* Reset the switch before initiating any communication to it */
void switch_reset(void)
{
    dbg_info("%s()\n", __func__);

    /* Assert reset, cycle the power supplies, then de-assert reset */
    stm32_gpiowrite(GPIO_SW_RST_40uS, false);
    power_cycle_switch();
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

    dbg_verbose("%s(): lutAddress=%d, destPortId=%d\n", __func__, lut_address,
                dest_portid);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for FunctionID */
    if (buffer[1] != NCP_LUTSETCNF) {
        dbg_warn("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    dbg_verbose("%s(): ret=0x%02x, lut(%d)=0x%02x\n", __func__,
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

    dbg_verbose("%s(): lutAddress: %d\n", __func__, lut_address);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for FunctionID */
    if (buffer[1] != NCP_LUTGETCNF) {
        dbg_warn("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    *dest_portid = buffer[3];

    dbg_verbose("%s(): ret=0x%02x, lut(%d)=0x%02x\n", __func__,
                buffer[0], lut_address, *dest_portid);

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

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d, attrValue=0x%08x\n",
                __func__, portid, attrid, select_index, attr_value);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for portID */
    if (buffer[0] != portid) {
        dbg_warn("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Check for FunctionID */
    if (buffer[1] != NCP_PEERSETCNF) {
        dbg_warn("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    dbg_verbose("%s(): ret=0x%02x, attr(0x%04x,%d)=0x%04x\n", __func__,
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

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d\n",
                __func__, portid, attrid, select_index);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for portID */
    if (buffer[0] != portid) {
        dbg_warn("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Check for FunctionID */
    if (buffer[1] != NCP_PEERGETCNF) {
        dbg_warn("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    /* Retrieve the attribute value from the uint8_t reads */
    *attr_value = buf_to_uint32(&buffer[4]);

    dbg_verbose("%s(): ret=0x%02x, attr(0x%04x,%d)=0x%04x\n", __func__,
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

    dbg_info("%s() attrId=%d\n", __func__, attrid);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for FunctionID */
    if (buffer[1] != NCP_SWITCHATTRGETCNF) {
        dbg_warn("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Retrieve the attribute value from the uint8_t reads */
    *attr_value = buf_to_uint32(&buffer[2]);

    dbg_info("%s(): ret=0x%02x, attr(0x%04x)=0x%08x\n", __func__,
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

    dbg_info("%s(): switchDeviceId=0x%01x, cPortId=0x%01x -> peerCPortId=0x%01x\n",
             __func__, SWITCH_DEVICE_ID, cportid, peercportid);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for FunctionID */
    if (buffer[1] != NCP_SWITCHIDSETCNF) {
        dbg_warn("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    dbg_info("%s(): ret=0x%02x, switchDeviceId=0x%01x, cPortId=0x%01x -> peerCPortId=0x%01x\n",
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

    dbg_verbose("%s(): portId=%d, attrId=0x%02x, selectIndex=%d\n",
                __func__, portid, attrid, select_index);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for portID */
    if (buffer[0] != portid) {
        dbg_warn("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Check for FunctionID */
    if (buffer[1] != NCP_SETCNF) {
        dbg_warn("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    dbg_verbose("%s(): ret=0x%02x, attr(0x%04x,%d)=0x%04x\n", __func__,
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

    dbg_verbose("%s(): portId=%d, attrId=0x%02x\n", __func__, portid, attrid);

    if (i2c_switch_send_msg(msg, sizeof(msg)))
        return ERROR;

    if (i2c_switch_receive_msg(buffer, sizeof(buffer)))
        return ERROR;

    /* Check for portID */
    if (buffer[0] != portid) {
        dbg_warn("%s(): wrong portId\n", __func__);
        return ERROR;
    }

    /* Check for FunctionID */
    if (buffer[1] != NCP_GETCNF) {
        dbg_warn("%s(): wrong FunctionID\n", __func__);
        return ERROR;
    }

    /* Retrieve the attribute value from the uint8_t reads */
    *attr_value = buf_to_uint32(&buffer[4]);

    dbg_verbose("%s(): ret=0x%02x, attr(0x%04x)=0x%04x\n", __func__,
                buffer[3], attrid, *attr_value);

    /* Return resultCode */
    return buffer[3];
}

/**
 * @brief Get the valid bitmask for routing table entries
 * @param bitmask destination on where to store the result
 */
static int switch_deviceidmask_getreq(uint8_t *bitmask)
{
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
    int i, j, idx, ret;
    uint8_t p = 0, valid_bitmask[16];
    char msg[64];

    dbg_info("%s(): Routing table\n"
             "======================================================\n",
             __func__);

    /* Replace invalid entries with 'XX' */
#define CHECK_VALID_ENTRY(entry) \
    (valid_bitmask[15 - ((entry) / 8)] & (1 << ((entry)) % 8))

    switch_deviceidmask_getreq(valid_bitmask);

    for (i = 0; i < 8; i++) {
        /* Build a line with the offset, 8 entries, a '|' then 8 entries */
        idx = 0;
        ret = sprintf(msg, "%3d: ", i * 16);
        if (ret <= 0)
            goto out;
        else
            idx += ret;

        for (j = 0; j < 16; j++) {
            if (CHECK_VALID_ENTRY(i * 16 + j)) {
                switch_lut_getreq(i * 16 + j, &p);
                ret = sprintf(msg + idx, "%2u ", p);
                if (ret <= 0)
                    goto out;
                else
                    idx += ret;
            } else {
                ret = sprintf(msg + idx, "XX ");
                if (ret <= 0)
                    goto out;
                else
                    idx += ret;
            }

            if (j == 7) {
                ret = sprintf(msg + idx, "| ");
                if (ret <= 0)
                    goto out;
                else
                    idx += ret;
            }
        }

        ret = sprintf(msg + idx, "\n");
        if (ret <= 0)
            goto out;
        else
            idx += ret;
        msg[idx] = 0;

        /* Output the line */
        dbg_info("%s", msg);
    }

out:
    dbg_info("======================================================\n");
}

static void switch_show_link_state(uint8_t devid)
{
    uint8_t portid;
    uint32_t num_cports, deviceid, peerdeviceid, peercportid;
    int i;

    portid = deviceid_table_get_portid(devid);
    if (portid == INVALID_ID)
        return;

    switch_peer_getreq(portid, T_NUMCPORTS, NCP_SELINDEX_NULL, &num_cports);
    switch_peer_getreq(portid, N_DEVICEID, NCP_SELINDEX_NULL, &deviceid);
    dbg_info("%s(): portID %d: N_DeviceID=%d, T_NumCPorts=%d\n", __func__,
             portid, deviceid, num_cports);

    for (i = 0; i < num_cports; i++) {
        switch_peer_getreq(portid, T_PEERDEVICEID, i, &peerdeviceid);
        switch_peer_getreq(portid, T_PEERCPORTID, i, &peercportid);
        dbg_info(" | CPortID %2d | PeerDeviceID %2d | PeerCPortID %2d |\n",
                 i, peerdeviceid, peercportid);
    }
}

/**
 * @brief Configure a UniPro link with defaults:
 *      2 lanes RX/TX, PWM mode as PA_GEAR
 * @param
 */
static int switch_configure_link(uint8_t devid)
{
    uint32_t val;
    uint8_t portid;

    portid = deviceid_table_get_portid(devid);

    /*  TOSHIBA Specific Register Access Control, testing only */
    switch_peer_setreq(portid, T_REGACCCTRL_TESTONLY, NCP_SELINDEX_NULL, 0x1);
    /*  COM REFCLKFREQ SEL */
    switch_peer_setreq(portid, COM_REFCLKFREQ_SEL, NCP_SELINDEX_NULL, 0x0);
    switch_peer_setreq(portid, T_REGACCCTRL_TESTONLY, NCP_SELINDEX_NULL, 0x0);

    /*  Power Mode Change in the switch */
    /* Q: should the peer device settings be changed as well ? */
    switch_set_req(portid, PA_TXGEAR, NCP_SELINDEX_NULL, PA_GEAR);
    switch_set_req(portid, PA_TXTERMINATION, NCP_SELINDEX_NULL, 0x1);
    switch_set_req(portid, PA_HSSERIES, NCP_SELINDEX_NULL, 0x1);
    switch_set_req(portid, PA_ACTIVETXDATALANES, NCP_SELINDEX_NULL,
                           PA_ACTIVE_TX_DATA_LANES_NR);
    switch_set_req(portid, PA_RXGEAR, NCP_SELINDEX_NULL, PA_GEAR);
    switch_set_req(portid, PA_RXTERMINATION, NCP_SELINDEX_NULL, 0x1);
    switch_set_req(portid, PA_ACTIVERXDATALANES, NCP_SELINDEX_NULL,
                           PA_ACTIVE_RX_DATA_LANES_NR);

    /* DL_FC0ProtectionTimeOutVal */
    switch_set_req(portid, PA_PWRMODEUSERDATA0, NCP_SELINDEX_NULL, 0x1FFF);

    /* Change power mode to Fast Auto Mode for RX & TX */
    switch_set_req(portid, PA_PWRMODE, NCP_SELINDEX_NULL, PA_FASTMODE_RXTX);
    do {
        /* Wait until the power mode change completes */
        switch_get_req(portid, DME_POWERMODEIND, NCP_SELINDEX_NULL, &val);
    } while (val != DME_POWERMODEIND_SUCCESS);

    /* Set TSB_MaxSegmentConfig */
    switch_peer_setreq(portid,
                       TSB_MAXSEGMENTCONFIG,
                       NCP_SELINDEX_NULL,
                       MAX_SEGMENT_CONFIG);

    dbg_info("%s(): deviceId=%d, portId=%d\n", __func__, devid, portid);

    return 0;
}

/**
 * @brief Set up L4 attributes for a cport
 * @param devid device id
 * @param cportid cport id to configure
 * @param peer_devid peer device id
 * @param peer_cportid peed cport to link this cport to
 * @returns 0 on success, ERROR otherwise
 */
static int switch_configure_cport(uint8_t devid,
                                  uint8_t cportid,
                                  uint8_t peer_devid,
                                  uint8_t peer_cportid)
{
    uint8_t portid;

    dbg_info("%s(): deviceId=%u, CPortId=%u, PeerDeviceId=%u, PeerCPortId=%u\n",
             __func__, devid, cportid, peer_devid, peer_cportid);

    portid = deviceid_table_get_portid(devid);
    if (portid == INVALID_ID)
        return ERROR;

    /*  Disable connection */
    switch_peer_setreq(portid, T_CONNECTIONSTATE, cportid, 0x0);

    /*  Set CSD_n and CSV_n flags */
    switch_peer_setreq(portid, T_CPORTFLAGS, cportid, 0x6);

    /*  TC0 for now */
    switch_peer_setreq(portid, T_TRAFFICCLASS, cportid, 0x0);

    /*  Set peer devid/cportid for this cport */
    switch_peer_setreq(portid, T_PEERCPORTID, cportid, peer_cportid);
    switch_peer_setreq(portid, T_PEERDEVICEID, cportid, peer_devid);

    /*  Enable connection */
    switch_peer_setreq(portid, T_CONNECTIONSTATE, cportid, 0x1);

    dbg_info("%s(): done!\n", __func__);

    return 0;
}


/**
 * @brief Configure both sides of a connection at L4. Points both cports
 *        to eachother
 * @param devid device id
 * @param cportid cport id to configure
 * @param peer_devid peer device id
 * @param peer_cportid peed cport to link this cport to
 */
static int switch_configure_connection(uint8_t devid,
                                       uint8_t cportid,
                                       uint8_t peer_devid,
                                       uint8_t peer_cportid)
{
    /* Configure peer device and port IDs */
    switch_configure_cport(devid, cportid, peer_devid, peer_cportid);
    switch_configure_cport(peer_devid, peer_cportid, devid, cportid);

    /* Configure the link power settings */
    switch_configure_link(devid);
    switch_configure_link(peer_devid);

    /* Dump link info */
    switch_show_link_state(devid);
    switch_show_link_state(peer_devid);

    return 0;
}

/* Detect devices on the Unipro link */
uint32_t switch_detect_devices(void)
{
    uint32_t attr_value, link_status;
    int i;

    /*
     * Get link up status, detect devices on the link and
     * setup default routes
     */
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
            switch_peer_getreq(i, PA_CONNECTEDTXDATALANES,
                               NCP_SELINDEX_NULL, &attr_value);
            if (attr_value != PA_CONN_TX_DATA_LANES_NR)
                dbg_error("%s(): Error: Connected TX Data lines=%d\n",
                          __func__, attr_value);
            switch_peer_getreq(i, PA_CONNECTEDRXDATALANES,
                               NCP_SELINDEX_NULL, &attr_value);
            if (attr_value != PA_CONN_RX_DATA_LANES_NR)
                dbg_error("%s(): Error: Connected RX Data lines=%d\n",
                          __func__, attr_value);
        }
    }

    return link_status;
}

/* Dynamically allocate a deviceID_Enc for new devices on the link */
void switch_assign_deviceid(uint32_t link_status)
{
    int i;

    /*
     * Set initial destDeviceId_Enc and routes
     *
     * Endo bring-up phase:
     *  destDeviceId_Enc are set to:
     *   SWITCH_DEVICE_ID = 0 for switch
     *   incremented for every new device on the link
     *
     * Final ES1 phase:
     *  destDeviceId_Enc are set to:
     *   SWITCH_DEVICE_ID = 0 for switch
     *   1 for the detected AP bridge
     *  The rest of the destDeviceId_Enc are assigned by the AP
     *  upon detection
     */

    /* Set destDeviceId_Enc for the connected devices */
    for (i = 0; i < NR_INTERFACES; i++) {
        if (link_status & (1 << i)) {
            /* Do not allocate a deviceId if already existing */
            if (deviceid_table_get_deviceid(i) != INVALID_ID)
                continue;

            /* Endo bring-up phase style: allocate next deviceid */
            switch_peer_setreq(i, N_DEVICEID, NCP_SELINDEX_NULL, i + 1);
            switch_peer_setreq(i, N_DEVICEID_VALID, NCP_SELINDEX_NULL,
                               1);
            deviceid_table_update(i, last_device_id);
            dbg_info("%s(): new deviceId %d for port %d\n", __func__,
                     last_device_id, i);
            last_device_id++;
        }
    }
}

int switch_control(int state)
{
    uint32_t attr_value, link_status;
    uint8_t dev1, dev2;
    int i;

    dbg_info("%s(): state %d\n", __func__, state);

    switch (state) {
    case SWITCH_DEINIT:
        /* De-init */
        /* Power down all interface blocks */
        for (i = 0; i < PWR_SPRING_NR; i++)
            power_set_power(i, false);

        /* Red LED at de-init */
        debug_rgb_led(1, 0, 0);

        break;

    case SWITCH_INIT:
        /* Init */

        /* Enable internal power supplies */
        power_enable_internal();

        /* Init GPIO pins for debug, RGB LED, SVC IRQ, Switch Reset etc. */
        gpio_init();

        /* Red LED at init */
        debug_rgb_led(1, 0, 0);

        /* Init comm with the switch */
        if (i2c_init_comm(I2C_SW_BUS))
            return ERROR;

        /* Init GPIO pins for interfaces power supplies and WAKEOUT */
        power_interface_block_init();

        /* Power-up all interface blocks */
        for (i = 0; i < PWR_SPRING_NR; i++)
            power_set_power(i, true);

        /* Wake-up/power-up all interfaces */
        /* For development: assert WAKEOUT on all interfaces */
        power_set_wakeout(0xFFFFFFFF, true);

        /* Init device id mapping table */
        deviceid_table_init();
        deviceid_table_update(PORT_ID_SWITCH, SWITCH_DEVICE_ID);

start_switch:
        /* Reset switch */
        switch_reset();

        /* Switch settle delay */
        sleep(SWITCH_SETTLE_INITIAL_DELAY);

        /* Get switch version */
        if (switch_get_attribute(SWVER, &attr_value)) {
            i2c_reset();
            goto start_switch;
        }

        /* Orange LED */
        debug_rgb_led(1, 1, 0);

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

        /* Let the switch boot and the links settle */
        sleep(SWITCH_SETTLE_INITIAL_DELAY);

        /* Pass through for routes and connection setup */

    case SWITCH_CHECK_LINKUP:
        /* Re-init device id mapping table */
        deviceid_table_init();
        deviceid_table_update(PORT_ID_SWITCH, SWITCH_DEVICE_ID);

        /* Detect devices on the Unipro link */
        link_status = switch_detect_devices();

        /* Setup initial routes */
        switch_lut_setreq(DEMO_SETUP_AP_M_DEVID,
                          spring_port_map[PORT_ID_SPRING_C]);
        switch_lut_setreq(DEMO_SETUP_LCD_M_DEVID,
                          spring_port_map[PORT_ID_SPRING_M]);

        /* Wait for switch to settle */
        sleep(SWITCH_SETTLE_DELAY);

        /*
         * Set initial destDeviceId_Enc and routes
         *
         * Final ES1 phase:
         *  destDeviceId_Enc are set to:
         *   SWITCH_DEVICE_ID = 0 for switch
         *   1 for the detected AP bridge
         *  The rest of the destDeviceId_Enc are assigned by the AP
         *  upon detection
         *
         * AP <-> LCD demo setup:
         *  destDeviceId_Enc are set to:
         *   SWITCH_DEVICE_ID = 0 for switch
         *   10 for the AP module on Spring C,
         *   14 for the LCD module on Spring M,
         *   CPorts: 5 (I2C) and 16 (DSI)
         */

        /* Set destDeviceId_Enc and route for the connected devices */
        for (i = 0; i < NR_INTERFACES; i++) {
            if (link_status & (1 << i)) {
                /* AP module on Spring C */
                if (i == spring_port_map[PORT_ID_SPRING_C]) {
                    switch_peer_setreq(i, N_DEVICEID, NCP_SELINDEX_NULL,
                                       DEMO_SETUP_AP_M_DEVID);
                    switch_peer_setreq(i, N_DEVICEID_VALID, NCP_SELINDEX_NULL,
                                       1);
                    deviceid_table_update(i, DEMO_SETUP_AP_M_DEVID);
                    dbg_info("%s(): new deviceId %d for port %d\n",
                             __func__, DEMO_SETUP_AP_M_DEVID, i);
                }
                /* LCD module on Spring M */
                if (i == spring_port_map[PORT_ID_SPRING_M]) {
                    switch_peer_setreq(i, N_DEVICEID, NCP_SELINDEX_NULL,
                                       DEMO_SETUP_LCD_M_DEVID);
                    switch_peer_setreq(i, N_DEVICEID_VALID, NCP_SELINDEX_NULL,
                                       1);
                    deviceid_table_update(i, DEMO_SETUP_LCD_M_DEVID);
                    dbg_info("%s(): new deviceId %d for port %d\n",
                             __func__, DEMO_SETUP_LCD_M_DEVID, i);
                }
            }
        }

        /* Set default routes and parameters
         *  AP <-> LCD setup:
         *   Spring C (AP module) <-> Spring M (LCD module)
         */
        dbg_ui("\nCreating Spring C <-> Spring M routing entries\n");
        dev1 = deviceid_table_get_deviceid(spring_port_map[PORT_ID_SPRING_C]);
        dev2 = deviceid_table_get_deviceid(spring_port_map[PORT_ID_SPRING_M]);
        if (dev1 != INVALID_ID && dev2 != INVALID_ID) {
            /*
             * Set up cports:
             *    [5]<->[5] for I2C
             *    [16]<->[16] for DSI
             */
            switch_configure_connection(dev1, DEV1_SETUP_I2C_CPORT,
                                        dev2, DEV2_SETUP_I2C_CPORT);
            switch_configure_connection(dev1, DEV1_SETUP_GPIO_CPORT,
                                        dev2, DEV2_SETUP_GPIO_CPORT);
            switch_configure_connection(dev1, DEMO_SETUP_LCD_CPORT,
                                        dev2, DEMO_SETUP_LCD_CPORT);
            /* Green LED */
            debug_rgb_led(0, 1, 0);
        }

        /* Dump routing table */
        switch_dump_routing_table();
        break;
    default:
        break;
    }

    return 0;
}
