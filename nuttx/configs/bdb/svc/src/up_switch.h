/****************************************************************************
 * configs/bdb/svc/src/up_switch.h
 * BDB SVC switch support. The switch is connected to the SVC via I2C
 *
 * Copyright (C) 2014 Google, Inc.
 * Author: Jean Pihet <jean.pihet@newoldbits.com>
 *
 ****************************************************************************/
#ifndef __CONFIGS_BDB_INCLUDE_UP_SWITCH_H
#define __CONFIGS_BDB_INCLUDE_UP_SWITCH_H

#define SWITCH_DEVICE_ID            0       /* Unipro deviceId of the switch */
/* Switch interfaces: 14 + internal */
#define NR_INTERFACES               15
#define INVALID_ID                  0xff
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

/* Switch attributes */
#define SWVER                       0x0000
#define SWSTA                       0x0003

/* NCP commands */
#define NCP_SETREQ                  0x00
#define NCP_SETCNF                  0x01
#define NCP_PEERSETREQ              0x02
#define NCP_PEERSETCNF              0x03
#define NCP_GETREQ                  0x04
#define NCP_GETCNF                  0x05
#define NCP_PEERGETREQ              0x06
#define NCP_PEERGETCNF              0x07
#define NCP_LUTSETREQ               0x10
#define NCP_LUTSETCNF               0x11
#define NCP_LUTGETREQ               0x12
#define NCP_LUTGETCNF               0x13
#define NCP_GETDEVICEIDMASKREQ      0x16
#define NCP_SWITCHATTRGETREQ        0x22
#define NCP_SWITCHATTRGETCNF        0x23
#define NCP_SWITCHIDSETREQ          0x24
#define NCP_SWITCHIDSETCNF          0x25
/* Unipro attributes */
#define T_REGACCCTRL_TESTONLY       0x007F
#define PA_ACTIVETXDATALANES        0x1560
#define PA_CONNECTEDTXDATALANES     0x1561
#define PA_TXGEAR                   0x1568
#define PA_TXTERMINATION            0x1569
#define PA_HSSERIES                 0x156A
#define PA_PWRMODE                  0x1571
#define PA_ACTIVERXDATALANES        0x1580
#define PA_CONNECTEDRXDATALANES     0x1581
#define PA_RXGEAR                   0x1583
#define PA_RXTERMINATION            0x1584
#define PA_PWRMODEUSERDATA0         0x15B0
#define N_DEVICEID                  0x3000
#define N_DEVICEID_VALID            0x3001
#define T_NUMCPORTS                 0x4000
#define T_CONNECTIONSTATE           0x4020
#define T_PEERDEVICEID              0x4021
#define T_PEERCPORTID               0x4022
#define T_TRAFFICCLASS              0x4023
#define T_CPORTFLAGS                0x4025
#define DME_DDBL1_REVISION          0x5000
#define DME_DDBL1_LEVEL             0x5001
#define DME_DDBL1_DEVICECLASS       0x5002
#define DME_DDBL1_MANUFACTUREID     0x5003
#define DME_DDBL1_PRODUCTID         0x5004
#define DME_DDBL1_LENGTH            0x5005
#define DME_DDBL2_VID               0x6000
#define DME_DDBL2_PID               0x6001
#define TSB_MAILBOX                 0xA000
#define TSB_MAXSEGMENTCONFIG        0xD089
#define DME_POWERMODEIND            0xD040
/* NCP field values */
#define NCP_RESERVED                0x00
#define NCP_SELINDEX_NULL           0x00

/* Unipro attributes values */
#define PA_CONN_TX_DATA_LANES_NR    2
#define PA_CONN_RX_DATA_LANES_NR    2
#define PA_ACTIVE_TX_DATA_LANES_NR  2
#define PA_ACTIVE_RX_DATA_LANES_NR  2
#define MAX_SEGMENT_CONFIG          280
#define PA_FASTMODE_RXTX            0x11
#define PA_FASTAUTOMODE_RXTX        0x44
#define PA_GEAR                     0x1
#define DME_POWERMODEIND_SUCCESS    2

/*  Toshiba specific */
#define COM_REFCLKFREQ_SEL          0x8032

/*
 * Physical portIds on the BDB
 *
 * We need a static list because:
 * . the routing is fixed in HW,
 * . the detection of the AP is not yet implemented and so it is assumed
 *   that AP bridge 1 is used from the AP module,
 * . dynamic enumeration with the AP is not yet implemented.
 */
enum {
    PORT_ID_APB1                    = 0,    /* AP bridge 1 */
    PORT_ID_APB2,                           /* AP bridge 2 */
    PORT_ID_APB3,                           /* AP bridge 3 */
    PORT_ID_GPB1,                           /* GP bridge 1 */
    PORT_ID_GPB2,                           /* GP bridge 2 */
    PORT_ID_NC,                             /* Not connected */
    PORT_ID_SPRING6,                        /* Spring connector 6 */
    PORT_ID_SPRING7,                        /* Spring connector 7 */
    PORT_ID_SPRING8,                        /* Spring connector 8 */
    PORT_ID_SPRING9,                        /* Spring connector 9 */
    PORT_ID_SPRING10,                       /* Spring connector 10 */
    PORT_ID_SPRING11,                       /* Spring connector 11 */
    PORT_ID_SPRING12,                       /* Spring connector 12 */
    PORT_ID_SPRING13,                       /* Spring connector 13 */
    PORT_ID_SWITCH                          /* Switch */
};


/*
 * Conversion from uint8_t * to uint32
 *
 * This is not portable, STM use only!
 */
static inline uint32_t buf_to_uint32(uint8_t *ptr)
{
    return ((*ptr << 24) | (*(ptr + 1) << 16) |
            (*(ptr + 2) << 8) | (*(ptr + 3)));
}

extern int switch_get_attribute(uint16_t attrid, uint32_t *attr_value);
extern int switch_set_req(uint8_t portid, uint16_t attrid,
                          uint16_t select_index, uint32_t attr_value);
extern int switch_get_req(uint8_t portid, uint16_t attrid,
                          uint16_t select_index, uint32_t *attr_value);
extern int switch_peer_setreq(uint8_t portid, uint16_t attrid,
                          uint16_t select_index, uint32_t attr_value);
extern int switch_peer_getreq(uint8_t portid, uint16_t attrid,
                          uint16_t select_index, uint32_t *attr_value);
extern int switch_control(int state);

#endif // __CONFIGS_BDB_INCLUDE_UP_SWITCH_H
