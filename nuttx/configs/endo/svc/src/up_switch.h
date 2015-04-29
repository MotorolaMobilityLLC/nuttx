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
 * configs/endo/svc/src/up_switch.h
 * Endo SVC switch support. The switch is connected to the SVC via I2C
 *
 ****************************************************************************/
#ifndef __CONFIGS_ENDO_INCLUDE_UP_SWITCH_H
#define __CONFIGS_ENDO_INCLUDE_UP_SWITCH_H

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
#define SWINT                       0x0010

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
 * Physical portIds on the Endo
 *
 * The Endo geometry is:
 * Top
 *      N
 *      M
 * Bottom
 *      B  A
 *      D  C
 *      E  F
 *      G  H
 *      I  J
 *      K  L
 */
enum {
    PORT_ID_SPRING_A            = 0,
    PORT_ID_SPRING_B,
    PORT_ID_SPRING_C,
    PORT_ID_SPRING_D,
    PORT_ID_SPRING_E,
    PORT_ID_SPRING_F,
    PORT_ID_SPRING_G,
    PORT_ID_SPRING_H,
    PORT_ID_SPRING_I,
    PORT_ID_SPRING_J,
    PORT_ID_SPRING_K,
    PORT_ID_SPRING_L,
    PORT_ID_SPRING_M,
    PORT_ID_SPRING_N,
    PORT_ID_SWITCH              /* Switch */
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

#endif // __CONFIGS_ENDO_INCLUDE_UP_SWITCH_H
