/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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

#ifndef __SLICE_BUS_H__
#define __SLICE_BUS_H__

#define SLICE_REG_INVALID        -2
#define SLICE_REG_NOT_SET        -1
#define SLICE_REG_INT             0     /* Interrupt register        */
#define SLICE_REG_SVC             1     /* SVC message register      */
#define SLICE_REG_UNIPRO          2     /* Unipro message register   */
#define SLICE_REG__NUM            3     /* Add new registers above   */

#define SLICE_REG_INT_SVC      0x01
#define SLICE_REG_INT_UNIPRO   0x02

#define SLICE_REG_SVC_RX_SZ       8
#define SLICE_REG_UNIPRO_SZ      16

struct slice_svc_msg
{
    uint8_t *buf;
    int size;
    struct list_head list;
};

struct slice_bus_data
{
    uint8_t reg_int;

    struct list_head reg_svc_tx_fifo;
    uint8_t reg_svc_rx[SLICE_REG_SVC_RX_SZ];

    uint8_t reg_unipro_tx_size;
    uint8_t *reg_unipro_tx;
    uint8_t reg_unipro_rx[SLICE_REG_UNIPRO_SZ];
    uint8_t reg_unipro_rx_cport;
};

struct slice_unipro_msg_rx
{
    __u8    checksum;
    __u8    slice_cport;
    __u8    ap_cport;
    __u8    data[0];
};

struct slice_unipro_msg_tx
{
    __u8    checksum;
    __u8    ap_cport;
    __u8    data[0];
};

/* Initialize the bus to the host/base/core. */
int bus_init(void);

/* Send an SVC message across the bus to the base */
int bus_svc_to_base(void *buf, size_t length);

/* Send a Greybus message across the bus to the base */
int bus_greybus_to_base(unsigned int cportid, const void *buf, size_t len);

/* Base has sent a Greybus message */
void bus_greybus_from_base(struct slice_bus_data *slf, size_t len);

/* Assert an interrupt to the base */
void bus_interrupt(struct slice_bus_data *slf, uint8_t int_mask, bool assert);

#endif

