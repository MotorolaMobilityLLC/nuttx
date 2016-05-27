/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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
#ifndef __USBTUN_H__
#define __USBTUN_H__

#include <nuttx/config.h>

#include <stddef.h>
#include <arch/atomic.h>
#include <queue.h>
#include <nuttx/usb.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/unipro/unipro.h>

/* uncomment for debugging specific EP */
/* #define USBTUN_DEBUG */
/* #define USBTUN_VDEBUG */
#define USBTUN_DEBUG_EP 0

/* Control message type from APBA to APBE */
#define PCD_NONE 0
#define PCD_ROUTER_READY 1
#define PCD_SETUP 2

/* Control message type from APBE to APBA */
#define HCD_NONE 0
#define HCD_ROUTER_READY 1
#define HCD_SETUP_RESP 2
#define HCD_ENDPOINTS 3
#define HCD_USB_CONNECTED 4
#define HCD_USB_DISCONNECTED 5

#define MAX_ENDPOINTS 16

typedef enum {
    USBTUN_MEM_NONE,
    USBTUN_MEM_UNIPRO,
    USBTUN_MEM_BUFRAM,
} buf_type_t;

typedef struct {
    buf_type_t type;
    void *ptr;
    size_t size;
} usbtun_buf_t;

typedef enum {
    USBTUN_NO_DATA,
    USBTUN_WAIT_DATA,
} usbtun_hdr_res_t;

void *USBTUN_ALLOC(uint32_t size);
void USBTUN_FREE(void *ptr);
void usbtun_print_mem_info(void);

typedef usbtun_hdr_res_t (*usbtun_hdr_handler_t)(uint8_t ep, uint8_t type,
                                                 int16_t code, size_t len);

typedef enum {
    USBTUN_FREE_BUF,
    USBTUN_KEEP_BUF,
} usbtun_data_res_t;

typedef usbtun_data_res_t (*usbtun_data_handler_t)(usbtun_buf_t *buf, uint8_t ep, uint8_t type);

int init_router_common(usbtun_hdr_handler_t hdr_handler, usbtun_data_handler_t data_handler);
void uninit_router_common(void);

sq_entry_t *usbtun_req_dq(uint8_t ep);
void usbtun_req_q(uint8_t ep, sq_entry_t *item);
void usbtun_req_to_usb(uint8_t ep, sq_entry_t *item);
void usbtun_req_from_usb(uint8_t ep, sq_entry_t *item);
bool usbtun_req_is_usb_empry(uint8_t ep);
sq_entry_t * usbtun_req_dq_usb(uint8_t ep);
void usbtun_clean_mem(usbtun_buf_t *buf);

int unipro_send_tunnel_cmd(uint8_t ep, uint8_t type, int16_t code, void *buffer, size_t pkt_len);

#endif /* __USB_TUN_H__ */
