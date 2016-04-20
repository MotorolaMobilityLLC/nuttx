/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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
#ifndef MHB_IPC_TRANSPORT_H
#define MHB_IPC_TRANSPORT_H

#include <nuttx/mhb/ipc.h>

//#define DEBUG
#ifdef DEBUG
    #define IPC_IRQ_DBG lldbg
    #ifdef CONFIG_NSH_CONSOLE
        #define MEM_DBG printf
        #define IPC_DBG printf
    #else
        #define MEM_DBG lldbg
        #define IPC_DBG lldbg
    #endif
#else
    #define IPC_IRQ_DBG(x...)
    #define MEM_DBG(x...)
    #define IPC_DBG(x...)
#endif

#ifdef CONFIG_NSH_CONSOLE
    #define IPC_ERR printf
#else
    #define IPC_ERR lldbg
#endif

//irq handler can not use printf
#define IPC_IRQ_ERR lldbg

#define TIMEOUT_WAIT_UNIPRO_READY 500 //ms
#define IPC_PACKET_REQUEST_TYPE  0
#define IPC_PACKET_RESPONSE_TYPE 1

#define container_of(ptr, type, member) ({               \
    const typeof( ((type *)0)->member ) *__mptr = (ptr); \
    (type *)( (char *)__mptr - offsetof(type,member) );})

//ipc request packet format
struct request_s
{
    uint32_t packet_type; //always IPC_PACKET_REQUEST_TYPE
    uint32_t cookie; //8 bit index, 24 bit seq
    uint32_t app_id;
    uint32_t param_len;
    char param[0];
} __packed;

//ipc response packet format
struct response_s
{
    uint32_t packet_type; //always IPC_PACKET_RESPONSE_TYPE
    uint32_t cookie; //the original request cookie
    uint32_t result;
    uint32_t param_len;
    char param[0];
} __packed;

/*
 * handle the incoming ipc response packet.
 */
void ipc_handle_response(struct response_s *response_packet);

/*
 * handle incoming ipc request packet.
 */
void ipc_handle_request(struct request_s *request_packet);

int ipc_server_init(void);

int ipc_client_init(void);

void ipc_client_deinit(void);

void ipc_server_deinit(void);

/*
 * send ipc packet
 * return 0 - work delivered to tx thread,otherwise fails
 */
int send_ipc_packet(void *hdr, size_t hdr_len,
        void *payload, size_t payload_size);

#endif
