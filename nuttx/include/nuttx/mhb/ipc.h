/*
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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

#ifndef ICE_BRIDGE_IPC_H
#define ICE_BRIDGE_IPC_H

#include <stdint.h>
#define __packed    __attribute__((packed))

/* Unipro currently uses a pool of ring buffers which are allocated at
 * startup using the bufram allocator.
 *
 * The ring buffer adds overhead of 32 bytes for a ring buffer control
 * struct.  The bufram itself adds overhead of 16 bytes.  The bufram
 * allocator will include these overhead sizes when trying allocate
 * the buffer, and the total size allocated must be a power of two.
 * This causes the nominal allocation of 128 bytes to actually fit in
 * the bufram bucket for size 256 allocations, because it requires 128
 * bytes plus overhead of 48 bytes.  Therefore, we can fit 208 bytes
 * as well in the 256 bucket (208 + 48 = 256).
 */
#define IPC_PACKET_MAX_SIZE 208

#define IPC_FOURCC(a,b,c,d) ((uint32_t)(a) | ((uint32_t)(b)<<8) \
        | ((uint32_t)(c)<<16) | ((uint32_t)(d)<<24))

#define IPC_APP_ID_IPC    IPC_FOURCC(' ', 'i', 'p', 'c')
#define IPC_APP_ID_CDSI   IPC_FOURCC('c', 's', 'd', 'i')
#define IPC_APP_ID_UNIRPO IPC_FOURCC('u', 'n', 'i', 'p')
#define IPC_APP_ID_I2S    IPC_FOURCC(' ', 'i', '2', 's')

/*
 * bridge ipc module provides a transport layer for two apps to talk from
 * different bridge cores. It's the app's duty to pack/unpack input/output
 * parameters (could be as simple as structure cast).
 *
 * server side calls register_ipc_handler to provide ipc service
 * client side calls ipc_request_sync to send ipc request
 */

#ifdef CONFIG_MHB_IPC_SERVER
/*
 * IPC service callback. return value will be sent to client. out_response_param
 * to client is optional.
 * Request/response parameters are transparent to ipc transport.
 * IPC client/server must have exact same definitions of these parameters.
 */
typedef uint32_t (*ipc_handler_t)(void *in_request_param, uint32_t in_len,
        void **out_response_param, uint32_t *out_len);

/*
 * IPC service provides this callback if the response data needs to be cleanup.
 */
typedef uint32_t (*release_response_handler_t)(void *response, uint32_t len);

/*
 * app_id is IPC_APP_ID_XXXX. Each app can only have one handler. The second
 * register will fail.
 * return 0 success otherwise -1.
 */
int register_ipc_handler(int my_app_id, ipc_handler_t handler,
        release_response_handler_t cleanup);

/*
 * handler is required in case a service is wrongly released by other app
 */
int unregister_ipc_handler(int my_app_id, ipc_handler_t handler);
#endif

#ifdef CONFIG_MHB_IPC_CLIENT
/*
 * in_param will be passed to the remote service handler.
 * out_param will be filled by transport if any.
 * return 0 success otherwise failed (-ETIMEDOUT may return)
 */
int ipc_request_sync(uint32_t server_app_id, void* in_param, uint32_t in_param_len,
        void **out_param, uint32_t *out_len, uint32_t timeout_ms);

/*
 * must be called if out_param is not null from ipc_request_sync
 */
void release_response(void* out_param);
#endif

/*
 * Get the max sync data size.
 */
size_t ipc_get_sync_data_sz(void);

/*
 * initialize ipc
 * called before any ipc service registration.
 */
int ipc_init(void);

/*
 * deinitialize ipc
 */
void ipc_deinit(void);

/*
 * notify ipc to register unipro driver when link is up
 */
int ipc_register_unipro(void);

/*
 * notify ipc to unregister unipro driver when link is down
 */
int ipc_unregister_unipro(void);
#endif

