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

#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/list.h>

#include <nuttx/mhb/ipc.h>

#include "transport.h"

#ifdef CONFIG_MHB_IPC_SERVER
//ipc service callback wrapper
struct ipc_handler_entry_s
{
    uint32_t app_id;
    ipc_handler_t ipc_handler;
    release_response_handler_t cleanup_handler;
    struct list_head list;
};

//registered ipc services
static struct list_head ipc_handlers;
static pthread_mutex_t ipc_handlers_lock;

int register_ipc_handler(int app_id, ipc_handler_t ipc_handler,
    release_response_handler_t cleanup_handler)
{
    struct ipc_handler_entry_s *node;
    struct list_head *iter;
    int found = 0;

    //check if exists
    pthread_mutex_lock(&ipc_handlers_lock);
    list_foreach(&ipc_handlers, iter) {
        node = list_entry(iter, struct ipc_handler_entry_s, list);
        if (node->app_id == app_id) {
            found = 1;
            break;
        }
    }
    pthread_mutex_unlock(&ipc_handlers_lock);
    if (found) return -EEXIST;

    node = zalloc(sizeof(*node));
    if (node == NULL) {
        return -ENOMEM;
    }
    MEM_DBG("allocate handler node at %p\n", node);

    node->app_id = app_id;
    node->ipc_handler = ipc_handler;
    node->cleanup_handler = cleanup_handler;

    pthread_mutex_lock(&ipc_handlers_lock);
    list_add(&ipc_handlers, &node->list);
    pthread_mutex_unlock(&ipc_handlers_lock);

    IPC_DBG("register ipc handler for %c%c%c%c\n",
        app_id & 0xFF, (app_id >> 8) & 0xFF,
        (app_id >> 16) & 0xFF, (app_id >> 24) & 0xFF);
    return 0;
}

int unregister_ipc_handler(int app_id, ipc_handler_t ipc_handler)
{
    struct ipc_handler_entry_s *node;
    struct list_head *iter, *tmp_iter;

    int found = 0;
    pthread_mutex_lock(&ipc_handlers_lock);
    list_foreach_safe(&ipc_handlers, iter, tmp_iter) {
        node = list_entry(iter, struct ipc_handler_entry_s, list);
        if (node->app_id == app_id && node->ipc_handler == ipc_handler) {
            list_del(iter);
            IPC_DBG("remove ipc handler for %08x\n", app_id);
            free(node);
            MEM_DBG("free handler node at %p\n", node);
            found = 1;
        }
    }
    pthread_mutex_unlock(&ipc_handlers_lock);

    return found ? 0 : -1;
}

void ipc_handle_request(struct request_s *req_data)
{
    int retval;
    struct ipc_handler_entry_s *node;
    ipc_handler_t ipc_handler;
    release_response_handler_t cleanup;
    struct list_head *iter;
    void *out_param;
    size_t out_len;
    size_t reponse_pkt_size;
    struct response_s response;

    ipc_handler = NULL;
    cleanup = NULL;
    pthread_mutex_lock(&ipc_handlers_lock);
    list_foreach(&ipc_handlers, iter) {
        node = list_entry(iter, struct ipc_handler_entry_s, list);
        if (node->app_id == req_data->app_id) {
            ipc_handler = node->ipc_handler;
            cleanup = node->cleanup_handler;
            break;
        }
    }
    pthread_mutex_unlock(&ipc_handlers_lock);

    out_param = NULL;
    out_len = 0;
    retval = -ENOSYS;
    if (ipc_handler != NULL) {
        retval = ipc_handler(req_data->param, req_data->param_len,
            &out_param, &out_len);
    }

    if (out_param == NULL) {
        if (out_len != 0) {
            IPC_ERR("out_len should be 0 when out_param is null\n");
            out_len = 0;
        }
    }

    //make the response packet
    reponse_pkt_size = sizeof(response) + out_len;
    if (reponse_pkt_size > IPC_PACKET_MAX_SIZE) {
        IPC_ERR("response pkt size exceeds limit %d>%d\n", out_len,
            IPC_PACKET_MAX_SIZE - sizeof(response));
        //force to return an error
        retval = -ENOMEM;
    }

    response.packet_type = IPC_PACKET_RESPONSE_TYPE;
    response.cookie = req_data->cookie;
    response.result = retval;
    response.param_len = out_len;

    retval = send_ipc_packet(&response, sizeof(response), out_param, out_len);
    if (retval != 0) {
        IPC_ERR("failed to send ipc response\n");
    } else {
        IPC_DBG("handle ipc request for %c%c%c%c result %d\n",
            req_data->app_id & 0xFF,
            (req_data->app_id >> 8)  & 0xFF,
            (req_data->app_id >> 16) & 0xFF,
            (req_data->app_id >> 24) & 0xFF,
            response.result);
    }

    if (cleanup != NULL) {
        cleanup(out_param, out_len);
    }
}

int ipc_server_init(void)
{
    int retval;
    list_init(&ipc_handlers);
    retval = pthread_mutex_init(&ipc_handlers_lock, NULL);
    if (retval != 0) {
        IPC_ERR("ipc handlers mutex init fail: %d\n", retval);
    }
    return retval;
}

void ipc_server_deinit(void)
{
    pthread_mutex_destroy(&ipc_handlers_lock);
}
#endif

