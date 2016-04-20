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

#include <arch/atomic.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/list.h>

#include <nuttx/mhb/ipc.h>

#include "transport.h"

#ifdef CONFIG_MHB_IPC_CLIENT

//pool size should less than 256 (8 bits index in request cookie)
#define TRACKING_REQ_POOL_SIZE 8 //max num of pending client requests

//local wrapper to tracking requests at client slide
//a tracking request is not in pool's free list
struct tracking_request_s
{
    struct request_s *req; //pointer to the variable at ipc caller's stack
    sem_t sem; //used to wait response
    struct list_head pool; //in pool free list

    /* Save response here until being retrieved by the caller.
     * Caller must call release_reponse to release the memory allocated
     * by transport (i.e. the tracking_request_s).
     */
    union {
        char response_pkt[IPC_PACKET_MAX_SIZE];
        struct response_s response;
    };
};

struct tr_pool_s
{
    struct tracking_request_s tr_pool_mem[TRACKING_REQ_POOL_SIZE];
    pthread_mutex_t lock;
    struct list_head pool_list;
};

static struct tr_pool_s tr_pool;
static struct tracking_request_s *find_tracking_request(uint32_t cookie);

inline uint32_t make_cookie(uint32_t index, uint32_t seq)
{
    return (index << 24) | (seq & 0xFFFFFF);
}

inline uint32_t get_index_from_cooke(uint32_t cookie)
{
    return cookie >> 24;
}

static void init_tracking_req_pool(void)
{
    uint32_t i;
    list_init(&tr_pool.pool_list);
    for (i=0; i<TRACKING_REQ_POOL_SIZE; i++) {
        list_add(&tr_pool.pool_list, &tr_pool.tr_pool_mem[i].pool);
    }
}

// allocate a tracking request from the pool
static struct tracking_request_s* zalloc_tracking_req(void)
{
    struct list_head *first;
    struct tracking_request_s *req;

    pthread_mutex_lock(&tr_pool.lock);
    if (list_is_empty(&tr_pool.pool_list)) {
        IPC_ERR("tracking req pool empty\n");
        req = NULL;
    } else {
        first = tr_pool.pool_list.next;
        list_del(first);
        req = list_entry(first, struct tracking_request_s, pool);
        memset(req, 0, sizeof(*req));
    }
    pthread_mutex_unlock(&tr_pool.lock);

    return req;
}

//return a tracking request to the pool
static void dealloc_tracking_req(struct tracking_request_s* tracking_req)
{
    pthread_mutex_lock(&tr_pool.lock);
    list_add(&tr_pool.pool_list, &tracking_req->pool);
    pthread_mutex_unlock(&tr_pool.lock);
}

/*
 * Save the response and wakeup caller to get the response.
 * Called by unipro message thread handler.
 */
void ipc_handle_response(struct response_s *response)
{
    struct tracking_request_s *req;

    IPC_DBG("got a response cookie 0x%x res %d\n",
        response->cookie, response->result);
    req = find_tracking_request(response->cookie);
    if (req != NULL) {
        memcpy(req->response_pkt, response,
            sizeof(*response) + response->param_len);
        sem_post(&req->sem);
    } else {
        IPC_ERR("request not found (timedout?)\n");
    }
}

/*
 * wait for a request then remove it from the tracking list.
 */
static int wait_for_response(struct request_s *req,
        struct response_s **response, uint64_t timeout /*ms*/)
{
    int retval;
    sem_t *sem;
    uint32_t index;
    struct tracking_request_s *tracking_req;

    sem = NULL;
    //pending request must have a valid cookie
    index = get_index_from_cooke(req->cookie);
    if (index < TRACKING_REQ_POOL_SIZE) {
        tracking_req = &tr_pool.tr_pool_mem[index];
        sem = &tracking_req->sem;
    }

    if (sem != NULL) {
        struct timespec abstime;

        clock_gettime(CLOCK_REALTIME, &abstime);
        abstime.tv_nsec += timeout * 1000000;
        if (abstime.tv_nsec > 1000000000) {
            abstime.tv_sec++;
            abstime.tv_nsec -= 1000000000;
        }

        retval = sem_timedwait(sem, &abstime);

        if (retval != 0) {
            IPC_ERR("wait sem failed %d\n", errno);
            *response = NULL;
            dealloc_tracking_req(tracking_req);
            MEM_DBG("free tracking req at %p\n", tracking_req);
        } else {
            //response should be released later by calling release_response
            *response = &tracking_req->response;
            //do not free tracking_req which contains the response data
            //release_response will release the whole tracking_req.
        }

        sem_destroy(sem);
        return retval;
    } else {
        IPC_ERR("request not found in the tracking list\n");
        *response = NULL;
        return -1;
    }
}

/**/
static uint32_t create_request_cookie(struct tracking_request_s * outreq) {
    static atomic_t seq;
    uint32_t new_seq = atomic_inc(&seq);

    uint32_t index = outreq - &tr_pool.tr_pool_mem[0];
    return make_cookie(index, new_seq);
}

/*
 * Send a request and put it in tracking list if succeed to send.
 */
static int send_request(struct request_s *req, void *payload, size_t payload_size)
{
    int retval;

    struct tracking_request_s *outreq;
    outreq = zalloc_tracking_req();
    if (outreq == NULL) {
        return -ENOMEM;
    }
    MEM_DBG("allocate tracking req at %p\n", outreq);

    req->cookie = create_request_cookie(outreq);
    sem_init(&outreq->sem, 0, 0);
    outreq->req = req;
    outreq->response.result = -EINVAL;

    retval = send_ipc_packet(req, sizeof(*req), payload, payload_size);
    if (retval != 0) {
        MEM_DBG("free tracking req at %p\n", outreq);
        IPC_ERR("failed to send request to ipc server\n");
        dealloc_tracking_req(outreq);
    }

    return retval;
}

/* Find the correct tracking request from cookie
 * Called when a response comes.
 */
static struct tracking_request_s *find_tracking_request(uint32_t cookie)
{
    uint32_t index;
    struct tracking_request_s *req;

    index = get_index_from_cooke(cookie);
    req = NULL;
    if (index < TRACKING_REQ_POOL_SIZE) {
        req = &tr_pool.tr_pool_mem[index];

        //check if it's a pending request
        pthread_mutex_lock(&tr_pool.lock);
        if (req->pool.next != NULL) {
            //in pool free list, not in tracking
            req = NULL;
        }
        pthread_mutex_unlock(&tr_pool.lock);

        //check if a timedout tracking request is re-used
        //i.e. same index, different seq
        if (req != NULL && req->req->cookie != cookie) {
            req = NULL;
        }
    }

    return req;
}

/*
 * Called by client app to request ipc service.
 * Caller must call release_response if caller gets non null out_param.
 * Note tracking_request_s is invisible for this function.
 */
int ipc_request_sync(uint32_t server_app_id, void* in_param,
        uint32_t in_param_len, void **out_param, uint32_t *out_len,
        uint32_t timeout_ms)
{
    int retval;
    size_t packet_size;
    struct request_s request; //request header
    struct response_s *response;

    if (up_interrupt_context()) {
        IPC_ERR("can not call from interrupt context\n");
        return -EINVAL;
    }

    packet_size = sizeof(request) + in_param_len;
    if (packet_size > IPC_PACKET_MAX_SIZE) {
        IPC_ERR("param should be less than %d bytes\n",
            IPC_PACKET_MAX_SIZE - sizeof(request));
        return -EINVAL;
    }

    if (packet_size > CPORT_BUF_SIZE) {
        IPC_ERR("param should be less than %d bytes\n",
            CPORT_BUF_SIZE - sizeof(request));
        return -EINVAL;
    }

    request.packet_type = IPC_PACKET_REQUEST_TYPE;
    request.app_id = server_app_id;
    request.param_len = in_param_len;

    retval = send_request(&request, in_param, in_param_len);
    if (retval != 0) {
        IPC_ERR("failed to send request\n");
        return retval;
    }

    response = NULL;
    retval = wait_for_response(&request, &response, timeout_ms);
    if (retval != 0) {
        IPC_ERR("failed to wait for response\n");
        return retval;
    }

    *out_param = NULL;
    *out_len = 0;
    if (response != NULL) {
        //response should be released here if not passed to the caller.
        retval = response->result;
        if (response->param_len == 0) {
            release_response(response->param);
        } else if (out_param != NULL && out_len != NULL) {
            //pass response data to caller
            *out_param = response->param;
            *out_len = response->param_len;
        } else {
            IPC_DBG("response data ignored by caller\n");
            release_response(response->param);
        }
    } else {
        //error since we do not support async call for now
        IPC_ERR("no response packet for req cookie 0x%x\n", request.cookie);
    }
    return retval;
}

/*
 * Tracking request contains the response from ipc server.
 * After client retrieves the response data, it must call this function
 * to release the tracking request.
 *
 * out_param - the original response data passed to client.
 */
void release_response(void* out_param)
{
    struct tracking_request_s *tracking_req = NULL;
    struct response_s *response = NULL;

    IPC_DBG("param at %p\n", out_param);
    if (out_param != NULL) {
        response = container_of(out_param, struct response_s, param);
        tracking_req = container_of(response,
            struct tracking_request_s, response);
        MEM_DBG("free tracking req at %p\n", tracking_req);
        dealloc_tracking_req(tracking_req);
    }
}

int ipc_client_init(void)
{
    int retval;
    IPC_DBG("enter\n");

    retval = pthread_mutex_init(&tr_pool.lock, NULL);
    if (retval != 0) {
        IPC_ERR("pool mutex init fail: %d\n", retval);
    }

    init_tracking_req_pool();
    return retval;
}

void ipc_client_deinit(void)
{
    pthread_mutex_destroy(&tr_pool.lock);
}
#endif

