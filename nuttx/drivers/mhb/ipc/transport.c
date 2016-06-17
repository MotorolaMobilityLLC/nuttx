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

#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arch/chip/unipro_p2p.h>
#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/ring_buf.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/list.h>

#include <nuttx/mhb/ipc.h>

#include "transport.h"

/* need a ring buffer and a thread to switch from interrupt context to thread
 * context to further process the response or request.
 */
#define BUFF_SIZE 8
static pthread_t rx_thread_handler;
static pthread_t tx_thread_handler;

//control flag of rx interrupt handler and rx/tx thread. 0 - stop
static volatile uint32_t ipc_control;

struct ipc_ring_buffer_s
{
    struct ring_buf* ring_buffer;
    struct ring_buf* volatile rbc; //consumer
    struct ring_buf* volatile rbp; //producer
    sem_t sem;
};
static struct ipc_ring_buffer_s rx_ring_buffer;
static struct ipc_ring_buffer_s tx_ring_buffer;
static pthread_mutex_t tx_producer_lock;

/*
 * Provide the max data size which can be sent by ipc_request_sync.
 */
size_t ipc_get_sync_data_sz(void)
{
    return (IPC_PACKET_MAX_SIZE-sizeof(struct response_s));
}

/* Receive ipc request/response in thread handler */
static void *unipro_rx_thread_func(void *data)
{
    uint32_t *pkg_type;
    struct ring_buf *rb;
    size_t pkt_size;
    void *pkt_data;
#ifdef CONFIG_MHB_IPC_CLIENT
    struct response_s *resp_msg;
#endif

#ifdef CONFIG_MHB_IPC_SERVER
    struct request_s *req_msg;
#endif

    while (1) {
        sem_wait(&rx_ring_buffer.sem);
        if (!ipc_control) break;

        rb = rx_ring_buffer.rbc;
        if (!ring_buf_is_consumers(rb)) {
            IPC_ERR("emtpy ring buffer\n");
            continue;
        }
        pkt_size = ring_buf_len(rb);
        if (pkt_size < sizeof(uint32_t)) {
            IPC_ERR("invalid packet received bytes %d\n", pkt_size);
        } else {
            pkt_data = ring_buf_get_data(rb);
            pkg_type = (uint32_t*) pkt_data;
            if (*pkg_type == IPC_PACKET_RESPONSE_TYPE) {
#ifdef CONFIG_MHB_IPC_CLIENT
                resp_msg = (struct response_s*)pkt_data;
                if (pkt_size ==
                        sizeof(*resp_msg) + resp_msg->param_len) {
                    ipc_handle_response(resp_msg);
                } else {
                    IPC_ERR("wrong response size\n");
                }
#else
                IPC_ERR("receive unwanted response\n");
#endif
            } else if (*pkg_type == IPC_PACKET_REQUEST_TYPE) {
#ifdef CONFIG_MHB_IPC_SERVER
                req_msg = (struct request_s*)pkt_data;
                if (pkt_size == sizeof(*req_msg) + req_msg->param_len) {
                    ipc_handle_request(req_msg);
                } else {
                    IPC_ERR("wrong request szie\n");
                }
#else
                IPC_ERR("receive unwanted request\n");
#endif
            } else {
                IPC_ERR("ipc: unknown packet type\n");
            }
        }

        rx_ring_buffer.rbc = ring_buf_get_next(rb);
        ring_buf_reset(rb);
        ring_buf_pass(rb);
    }
    return NULL;
}

/* Receive ipc request/response (in interrupt context) */
static int unipro_rx_irq_handler(unsigned int cport, void *data, size_t size)
{
    struct ring_buf *rb;

    IPC_IRQ_DBG("recieve %d bytes\n", size);
    if (!ipc_control) {
        IPC_IRQ_ERR("ipc not ready\n");
    } else if (cport != CONFIG_MHB_IPC_CPORT_ID) {
        IPC_IRQ_ERR("receive ipc data from wrong cport %d\n", cport);
    } else if (size > IPC_PACKET_MAX_SIZE) {
        IPC_IRQ_ERR("packet size %d exceeds %d, discarded\n",
            size, IPC_PACKET_MAX_SIZE);
    } else {
        rb = rx_ring_buffer.rbp;
        if (ring_buf_is_producers(rb)) {
            memcpy(ring_buf_get_data(rb), data, size);
            ring_buf_put(rb, size);
            ring_buf_pass(rb);

            rx_ring_buffer.rbp = ring_buf_get_next(rb);
            sem_post(&rx_ring_buffer.sem);
        } else {
            IPC_IRQ_ERR("no ring buffer to hold incoming message\n");
        }
    }

    unipro_rxbuf_free(CONFIG_MHB_IPC_CPORT_ID, data);

    return 0;
}

static void *unipro_tx_thread_func(void *data)
{
    struct ring_buf *rb;
    size_t pkt_size;
    void *pkt_data;
    int retval;

    while (1) {
        sem_wait(&tx_ring_buffer.sem);
        if (!ipc_control) break;

        rb = tx_ring_buffer.rbc;
        if (!ring_buf_is_consumers(rb)) {
            IPC_ERR("empty ring buffer\n");
            continue;
        }
        pkt_size = ring_buf_len(rb);
        pkt_data = ring_buf_get_data(rb);

        IPC_DBG("send %d bytes\n", pkt_size);
        retval = unipro_send(CONFIG_MHB_IPC_CPORT_ID, pkt_data, pkt_size);
        if (retval != 0) {
            IPC_ERR("failed to send ipc packet\n");
        }
        tx_ring_buffer.rbc = ring_buf_get_next(rb);
        ring_buf_reset(rb);
        ring_buf_pass(rb);
    }

    return NULL;
}

int send_ipc_packet(void *hdr, size_t hdr_len,
        void *payload, size_t payload_size)
{
    struct ring_buf *rb;
    char *buf;
    int retval;

    retval = -EINVAL;
    pthread_mutex_lock(&tx_producer_lock);
    rb = tx_ring_buffer.rbp;
    if (ring_buf_is_producers(rb)) {
        //can't make use of ring buffer's head room, since
        //ipc request/response have different header
        buf = ring_buf_get_data(rb);
        memcpy(buf, hdr, hdr_len);
        buf += hdr_len;
        memcpy(buf, payload, payload_size);
        ring_buf_put(rb, hdr_len + payload_size);

        tx_ring_buffer.rbp = ring_buf_get_next(rb);
        ring_buf_pass(rb);
        sem_post(&tx_ring_buffer.sem);
        retval = 0;
    } else {
        IPC_ERR("no ring buffer to hold outgoing message\n");
    }
    pthread_mutex_unlock(&tx_producer_lock);

    return retval;
}

static struct unipro_driver ipc_driver = {
    .name = "ipc",
    .rx_handler = unipro_rx_irq_handler,
};

/* should be called after unipro init*/
int ipc_init(void)
{
    int retval;

    //rx initialize
    rx_ring_buffer.ring_buffer = ring_buf_alloc_ring(BUFF_SIZE,
        0, IPC_PACKET_MAX_SIZE, 0, NULL, NULL, NULL);
    if (rx_ring_buffer.ring_buffer == NULL) {
        IPC_ERR("ipc rx ring buffer alloc failed\n");
        return -ENOMEM;
    }
    rx_ring_buffer.rbc = rx_ring_buffer.rbp = rx_ring_buffer.ring_buffer;
    sem_init(&rx_ring_buffer.sem, 0, 0);

    retval = pthread_create(&rx_thread_handler,
            NULL, unipro_rx_thread_func, NULL);
    if (retval != 0) {
        IPC_ERR("failed to create rx handler thread\n");
        goto rx_thread_init_err;
    }

    //tx initialize
    tx_ring_buffer.ring_buffer = ring_buf_alloc_ring(BUFF_SIZE,
        0, IPC_PACKET_MAX_SIZE, 0, NULL, NULL, NULL);
    if (tx_ring_buffer.ring_buffer == NULL) {
        IPC_ERR("ipc tx ring buffer alloc failed\n");
        retval = -ENOMEM;
        goto tx_ring_buffer_init_err;
    }
    tx_ring_buffer.rbc = tx_ring_buffer.rbp = tx_ring_buffer.ring_buffer;
    sem_init(&tx_ring_buffer.sem, 0, 0);

    retval = pthread_create(&tx_thread_handler,
            NULL, unipro_tx_thread_func, NULL);

    if (retval != 0) {
        IPC_ERR("failed to create tx handler thread\n");
        goto tx_thread_init_err;
    }

    retval = pthread_mutex_init(&tx_producer_lock, NULL);
    if (retval != 0) {
        IPC_ERR("failed to init tx producer lock\n");
        goto tx_producer_lock_err;
    }

#ifdef CONFIG_MHB_IPC_CLIENT
    retval = ipc_client_init();
    if (retval != 0) {
        IPC_ERR("client init failed %d\n", retval);
        goto client_init_err;
    }
#endif

#ifdef CONFIG_MHB_IPC_SERVER
    retval = ipc_server_init();
    if (retval != 0) {
        IPC_ERR("server init failed %d\n", retval);
        goto server_init_err;
    }
#endif

    ipc_control = 1;
    return 0;

#ifdef CONFIG_MHB_IPC_SERVER
    //ipc_server_deinit(); //uncomment if more init after ipc_server_init()
server_init_err:
#endif
#ifdef CONFIG_MHB_IPC_CLIENT
    ipc_client_deinit();
client_init_err:
#endif
    pthread_mutex_destroy(&tx_producer_lock);
tx_producer_lock_err:
    sem_post(&tx_ring_buffer.sem);//quit tx thread
    pthread_join(tx_thread_handler, NULL);
tx_thread_init_err:
    ring_buf_free_ring(tx_ring_buffer.ring_buffer, NULL, NULL);
    sem_destroy(&tx_ring_buffer.sem);
tx_ring_buffer_init_err:
    sem_post(&rx_ring_buffer.sem);//quit rx thread
    pthread_join(rx_thread_handler, NULL);
rx_thread_init_err:
    ring_buf_free_ring(rx_ring_buffer.ring_buffer, NULL, NULL);
    sem_destroy(&rx_ring_buffer.sem);

    return retval;
}

int ipc_register_unipro(void)
{
    int retval;

    IPC_DBG("register unipro\n");
    retval = unipro_driver_register(&ipc_driver,
        CONFIG_MHB_IPC_CPORT_ID);
    if (retval != 0) {
        IPC_ERR("failed to register ipc driver\n");
    } else {
#ifdef CONFIG_UNIPRO_P2P_APBA
        IPC_DBG("connect to ipc cport\n");
        unipro_p2p_setup_connection(CONFIG_MHB_IPC_CPORT_ID);
#endif
    }
    return retval;
}

int ipc_unregister_unipro(void)
{
    IPC_DBG("unregister unipro\n");
#ifdef CONFIG_UNIPRO_P2P_APBA
    unipro_p2p_reset_connection(CONFIG_MHB_IPC_CPORT_ID);
#endif
    unipro_driver_unregister(CONFIG_MHB_IPC_CPORT_ID);
    return 0;
}

//TODO: where to call this func?
void ipc_deinit(void)
{
#ifdef CONFIG_MHB_IPC_CLIENT
    ipc_client_deinit();
#endif
#ifdef CONFIG_MHB_IPC_SERVER
    ipc_server_deinit();
#endif
    ipc_control = 0;

    sem_post(&rx_ring_buffer.sem);//wake to exit thread
    pthread_join(rx_thread_handler, NULL);
    sem_destroy(&rx_ring_buffer.sem);
    ring_buf_free_ring(rx_ring_buffer.ring_buffer, NULL, NULL);

    sem_post(&tx_ring_buffer.sem);//wake to exit thread
    pthread_join(tx_thread_handler, NULL);
    sem_destroy(&tx_ring_buffer.sem);
    ring_buf_free_ring(tx_ring_buffer.ring_buffer, NULL, NULL);

    pthread_mutex_destroy(&tx_producer_lock);
}
