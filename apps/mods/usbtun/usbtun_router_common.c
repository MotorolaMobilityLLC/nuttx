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
#include "usbtun.h"

#include <debug.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>

#include <arch/chip/unipro_p2p.h>
#include <nuttx/bufram.h>
#include <nuttx/irq.h>

struct usbtun_hdr_s {
    uint32_t magic;
    uint8_t ep;
    uint8_t type;
    int16_t code;
    size_t pkt_len;
};

#define USBTUN_HDR_MAGIC 0x2468fdb9

#ifndef CONFIG_MODS_USBTUN_CPORT_ID
#define CONFIG_MODS_USBTUN_CPORT_ID 13
#endif

#define NUM_UNIPRO_TX_ITEMS 5
#define NUM_UNIPRO_RX_ITEMS CONFIG_TSB_UNIPRO_MAX_INFLIGHT_BUFCOUNT + 5

/*
 * structure used to pool usbtun_hdr_s.
 * usbtun_hdr_s have to be allocated from bufram and we pool them
 * in upfront.
*/
struct unipro_tx_s {
    sq_entry_t entry;
    struct usbtun_hdr_s *hdr;
};


/*
 * structure used to pool incoming unipro_rx data.
*/
struct unipro_rx_s {
    sq_entry_t entry;
    void *data;
    size_t size;
};

/*
 * The requests queued in usb or unipro are also pooled by either
 * hcd router or pcd router. This structure provides common
 * queue so all pooled items.
*/
struct ep_req_list_s {
    /* free ep_req_s items */
    sq_queue_t free;
    /* items pending usb transfer */
    sq_queue_t usb;
};

struct unipro_rx_context_s {
    uint8_t ep;
    uint8_t type;
    size_t len;
    void *seg_ptr;
    uint32_t seg_offset;
};

struct usb_router_common_s {
    sq_queue_t tx_free;
    sq_queue_t rx_free;
    sq_queue_t rx_q;
    pthread_t rx_thread;
    sem_t rx_sem;
    bool rx_run;
    struct unipro_tx_s tx_items[NUM_UNIPRO_TX_ITEMS];
    /* Guarantee rx item won't be running out before unipro_rx buf. */
    struct unipro_rx_s rx_items[NUM_UNIPRO_RX_ITEMS];
    struct ep_req_list_s req_list[MAX_ENDPOINTS];
    usbtun_hdr_handler_t hdr_handler;
    usbtun_data_handler_t data_handler;
};

static struct usb_router_common_s s_common;

static int unipro_rx_handler(unsigned int cport, void *data, size_t size);
static int start_unipro_rx_thread(void);
static void stop_unipro_rx_thread(void);

static int count = 0;

static struct unipro_driver route_driver = {
    .name = "usbtun",
    .rx_handler = unipro_rx_handler,
};

void *USBTUN_ALLOC(uint32_t size) {
#ifdef CONFIG_DWC_USE_BUFRAM
    void *buf = bufram_alloc(size);

    if (buf)
        memset(buf, 0, size);

    return buf;
#else
    return zalloc(size);
#endif
}

void USBTUN_FREE(void *ptr) {
#ifdef CONFIG_DWC_USE_BUFRAM
    bufram_free(ptr);
#else
    free(ptr);
#endif
}

void usbtun_print_mem_info(void) {
    struct mallinfo mem;
#ifdef CONFIG_CAN_PASS_STRUCTS
    mem = mallinfo();
#else
    (void)mallinfo(&mem);
#endif
    lldbg("total:%d, used:%d, free:%d, largest:%d\n",
          mem.arena, mem.uordblks, mem.fordblks, mem.mxordblk);
}

int init_router_common(usbtun_hdr_handler_t hdr_handler, usbtun_data_handler_t data_handler) {
    int i;
    irqstate_t flags = irqsave();

    memset(&s_common, 0, sizeof(s_common));

    /* Prepare TX item pool used to keep track of unipro TX direction */
    sq_init(&s_common.tx_free);
    for (i = 0; i < NUM_UNIPRO_TX_ITEMS; i++) {
        void *ptr = bufram_alloc(sizeof(struct usbtun_hdr_s));
        if (ptr) {
            s_common.tx_items[i].hdr = ptr;
            sq_addlast(&s_common.tx_items[i].entry, &s_common.tx_free);
        } else
            lldbg("Failed to allocate bufram for tun_hdr\n");
    }

    sem_init(&s_common.rx_sem, 0, 0);
    sq_init(&s_common.rx_free);
    for (i = 0; i < NUM_UNIPRO_RX_ITEMS; i++) {
        sq_addlast(&s_common.rx_items[i].entry, &s_common.rx_free);
        count++;
    }

    for (i = 0; i < MAX_ENDPOINTS; i++) {
        sq_init(&s_common.req_list[i].free);
        sq_init(&s_common.req_list[i].usb);
    }

    irqrestore(flags);

    s_common.hdr_handler = hdr_handler;
    s_common.data_handler = data_handler;

    start_unipro_rx_thread();

    return 0;
}

void uninit_router_common(void) {
    int i;

    stop_unipro_rx_thread();

    irqstate_t flags = irqsave();
    for (i = 0; i < NUM_UNIPRO_TX_ITEMS; i++) {
        if (s_common.tx_items[i].hdr) {
            bufram_free(s_common.tx_items[i].hdr);
        }
    }
    sq_init(&s_common.tx_free);

    for (i = 0; i < NUM_UNIPRO_RX_ITEMS; i++) {
        if (s_common.rx_items[i].data) {
            bufram_free(s_common.rx_items[i].data);
        }
    }
    sq_init(&s_common.rx_free);

    for (i = 0; i < MAX_ENDPOINTS; i++) {
        sq_init(&s_common.req_list[i].free);
        sq_init(&s_common.req_list[i].usb);
    }

    irqrestore(flags);

    s_common.hdr_handler = NULL;
    s_common.data_handler = NULL;
}

static struct unipro_tx_s *dequeue_tx_item(void) {

    struct unipro_tx_s *tx = NULL;

    irqstate_t flags = irqsave();
    if (!sq_empty(&s_common.tx_free))
        tx = (struct unipro_tx_s *)sq_remfirst(&s_common.tx_free);
    irqrestore(flags);

    return tx;
}

static void enqueue_tx_item(struct unipro_tx_s *item) {
    irqstate_t flags = irqsave();
    sq_addlast(&item->entry, &s_common.tx_free);
    irqrestore(flags);
}

sq_entry_t *usbtun_req_dq(uint8_t ep) {
    sq_entry_t *ret = NULL;

    irqstate_t flags = irqsave();
    if (!sq_empty(&s_common.req_list[ep].free))
        ret = sq_remfirst(&s_common.req_list[ep].free);
    irqrestore(flags);
    return ret;
}

void usbtun_req_q(uint8_t ep, sq_entry_t *item) {
    irqstate_t flags = irqsave();
    sq_addlast(item, &s_common.req_list[ep].free);
    irqrestore(flags);
}

void usbtun_req_to_usb(uint8_t ep, sq_entry_t *item) {
    irqstate_t flags = irqsave();
    sq_addlast(item, &s_common.req_list[ep].usb);
    irqrestore(flags);
}

void usbtun_req_from_usb(uint8_t ep, sq_entry_t *item) {
    irqstate_t flags = irqsave();
    sq_rem(item, &s_common.req_list[ep].usb);
    irqrestore(flags);
}

bool usbtun_req_is_usb_empry(uint8_t ep) {
    bool ret;
    irqstate_t flags = irqsave();
    ret = sq_empty(&s_common.req_list[ep].usb);
    irqrestore(flags);
    return ret;
}

sq_entry_t *usbtun_req_dq_usb(uint8_t ep) {
    sq_entry_t *ret = NULL;

    irqstate_t flags = irqsave();
    if (!sq_empty(&s_common.req_list[ep].usb))
        ret = sq_remfirst(&s_common.req_list[ep].usb);
    irqrestore(flags);
    return ret;
}

void usbtun_clean_mem(usbtun_buf_t *buf) {
    if (buf->ptr) {
        if (buf->type == USBTUN_MEM_UNIPRO)
            unipro_rxbuf_free(CONFIG_MODS_USBTUN_CPORT_ID, buf->ptr);
        else if (buf->type == USBTUN_MEM_BUFRAM)
            bufram_free(buf->ptr);
    }
    buf->type = USBTUN_MEM_NONE;
    buf->ptr = NULL;
    buf->size = 0;
}

int unipro_send_tunnel_cmd(uint8_t ep, uint8_t type, int16_t code, void *buffer, size_t pkt_len) {
    struct unipro_tx_s *item;
    int ret;

    item = dequeue_tx_item();

    if (item == NULL) {
        lldbg("No more unipro tx item found\n");
        return -ENOMEM;
    }
    item->hdr->magic = USBTUN_HDR_MAGIC;
    item->hdr->ep = ep;
    item->hdr->type = type;
    item->hdr->code = code;
    item->hdr->pkt_len = pkt_len;
#ifdef USBTUN_VDEBUG
    if (USBTUN_DEBUG_EP == ep) {
        static int counter = 1;
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        lldbg("[%u:%ld][%d] ep %d, type %d, buf %p, len %d\n",
              ts.tv_sec, ts.tv_nsec / 1000000, counter++, ep, type, buffer, pkt_len);
    }
#endif
    ret = unipro_send(CONFIG_MODS_USBTUN_CPORT_ID, item->hdr, sizeof(struct usbtun_hdr_s));
    enqueue_tx_item(item);
    if (ret) {
        lldbg("unipro send hdr failure\n");
        return ret;
    }

    if (pkt_len == 0 || buffer == NULL) {
        /* Return here if no data packet is sent.  */
        return 0;
    }

    /* Need to perform flagmentation if supplied pkt_len is larger than CPORT_BUF_SIZE */
    uint32_t offset = 0;
    uint32_t remain = pkt_len;
    size_t len;
    bool last;

    while (offset < pkt_len) {
        if (remain > CPORT_BUF_SIZE) {
            last = false;
            len = CPORT_BUF_SIZE;
        } else {
            /* no more fragment */
            last = true;
            len = remain;
        }

        ret = unipro_send(CONFIG_MODS_USBTUN_CPORT_ID, buffer + offset, len);

        if (ret == 0) {
            if (last)
                break;
        } else {
            lldbg("unipro send body failure (%d) len=%d\n", ret, len);
            break;
        }

        remain -= len;
        offset += len;
    }

    return ret;
}

static void clean_rx_context(struct unipro_rx_context_s *context) {
    context->ep = 0;
    context->type = 0;
    context->len = 0;
    if (context->seg_ptr) {
        bufram_free(context->seg_ptr);
        context->seg_ptr = NULL;
    }
    context->seg_offset = 0;
}

static int unipro_rx_handler(unsigned int cport, void *data, size_t size) {
#ifdef USBTUN_VDEBUG
    lldbg("tun rx handler - data=%p, len = %d\n", data, size);
#endif

    irqstate_t flags = irqsave();
    struct unipro_rx_s *item = (struct unipro_rx_s *)sq_remfirst(&s_common.rx_free);
    irqrestore(flags);

    if (item == NULL) {
        /* Should not happen. Number of item must be corresponding to max
         * infight unipro_rx buffers.
         */
        lldbg("No more unipro_rx item\n");
        return 0;
    }
    item->data = data;
    item->size = size;

    flags = irqsave();
    sq_addlast(&item->entry, &s_common.rx_q);
    irqrestore(flags);

    sem_post(&s_common.rx_sem);
    return 0;
}

static void handle_unipro(struct unipro_rx_context_s *ctx, void *data, size_t size) {
    usbtun_buf_t buf;

    /* check magic, size and expect_body flag */
    if (ctx->ep || ctx->type) {
        /* handle body according to the last received hdr */
        if (size > ctx->len) {
            lldbg("Received data is larger than expected (ep %d)\n", ctx->ep);
        } else if (size < ctx->len) {
            /* segmented body */
            if (!ctx->seg_ptr) {
                ctx->seg_ptr = bufram_alloc(ctx->len);
            }
            if (ctx->seg_ptr) {
                uint32_t copy_size = size;
                if (ctx->seg_offset + size >= ctx->len)
                    copy_size = ctx->len - ctx->seg_offset;

                memcpy(ctx->seg_ptr + ctx->seg_offset, data, copy_size);
                ctx->seg_offset += copy_size;

                if (ctx->seg_offset >= ctx->len) {
                    /* callback passing bufram memory */
                    buf.type = USBTUN_MEM_BUFRAM;
                    buf.ptr = ctx->seg_ptr;
                    buf.size = ctx->len;

                    if (s_common.data_handler(&buf, ctx->ep, ctx->type) == USBTUN_KEEP_BUF) {
                        /* Now bufram will be owned by outside of this routine */
                        ctx->seg_ptr = NULL;
                    }
                    clean_rx_context(ctx);
                }
                /* Done with data (unipro memory). Release it */
                unipro_rxbuf_free(CONFIG_MODS_USBTUN_CPORT_ID, data);

                return;
            } else {
                lldbg("No segmentation buffer available\n");
            }
        } else {
            buf.type = USBTUN_MEM_UNIPRO;
            buf.ptr = data;
            buf.size = ctx->len;

            if (s_common.data_handler(&buf, ctx->ep, ctx->type) == USBTUN_FREE_BUF)
                unipro_rxbuf_free(CONFIG_MODS_USBTUN_CPORT_ID, data);

            clean_rx_context(ctx);
            return;
        }
    }

    clean_rx_context(ctx);

    struct usbtun_hdr_s *hdr;

    if (size < sizeof(*hdr)) {
        lldbg("invalid data received from unipro\n");
        goto freebufram;
    }
    hdr = (struct usbtun_hdr_s *)data;

    if (hdr->magic != USBTUN_HDR_MAGIC) {
        lldbg("unknown data received from unipro\n");
        goto freebufram;
    }

    if (s_common.hdr_handler(hdr->ep, hdr->type, hdr->code, hdr->pkt_len) == USBTUN_WAIT_DATA) {
        ctx->ep = hdr->ep;
        ctx->type = hdr->type;
        ctx->len = hdr->pkt_len;
    }

freebufram:
    unipro_rxbuf_free(CONFIG_MODS_USBTUN_CPORT_ID, data);

    return;
}

static void* unipro_rx_thread(void *arg) {
    int ret;
    struct unipro_rx_context_s ctx;

    lldbg("Unipro RX thread started\n");

    memset(&ctx, 0, sizeof(ctx));

    if (unipro_driver_register(&route_driver, CONFIG_MODS_USBTUN_CPORT_ID)) {
        lldbg("Failed to registered to cport %d\n", CONFIG_MODS_USBTUN_CPORT_ID);
        return NULL;
    }

    unipro_p2p_reset_connection(CONFIG_MODS_USBTUN_CPORT_ID);
#ifdef CONFIG_MODS_USB_PCD_ROUTER
    unipro_p2p_setup_connection(CONFIG_MODS_USBTUN_CPORT_ID);
#endif
    lldbg("Registered to unipro\n");

    while (s_common.rx_run) {
        /* Block until a buffer is available */
        ret = sem_wait(&s_common.rx_sem);
        if (ret < 0) {
            lldbg("Error on sem_wait\n");
            break;
        }
        irqstate_t flags = irqsave();
        struct unipro_rx_s *item = (struct unipro_rx_s *)sq_remfirst(&s_common.rx_q);
        irqrestore(flags);

        if (item == NULL)
            continue;

        handle_unipro(&ctx, item->data, item->size);

        item->data = 0;
        item->size = 0;

        flags = irqsave();
        sq_addlast(&item->entry, &s_common.rx_free);
        irqrestore(flags);
    }

    unipro_driver_unregister(CONFIG_MODS_USBTUN_CPORT_ID);

    lldbg("Unipro RX thread stopped\n");

    return NULL;
}

static int start_unipro_rx_thread(void) {
    s_common.rx_run = true;
    return pthread_create(&s_common.rx_thread, NULL, unipro_rx_thread, NULL);
}

static void stop_unipro_rx_thread(void) {
    s_common.rx_run = false;
    sem_post(&s_common.rx_sem);
    pthread_join(s_common.rx_thread, NULL);
}
