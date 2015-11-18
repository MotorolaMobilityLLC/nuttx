/*
 * Copyright (c) 2014-2015 Google Inc.
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

#include <nuttx/config.h>

#include <string.h>
#include <nuttx/util.h>
#include <nuttx/bufram.h>
#include <arch/board/common_gadget.h>

#define REQ_SIZE_MUL (128)

struct request_list {
    struct list_head list;
    struct usbdev_ep_s *ep;
    struct usbdev_req_s *req;
    size_t len;                 /* size of allocated buffer */
    void *priv;
};

struct request_pool {
    size_t len;
    struct list_head request;
    struct list_head request_in_use;
    struct list_head pool;
};
static LIST_DECLARE(request_pool);

static struct request_pool *get_request_pool(size_t len);
static void free_request(struct usbdev_ep_s *ep, struct usbdev_req_s *req);

static struct request_pool *request_pool_alloc(size_t len)
{
    struct request_pool *pool;

    pool = kmm_malloc(sizeof(*pool));
    if (!pool) {
        return NULL;
    }
    pool->len = len;
    list_init(&pool->request);
    list_init(&pool->request_in_use);

    list_add(&request_pool, &pool->pool);

    return pool;
}

static void request_pool_free(struct request_pool *pool)
{
    struct request_list *req_list;
    struct list_head *iter, *next;

    list_foreach_safe(&pool->request, iter, next) {
        req_list = list_entry(iter, struct request_list, list);
        free_request(req_list->ep, req_list->req);
        list_del(&req_list->list);
        kmm_free(req_list);
    }
    list_del(&pool->pool);
    kmm_free(pool);
}

static struct request_pool *get_request_pool(size_t len)
{
    struct list_head *iter;
    struct request_pool *pool;

    list_foreach(&request_pool, iter) {
        pool = list_entry(iter, struct request_pool, pool);
        if (pool->len == len)
            return pool;
    }
    return NULL;
}

static void init_request(struct usbdev_req_s *req,
                         usb_callback callback, size_t len, void *priv)
{
    req->len = len;
    req->priv = priv;
    req->callback = callback;
}


static struct usbdev_req_s *alloc_request(struct usbdev_ep_s *ep,
                                          usb_callback callback,
                                          size_t len, void *priv)
{
    struct usbdev_req_s *req;

    DEBUGASSERT(ep);

    req = EP_ALLOCREQ(ep);
    if (!req) {
        return NULL;
    }

    req->buf = NULL;

    /* Only allocate request buffer for OUT requests */
    if (len && ep->eplog % 2 == 0) {
        req->buf = EP_ALLOCBUFFER(ep, len);
        if (!req->buf) {
            EP_FREEREQ(ep, req);
            return NULL;
        }
    }
    init_request(req, callback, len, priv);

    return req;
}

static void free_request(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
    if (!req)
        return;

    if (req->buf != NULL) {
        if (ep->eplog % 2 == 0) /* free only OUT requests */
            EP_FREEBUFFER(ep, req->buf);
        req->buf = NULL;
        req->len = 0;
    }
    EP_FREEREQ(ep, req);
}

/*
 * Return a multiple of REQ_SIZE_MUL
 */
static size_t request_len_align(size_t len)
{
    if (!len) {
        return 0;
    }
    return ((--len / REQ_SIZE_MUL) + 1) * REQ_SIZE_MUL;
}

/*
 * Assign private data to request
 * \param req request's pointer
 * \param priv pointer to private data to assign
 */
void request_set_priv(struct usbdev_req_s *req, void *priv)
{
    struct request_list *req_list = req->priv;
    req_list->priv = priv;
}

/*
 * Get the private data from request
 * \param req request's pointer
 * \return a pointer to private data
 */
void *request_get_priv(struct usbdev_req_s *req)
{
    struct request_list *req_list = req->priv;
    return req_list->priv;
}

/*
 * \brief Get a request from request manager
 * Allocate a new request a return one from the request pool.
 * \param ep the pointer of endpoint that is going to use the request.
 * \param callback transfer completion callback.
 *     NULL is only accepted to preallocate request.
 * \param len Size of request.
 * \param priv pointer to private data (can be NULL).
 */
struct usbdev_req_s *get_request(struct usbdev_ep_s *ep,
                                 usb_callback callback,
                                 size_t len, void *priv)
{
    int empty;
    irqstate_t flags;
    size_t len_aligned;
    struct list_head *list;
    struct usbdev_req_s *req = NULL;
    struct request_list *req_list;
    struct request_pool *pool;

    len_aligned = request_len_align(len);
    pool = get_request_pool(len_aligned);
    if (!pool) {
        pool = request_pool_alloc(len_aligned);
        if (!pool) {
            return NULL;
        }
    }

    flags = irqsave();
    empty = list_is_empty(&pool->request);
    irqrestore(flags);
    if (empty) {
        /* Assume device driver support ep = NULL */
        req_list = kmm_malloc(sizeof(*req_list));
        if (!req_list) {
            return NULL;
        }
        req = alloc_request(ep, callback, len, req_list);
        req_list->req = req;
        req_list->len = len_aligned;
        if (!req) {
            kmm_free(req_list);
            return NULL;
        }
    } else {
        list = pool->request.next;
        req_list = list_entry(list, struct request_list, list);
        req = req_list->req;
        init_request(req, callback, len, req_list);
        flags = irqsave();
        list_del(&req_list->list);
        irqrestore(flags);
    }
    req_list->priv = priv;
    req_list->ep = ep;

    flags = irqsave();
    list_add(&pool->request_in_use, &req_list->list);
    irqrestore(flags);

    return req;
}

/*
 * Return the request to the request pool.
 * \param req request's pointer
 */
void put_request(struct usbdev_req_s *req)
{
    irqstate_t flags;
    size_t len_aligned;
    struct request_list *req_list;
    struct request_pool *pool;

    if (!req)
        return;

    req_list = req->priv;
    len_aligned = request_len_align(req_list->len);
    pool = get_request_pool(len_aligned);
    DEBUGASSERT(pool);

    flags = irqsave();
    list_del(&req_list->list);
    list_add(&pool->request, &req_list->list);
    irqrestore(flags);
}

/*
 * Look up a request in request pool using a private data
 * \param priv private data pointer to use for look up
 * \return request's pointer or NULL if request can not be found
 */
struct usbdev_req_s *find_request_by_priv(const void *priv)
{
    struct list_head *pool_iter, *req_iter;
    struct request_pool *pool;
    struct request_list *req_list;
    irqstate_t flags;

    list_foreach(&request_pool, pool_iter) {
        pool = list_entry(pool_iter, struct request_pool, pool);
        flags = irqsave();
        list_foreach(&pool->request_in_use, req_iter) {
            req_list = list_entry(req_iter, struct request_list, list);
            if (req_list->priv == priv) {
                irqrestore(flags);
                return req_list->req;
            }
        }
        irqrestore(flags);
    }
    return NULL;
}

/*
 * Get the endpoint that use the request
 * \param req request's pointer
 * \return a pointer to endpoint
 */
struct usbdev_ep_s *request_to_ep(struct usbdev_req_s *req)
{
    struct request_list *req_list = req->priv;

    return req_list->ep;
}

/*
 * Preallocate requests
 * \param ep endpoint that will use the request.
       Note that the requests may be available to other endpoints.
 * \param len the size of request to preallocate.
 * \param n number of request to preallocate
 */
int request_pool_prealloc(struct usbdev_ep_s *ep, size_t len, int n)
{
    int i;
    int ret = 0;
    struct usbdev_req_s *req[n];

    for (i = 0; i < n; i++) {
        req[i] = get_request(ep, NULL, len, NULL);
        if (!req[i]) {
            ret = -ENOMEM;
            break;
        }
    }
    for (i--; i >= 0; i--) {
        put_request(req[i]);
    }
    return ret;
}

/*
 * Free all requests
 */
void request_pool_freeall(void)
{
    struct request_pool *pool;
    struct list_head *iter, *next;

    list_foreach_safe(&request_pool, iter, next) {
        pool = list_entry(iter, struct request_pool, pool);
        request_pool_free(pool);
    }
}

