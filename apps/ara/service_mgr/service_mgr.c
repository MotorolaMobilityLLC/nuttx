/*
 * Copyright (c) 2014, 2015 Google Inc.
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
 *
 * Author: Bartosz Golaszewski <bgolaszewski@baylibre.com>
 */

#include <nuttx/list.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include <apps/ara/service_mgr.h>

enum {
    STATE_STARTING = 0,
    STATE_RUNNING,
};

static const char *state_str[] = {
    "STARTING\t",
    "RUNNING\t",
};

struct service_context {
    struct list_head list;
    pthread_t id;
    const char *name;
    int state;
    pid_t pid;
    pthread_mutex_t lock;
};

/*
 * Static initialization makes srvctl() safe without prior call to
 * srvmgr_start().
 */
static struct list_head service_list = LIST_INIT(service_list);
static pthread_mutex_t service_list_mutex = PTHREAD_MUTEX_INITIALIZER;

static void service_list_lock(void)
{
    pthread_mutex_lock(&service_list_mutex);
}

static void service_list_unlock(void)
{
    pthread_mutex_unlock(&service_list_mutex);
}

static void srvctx_lock(struct service_context *ctx)
{
    pthread_mutex_lock(&ctx->lock);
}

static void srvctx_unlock(struct service_context *ctx)
{
    pthread_mutex_unlock(&ctx->lock);
}

struct service_wrapper_data {
    struct service_context *ctx;
    srvmgr_service_func func;
};

static void *service_wrapper(void *data)
{
    struct service_wrapper_data *srv_data = data;
    struct service_context *ctx = srv_data->ctx;
    int status;

    srvctx_lock(ctx);
    ctx->pid = getpid();
    ctx->state = STATE_RUNNING;
    srvctx_unlock(ctx);

    status = srv_data->func();

    srvctx_lock(ctx);
    ctx->pid = -1;
    if (status != 0)
        fprintf(stderr, "%s exited with error %d\n", ctx->name, status);
    srvctx_unlock(ctx);

    /*
     * The thread removes itself from the list - pthreads are detached by
     * default in nuttx so no need to call pthread_join().
     */
    service_list_lock();
    list_del(&ctx->list);
    service_list_unlock();
    /*
     * We removed ourselves from the list and the only place where ctx can
     * potentially be locked outside the list lock is this function. It's
     * now safe to free ctx.
     */
    pthread_mutex_destroy(&ctx->lock);
    free(ctx);
    free(srv_data);

    return NULL;
}

/*
 * Kills and frees all already created pthreads.
 */
static void destroy_all_services(void)
{
    struct service_context *ctx;
    struct list_head *iter;

    fprintf(stderr, "destroying all previously created threads\n");

    service_list_lock();
    list_foreach(&service_list, iter) {
        ctx = list_entry(iter, struct service_context, list);

        pthread_kill(ctx->id, 9);
        pthread_mutex_destroy(&ctx->lock);
        list_del(iter);
        free(ctx);
    }
    service_list_unlock();
}

/**
 * @brief Start a group of services
 * @param services Array of struct srvmgr_service describing the services
 * @return 0 on success, -1 on out-of-memory
 *
 * Start a group of services. Each must be described by a single
 * struct srvmgr_service in the services array. The function expects
 * an empty sentinel at the end.
 *
 * Example usage:
 * @code
 * static int dummy_service(void)
 * {
 *     (do something...)
 *
 *     return 0;
 * }
 *
 * static struct srvmgr_service srvs[] = {
 *     {
 *         .name = "dummy",
 *         .func = dummy_service,
 *     },
 *     { NULL, NULL }
 * };
 *
 * int main()
 * {
 *     srvmgr_start(srvs);
 *     (...)
 * }
 * @endcode
 */
int srvmgr_start(struct srvmgr_service *services)
{
    struct service_wrapper_data *wr_data;
    struct service_context *srv_ctx;
    struct srvmgr_service *srv;
    int status, retval = 0;

    if (services == NULL)
        return 0;

    for (srv = services; srv->name; srv++) {
        srv_ctx = malloc(sizeof(struct service_context));
        if (srv_ctx == NULL)
            goto oom;

        wr_data = malloc(sizeof(struct service_wrapper_data));
        if (wr_data == NULL) {
            free(srv_ctx);
            goto oom;
        }

        memset(srv_ctx, 0, sizeof(struct service_context));
        srv_ctx->name = srv->name;
        pthread_mutex_init(&srv_ctx->lock, NULL);

        wr_data->ctx = srv_ctx;
        wr_data->func = srv->func;

        status = pthread_create(&srv_ctx->id, NULL, service_wrapper, wr_data);
        if (status != 0) {
            fprintf(stderr,
                    "pthread_create error when starting service %s: %d\n",
                    srv->name, status);
            pthread_mutex_destroy(&srv_ctx->lock);
            free(srv_ctx);
            free(wr_data);
            destroy_all_services();
            set_errno(status);
            retval = -1;
            break;
        }

        service_list_lock();
        list_add(&service_list, &srv_ctx->list);
        service_list_unlock();
    }

    return retval;

oom:
    fprintf(stderr, "%s: out-of-memory\n", __FUNCTION__);
    return -1;
}

/*
 * Simple application for displaying the state of registered services. For
 * now it doesn't allow any kind of interaction with them, just displays
 * some basic info.
 */
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int srvctl_main(int argc, char *argv[])
#endif
{
    struct service_context *ctx;
    struct list_head *iter;

    service_list_lock();

    printf("     PID      STATUS       NAME\n"
           "-----------------------------------------------\n");

    list_foreach(&service_list, iter) {
        ctx = list_entry(iter, struct service_context, list);

        srvctx_lock(ctx);
        printf("%8d %12s   %s\n",
               ctx->pid, state_str[ctx->state], ctx->name);
        srvctx_unlock(ctx);
    }

    service_list_unlock();

    return 0;
}
