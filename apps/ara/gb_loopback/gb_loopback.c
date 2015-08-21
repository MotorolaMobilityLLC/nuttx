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
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#include <nuttx/greybus/loopback.h>
#include <nuttx/util.h>
#include <nuttx/time.h>

struct gb_loopback_operation {
    const char *name;
    int op;
};

static const struct gb_loopback_operation gb_loopback_ops[] = {
    {
        .name = "ping",
        .op = GB_LOOPBACK_TYPE_PING,
    },
    {
        .name = "xfer",
        .op = GB_LOOPBACK_TYPE_TRANSFER,
    },
    {
        .name = "sink",
        .op = GB_LOOPBACK_TYPE_SINK,
    }
};

static int op_type_from_str(const char *str)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(gb_loopback_ops); i++) {
        if (strcmp(str, gb_loopback_ops[i].name) == 0) {
            return gb_loopback_ops[i].op;
        }
    }

    return -1;
}

struct loopback_context {
    struct list_head list;
    pthread_mutex_t lock;
    int active;
    int cport;
    int type;
    size_t size;
    unsigned wait;
    unsigned left_to_wait;
    int count; /* -1 = infinite */
    int err;
    unsigned sent;
};

static void loopback_ctx_lock(struct loopback_context *ctx)
{
    pthread_mutex_lock(&ctx->lock);
}

static void loopback_ctx_unlock(struct loopback_context *ctx)
{
    pthread_mutex_unlock(&ctx->lock);
}

static struct list_head loopback_ctx_list = LIST_INIT(loopback_ctx_list);
static pthread_mutex_t loopback_ctx_list_mutex = PTHREAD_MUTEX_INITIALIZER;

static void loopback_ctx_list_lock(void)
{
    pthread_mutex_lock(&loopback_ctx_list_mutex);
}

static void loopback_ctx_list_unlock(void)
{
    pthread_mutex_unlock(&loopback_ctx_list_mutex);
}

static pthread_once_t loopback_init_once = PTHREAD_ONCE_INIT;

static int get_cport(int cport, void *data)
{
    struct loopback_context *ctx;

    ctx = zalloc(sizeof(struct loopback_context));
    if (ctx == NULL)
        return -ENOMEM;

    ctx->cport = cport;
    pthread_mutex_init(&ctx->lock, NULL);
    loopback_ctx_list_lock();
    list_add(&loopback_ctx_list, &ctx->list);
    loopback_ctx_list_unlock();

    return 0;
}

static void loopback_init(void)
{
    int status;

    status = gb_loopback_get_cports(get_cport, NULL);
    if (status < 0)
        fprintf(stderr, "gb_loopback initialization failed: %d\n", status);
}

static pthread_cond_t loopback_service_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t loopback_service_cond_lock = PTHREAD_MUTEX_INITIALIZER;

static void loopback_sleep(void)
{
    pthread_mutex_lock(&loopback_service_cond_lock);
    pthread_cond_wait(&loopback_service_cond, &loopback_service_cond_lock);
    pthread_mutex_unlock(&loopback_service_cond_lock);
}

static void loopback_wakeup(void)
{
    pthread_cond_broadcast(&loopback_service_cond);
}

/*
 * The main loop of the service is woken up when there's at least one
 * loopback command running. The user can specify a single command per
 * cport.
 *
 * When not sleeping the service will try to run as fast as possible through
 * all active loopback commands and send all requests, then sleep just enough
 * to wake up in time to service the next request.
 *
 * As soon as there are no more active commands the thread goes back to sleep.
 */
int gb_loopback_service(void)
{
    struct timeval tv_start, tv_end, tv_total;
    useconds_t loop_time, sleep_time;
    struct loopback_context *ctx;
    struct list_head *iter;
    unsigned wait_min;
    int all_down;
    size_t size;
    int status;

    gettimeofday(&tv_start, NULL);

    while (1) {
        all_down = 1;
        wait_min = 0;

        loopback_ctx_list_lock();
        list_foreach(&loopback_ctx_list, iter) {
            ctx = list_entry(iter, struct loopback_context, list);

            loopback_ctx_lock(ctx);

            if (!ctx->active) {
                loopback_ctx_unlock(ctx);
                continue;
            }

            /* At least one loopback command is active in this iteration. */
            all_down = 0;
            size = ctx->type == GB_LOOPBACK_TYPE_PING ? 1 : ctx->size;

            status = gb_loopback_send_req(ctx->cport, size, ctx->type);
            if (status != OK) {
                if (errno == EINVAL) {
                    /* This cport is disconnected. */
                    fprintf(stderr,
                            "%s: cport %d disconnected\n",
                            __FUNCTION__, ctx->cport);
                    ctx->active = 0;
                } else {
                    ctx->err++;
                }
                continue;
            }
            ctx->sent++;

            if (ctx->count > 0)
                ctx->count--;

            if (ctx->count == 0) {
                ctx->active = 0;
            } else {
                if (ctx->left_to_wait == 0)
                    ctx->left_to_wait = ctx->wait;

                /*
                 * Need to determine the shortest wait period of all
                 * loopback commands.
                 */
                if (wait_min == 0) {
                    wait_min = ctx->left_to_wait;
                } else {
                    if (ctx->left_to_wait < wait_min) {
                        wait_min = ctx->left_to_wait;
                    }
                }
            }

            loopback_ctx_unlock(ctx);
        }

        if (all_down) {
            loopback_ctx_list_unlock();
            loopback_sleep();
        } else {
            /*
             * We have to update the wait period of all the loopbacks and
             * also account for the time spent processing the requests.
             */
            list_foreach(&loopback_ctx_list, iter) {
                ctx = list_entry(iter, struct loopback_context, list);
                loopback_ctx_lock(ctx);
                ctx->left_to_wait -= wait_min;
                loopback_ctx_unlock(ctx);
            }
            loopback_ctx_list_unlock();

            gettimeofday(&tv_end, NULL);
            timersub(&tv_end, &tv_start, &tv_total);

            loop_time = timeval_to_usec(&tv_total);
            sleep_time = wait_min;

            /*
             * If no sleep is required, then we will probably go down in the
             * next iteration - don't call usleep().
             */
            if (sleep_time > 0) {
                if (loop_time > sleep_time)
                    fprintf(stderr, "%s running late\n", __FUNCTION__);
                else
                    usleep(sleep_time - loop_time);
            }
        }

        gettimeofday(&tv_start, NULL);
    }

    return 0;
}

static void print_status_normal(void)
{
    struct gb_loopback_statistics stats;
    struct loopback_context *ctx;
    struct list_head *iter;

    printf("  CPORT    ACTIVE    RECV ERR    SEND ERR    SENT    RECV    THROUGHPUT   LATENCY   REQ_PER_SEC\n");
    loopback_ctx_list_lock();
    list_foreach(&loopback_ctx_list, iter) {
        ctx = list_entry(iter, struct loopback_context, list);

        loopback_ctx_lock(ctx);
        gb_loopback_get_stats(ctx->cport, &stats);
        printf("%7d %9s %11d %11d %7u %7u %13u %9u %13u\n",
               ctx->cport,
               ctx->active ? "yes" : "no",
               stats.recv_err,
               ctx->err,
               ctx->sent,
               stats.recv,
               stats.throughput_avg,
               stats.latency_avg,
               stats.reqs_per_sec_avg);
        loopback_ctx_unlock(ctx);
    }
    loopback_ctx_list_unlock();
}

static void print_status_csv(void)
{
    struct gb_loopback_statistics stats;
    struct loopback_context *ctx;
    struct list_head *iter;

    printf("; generated by gbl\n");
    printf("; iterations, errors, requests per second (min, max, avg, jitter), latency (min, max, avg, jitter), throughput (min, max, avg, jitter)\n");
    loopback_ctx_list_lock();
    list_foreach(&loopback_ctx_list, iter) {
        ctx = list_entry(iter, struct loopback_context, list);

        loopback_ctx_lock(ctx);
        gb_loopback_get_stats(ctx->cport, &stats);
        printf("%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               stats.recv,
               stats.recv_err,
               stats.reqs_per_sec_min,
               stats.reqs_per_sec_max,
               stats.reqs_per_sec_avg,
               stats.reqs_per_sec_max - stats.reqs_per_sec_min,
               stats.latency_min,
               stats.latency_max,
               stats.latency_avg,
               stats.latency_max - stats.latency_min,
               stats.throughput_min,
               stats.throughput_max,
               stats.throughput_avg,
               stats.throughput_max - stats.throughput_min);
        loopback_ctx_unlock(ctx);
    }
    loopback_ctx_list_unlock();
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gbl_main(int argc, char *argv[])
#endif
{
    int opt, rv = EXIT_SUCCESS, st, cport = -1, type = 0, count = -1;
    struct loopback_context *ctx;
    const char *cmd, *fmt = NULL;
    struct list_head *iter;
    unsigned wait = 1000;
    size_t size = 1;

    pthread_once(&loopback_init_once, loopback_init);

    optind = -1;
    while ((opt = getopt (argc, argv, "c:s:t:w:n:f:")) != -1) {
        switch (opt) {
        case 'c':
            st = sscanf(optarg, "%d", &cport);
            if (st != 1)
                goto help;
            break;
        case 's':
            st = sscanf(optarg, "%u", &size);
            if (st != 1)
                goto help;
            break;
        case 't':
            type = op_type_from_str(optarg);
            if (type < 0)
                goto help;
            break;
        case 'w':
            st = sscanf(optarg, "%u", &wait);
            if (st != 1)
                goto help;
            break;
        case 'n':
            st = sscanf(optarg, "%d", &count);
            if (st != 1)
                goto help;
            break;
        case 'f':
            fmt = optarg;
            break;
        default:
            goto help;
        }
    }

    /* At least one argument required. */
    if (argc < 2)
        goto help;
    cmd = argv[argc - 1];

    if (wait == 0) {
        fprintf(stderr, "wait period must be greater than 0\n");
        return EXIT_FAILURE;
    }

    /* Bail-out if the cport is invalid since we would do nothing anyway. */
    if (cport >= 0 && !gb_loopback_cport_valid(cport)) {
        fprintf(stderr, "cport %d not registered for gb_loopback\n", cport);
        rv = EXIT_FAILURE;
        goto out;
    }

    if (strcmp(cmd, "start") == 0) {
        if (type == GB_LOOPBACK_TYPE_NONE) {
            fprintf(stderr, "operation type must be specified for 'start'\n");
            rv = EXIT_FAILURE;
            goto out;
        }

        loopback_ctx_list_lock();

        list_foreach(&loopback_ctx_list, iter) {
            ctx = list_entry(iter, struct loopback_context, list);

            if (cport < 0 || cport == ctx->cport) {
                loopback_ctx_lock(ctx);
                gb_loopback_reset(ctx->cport);
                ctx->active = 1;
                ctx->type = type;
                ctx->size = size;
                ctx->wait = msec_to_usec(wait);
                ctx->count = count;
                ctx->sent = 0;
                loopback_ctx_unlock(ctx);
            }
        }

        loopback_ctx_list_unlock();
        loopback_wakeup();
    } else if (strcmp(cmd, "stop") == 0) {
        loopback_ctx_list_lock();

        list_foreach(&loopback_ctx_list, iter) {
            ctx = list_entry(iter, struct loopback_context, list);

            if (cport < 0 || cport == ctx->cport) {
                loopback_ctx_lock(ctx);
                ctx->active = 0;
                loopback_ctx_unlock(ctx);
            }
        }

        loopback_ctx_list_unlock();
        loopback_wakeup();
    } else if (strcmp(cmd, "status") == 0) {
        if (strcmp(fmt, "csv") == 0)
            print_status_csv();
        else
            print_status_normal();
    } else {
        goto help;
    }

out:
    return rv;

help:
    printf(
        "Greybus loopback tool\n\n"
        "Usage:\n"
        "\tgbl [-c CPORT] [-s SIZE] [-t ping|xfer|sink] "
                        "[-w MS] [-n COUNT] [-f csv] start|stop|status\n\n"
        "\tCommands:\n"
        "\t\tstart:\t\tstart a loopback command on a cport\n"
        "\t\tstop:\t\tstop the command on given cport\n"
        "\t\tstatus:\t\tshow current status\n\n"
        "\tOptions:\n"
        "\t\t-c CPORT:\tcport number (all cports if not given)\n"
        "\t\t-s SIZE:\tdata size in bytes\n"
        "\t\t-t TYPE:\tloopback operation type\n"
        "\t\t-w MS:\t\ttime to wait before sending next request (in ms)\n"
        "\t\t-n COUNT:\tnumber of requests to send before stopping\n"
        "\t\t-f FORMAT:\tspecify a different output format\n"
    );

    return EXIT_FAILURE;
}
