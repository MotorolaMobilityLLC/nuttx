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

#define DEBUG

#include <errno.h>
#include <debug.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/i2c.h>
#include <arch/board/mods.h>
#include <nuttx/time.h>
#include <nuttx/device.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/mhb/mhb_csi_camera.h>
#include <nuttx/wqueue.h>
#include <nuttx/camera/v4l2_camera_ext_ctrls.h>
#include <nuttx/camera/camera_ext.h>

#include "mhb_csi_camera_sm.h"

/*
 * This is the reference greybus camera extension driver
 * for MHB (Motorola Mods Hi-speed Bus Archirecture)
 *
 */

#define MAX_COMMAND_ITEMS 6
#define OFF_STATE_DELAY   MSEC2TICK(CONFIG_MHB_CAMERA_OFF_DELAY_MS)

struct command_item {
    struct list_head node;
    mhb_camera_command_func func;
};

static struct command_item s_commands[MAX_COMMAND_ITEMS];
static struct list_head s_free_list;
static struct list_head s_active_list;
static sem_t s_command_sem;
static pthread_mutex_t s_command_mutex;
static pthread_mutex_t s_sm_mutex;
static struct work_s mhb_sm_work;
static mhb_camera_sm_state_t s_state = MHB_CAMERA_STATE_INVALID;

static struct command_item *get_item(struct list_head *list) {
    struct command_item* item = NULL;

    if (!list_is_empty(list)) {
        item = list_entry(list->next,
                          struct command_item, node);
        list_del(&item->node);
    }

    return item;
}

static void put_item(struct command_item *item, struct list_head *list) {
    struct list_head *iter;

    /* duplicate check */
    list_foreach(list, iter) {
        if (item == list_entry(iter, struct command_item, node)) {
            return;
        }
    }
    list_add(list, &item->node);
}

static void state_worker(void *data)
{
    sem_post(&s_command_sem);
}

static void *mhb_camera_sm_command_thread(void *arg)
{
    struct command_item *item;
    mhb_camera_command_func func;

    while (1) {
        sem_wait(&s_command_sem);
        pthread_mutex_lock(&s_command_mutex);
        item = get_item(&s_active_list);

        if (item != NULL) {
            func = item->func;
            item->func = NULL;
            put_item(item, &s_free_list);
        } else {
            func = NULL;
        }
        pthread_mutex_unlock(&s_command_mutex);

        if (func) {
            mhb_camera_sm_execute(func());
        }
    }

    return NULL;
}

static int mhb_camera_sm_run_command(mhb_camera_command_func func, uint32_t delay)
{
    pthread_mutex_lock(&s_command_mutex);
    struct command_item *item = get_item(&s_free_list);
    if (item == NULL) {
        CAM_ERR("ERROR: Command queue full. Discard\n");
        pthread_mutex_unlock(&s_command_mutex);
        return -EBUSY;
    }

    item->func = func;
    put_item(item, &s_active_list);

    if (!delay)
        sem_post(&s_command_sem);
    else
        work_queue(HPWORK, &mhb_sm_work, state_worker, NULL, delay);
    pthread_mutex_unlock(&s_command_mutex);

    return 0;
}

/*******************************************************************************
** State Machine Functions
*******************************************************************************/

typedef struct {
    mhb_camera_sm_state_t (*enter)(mhb_camera_sm_event_t event);
    mhb_camera_sm_state_t (*process_event)(mhb_camera_sm_event_t event);
}
mhb_camera_sm_t;


mhb_camera_sm_event_t mhb_camera_sm_waiting(void)
{
    return MHB_CAMERA_EV_WAIT_OVER;
}

static mhb_camera_sm_state_t  mhb_camera_sm_off_enter(mhb_camera_sm_event_t event)
{
    struct command_item* item = NULL;
    mhb_camera_sm_state_t next_state = s_state;
    int dump = 0;

    pthread_mutex_lock(&s_command_mutex);
    while ((item = get_item(&s_active_list)) != NULL) {
        put_item(item, &s_free_list);
        ++dump;
    }
    pthread_mutex_unlock(&s_command_mutex);

    if (dump) CAM_ERR("Dumped %d event\\s\n", dump);
    if (mhb_camera_sm_run_command(mhb_camera_power_off, 0)) {
        CAM_ERR("ERROR: Very Strange! Off Enter Failed. Move to OFF anyway %d\n", s_state);
        // TODO: Handle Failure.
    }

    return next_state;
}

static mhb_camera_sm_state_t  mhb_camera_sm_wait_poweron_enter(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = s_state;

    if (mhb_camera_sm_run_command(mhb_camera_power_on, 0)) {
        CAM_ERR("ERROR: Wait ON Enter Failed. Stay OFF %d \n", s_state);
        // TODO: Handle & V4L2 Notify Async Error
    }
    return next_state;
}
static mhb_camera_sm_state_t  mhb_camera_sm_wait_stream_enter(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = s_state;

    if (mhb_camera_sm_run_command(mhb_camera_stream_on, 0)) {
        CAM_ERR("ERROR: Wait Stream Enter Failed. Stay OFF %d \n", s_state);
        // TODO: Handle & V4L2 Notify Async Error
    }
    return next_state;
}

static mhb_camera_sm_state_t  mhb_camera_sm_wait_stream_off_enter(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = s_state;

    if (mhb_camera_sm_run_command(mhb_camera_stream_off, 0)) {
        CAM_ERR("ERROR: Wait Stream OFF Enter Failed. Stay OFF %d \n", s_state);
        // TODO: Handle & V4L2 Notify Async Error
    }
    return next_state;
}

/* Special state to have lingering wait to avoid cycle time between ON-OFF-ON */
static mhb_camera_sm_state_t  mhb_camera_sm_wait_off_enter(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_run_command(mhb_camera_lens_retract, 0);
    mhb_camera_sm_run_command(mhb_camera_sm_waiting, OFF_STATE_DELAY);
    return s_state;
}

static mhb_camera_sm_state_t mhb_camera_sm_off_process_ev(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = MHB_CAMERA_STATE_INVALID;
    switch (event) {
        case MHB_CAMERA_EV_POWER_ON_REQ:
            pthread_mutex_lock(&s_command_mutex);
            if (list_is_empty(&s_active_list)) {
                next_state = MHB_CAMERA_STATE_WAIT_POWER_ON;
            } else {
                CAM_ERR("Queued events pending. Stay OFF\n");
                next_state = MHB_CAMERA_STATE_OFF;
            }
            pthread_mutex_unlock(&s_command_mutex);
            break;

        case MHB_CAMERA_EV_POWER_OFF_REQ:
            next_state = s_state;
            // Re-execute Power_Off sequence anyway.
            mhb_camera_sm_run_command(mhb_camera_power_off, 0);
            break;

        case MHB_CAMERA_EV_STREAM_OFF_REQ:
        case MHB_CAMERA_EV_DECONFIGURED:
        case MHB_CAMERA_EV_NONE:
            next_state = s_state;
            break;
        default:
            CAM_ERR("Unexpected Event %d in State %d\n", event, s_state);
            break;
    }
    return next_state;
}

static mhb_camera_sm_state_t mhb_camera_sm_wait_poweron_process_ev(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = MHB_CAMERA_STATE_INVALID;
    switch (event) {
        case MHB_CAMERA_EV_FAIL:
            next_state = MHB_CAMERA_STATE_OFF;
            break;
        case MHB_CAMERA_EV_POWERED_ON:
            mhb_camera_process_ctrl_cache(FALSE);
            next_state = MHB_CAMERA_STATE_ON;
            break;
        case MHB_CAMERA_EV_STREAM_ON_REQ:
            next_state = MHB_CAMERA_STATE_WAIT_STREAM;
            break;
        case MHB_CAMERA_EV_POWER_OFF_REQ:
            next_state = MHB_CAMERA_STATE_WAIT_OFF;
            break;
        case MHB_CAMERA_EV_CONFIGURED:
        case MHB_CAMERA_EV_DECONFIGURED:
        case MHB_CAMERA_EV_NONE:
            next_state = s_state;
            break;
        default:
            CAM_ERR("Unexpected Event %d in State %d\n", event, s_state);
            break;
    }
   return next_state;
}

static mhb_camera_sm_state_t mhb_camera_sm_on_process_ev(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = MHB_CAMERA_STATE_INVALID;
    switch (event) {
        case MHB_CAMERA_EV_STREAM_ON_REQ:
            next_state = MHB_CAMERA_STATE_WAIT_STREAM;
            break;
        case MHB_CAMERA_EV_POWER_OFF_REQ:
            next_state = MHB_CAMERA_STATE_WAIT_OFF;
            break;
        case MHB_CAMERA_EV_POWERED_ON:
        case MHB_CAMERA_EV_NONE:
            next_state = s_state;
            break;
        default:
            CAM_ERR("Unexpected Event %d in State %d\n", event, s_state);
            break;
    }
    return next_state;
}

static mhb_camera_sm_state_t mhb_camera_sm_wait_stream_process_ev(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = MHB_CAMERA_STATE_INVALID;
    switch (event) {
        case MHB_CAMERA_EV_CONFIGURED:
            next_state = s_state;
            pthread_mutex_lock(&s_command_mutex);
            if (list_is_empty(&s_active_list)) {
                mhb_camera_process_ctrl_cache(FALSE);
                next_state = MHB_CAMERA_STATE_STREAMING;
            }
            else {
                CAM_ERR("Queued events pending retain state %d\n", event);
            }
            pthread_mutex_unlock(&s_command_mutex);
            break;
        case MHB_CAMERA_EV_STREAM_OFF_REQ:
            next_state = MHB_CAMERA_STATE_WAIT_STREAM_CLOSE;
            break;
        case MHB_CAMERA_EV_FAIL:
            //TODO: Greybus ERROR
            CAM_ERR("FAILED s_state %d event %d. Powering Off\n", s_state, event);
        case MHB_CAMERA_EV_POWER_OFF_REQ:
            next_state = MHB_CAMERA_STATE_OFF;
            break;
        case MHB_CAMERA_EV_DECONFIGURED:
        case MHB_CAMERA_EV_POWERED_ON:
        case MHB_CAMERA_EV_NONE:
            next_state = s_state;
            break;
        default:
            CAM_ERR("Unexpected Event %d in State %d\n", event, s_state);
            break;
    }
    return next_state;
}

static mhb_camera_sm_state_t mhb_camera_sm_stream_process_ev(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = MHB_CAMERA_STATE_INVALID;
    switch (event) {
        case MHB_CAMERA_EV_STREAM_OFF_REQ:
            next_state = MHB_CAMERA_STATE_WAIT_STREAM_CLOSE;
            break;
        case MHB_CAMERA_EV_POWER_OFF_REQ:
            next_state = MHB_CAMERA_STATE_OFF;
            break;
        default:
            CAM_ERR("Unexpected Event %d in State %d\n", event, s_state);
            break;
    }
    return next_state;
}

static mhb_camera_sm_state_t mhb_camera_sm_wait_stream_close_process_ev(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = MHB_CAMERA_STATE_INVALID;
    switch (event) {
        case MHB_CAMERA_EV_DECONFIGURED:
            next_state = MHB_CAMERA_STATE_ON;
            break;
        case MHB_CAMERA_EV_STREAM_ON_REQ:
            next_state = MHB_CAMERA_STATE_WAIT_STREAM;
            break;
        case MHB_CAMERA_EV_FAIL:
            //TODO: Greybus ERROR
            CAM_ERR("FAILED s_state %d event %d. Powering Off\n", s_state, event);
        case MHB_CAMERA_EV_POWER_OFF_REQ:
            next_state = MHB_CAMERA_STATE_WAIT_OFF;
            break;
        case MHB_CAMERA_EV_POWERED_ON:
        case MHB_CAMERA_EV_CONFIGURED:
        case MHB_CAMERA_EV_NONE:
            next_state = s_state;
            break;
        default:
            CAM_ERR("Unexpected Event %d in State %d\n", event, s_state);
            break;
    }
    return next_state;
}

static mhb_camera_sm_state_t mhb_camera_sm_wait_off_process_ev(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_state_t next_state = MHB_CAMERA_STATE_INVALID;
    struct command_item* item = NULL;
    int dump = 0;
    switch (event) {
        case MHB_CAMERA_EV_POWER_ON_REQ:
            pthread_mutex_lock(&s_command_mutex);
            if (work_cancel(HPWORK, &mhb_sm_work)) {
                CAM_ERR("ERROR: Cancel off wait FAILED");
            }

            while ((item = get_item(&s_active_list)) != NULL) {
                put_item(item, &s_free_list);
                ++dump;
            }
            pthread_mutex_unlock(&s_command_mutex);
            if (dump) CAM_ERR("Dumped %d event\\s\n", dump);
            next_state = MHB_CAMERA_STATE_WAIT_POWER_ON;
            break;

        case MHB_CAMERA_EV_FAIL:
        case MHB_CAMERA_EV_POWER_OFF_REQ:
            pthread_mutex_lock(&s_command_mutex);
            if (work_cancel(HPWORK, &mhb_sm_work)) {
                CAM_ERR("ERROR: Cancel off wait FAILED");
            }
            pthread_mutex_unlock(&s_command_mutex);

        case MHB_CAMERA_EV_WAIT_OVER:
            next_state = MHB_CAMERA_STATE_OFF;
            break;

        case MHB_CAMERA_EV_CONFIGURED:
        case MHB_CAMERA_EV_POWERED_ON:
        case MHB_CAMERA_EV_DECONFIGURED:
        case MHB_CAMERA_EV_NONE:
            next_state = s_state;
            break;

        default:
            CAM_ERR("Unexpected Event %d in State %d\n", event, s_state);
            break;
    }
    return next_state;
}


/* s_state table */
static mhb_camera_sm_t mhb_camera_sm_table[] =
{
    {mhb_camera_sm_off_enter,             mhb_camera_sm_off_process_ev},
    {mhb_camera_sm_wait_poweron_enter,    mhb_camera_sm_wait_poweron_process_ev},
    {NULL,                                mhb_camera_sm_on_process_ev},
    {mhb_camera_sm_wait_stream_enter,     mhb_camera_sm_wait_stream_process_ev},
    {NULL,                                mhb_camera_sm_stream_process_ev},
    {mhb_camera_sm_wait_stream_off_enter, mhb_camera_sm_wait_stream_close_process_ev},
    {mhb_camera_sm_wait_off_enter,        mhb_camera_sm_wait_off_process_ev},
};

mhb_camera_sm_state_t mhb_camera_sm_get_state(void)
{
    mhb_camera_sm_state_t state;

    pthread_mutex_lock(&s_sm_mutex);
    state = s_state;
    pthread_mutex_unlock(&s_sm_mutex);
    return state;
}

int mhb_camera_sm_init(void)
{
    int i;
    pthread_t thread;

    if (s_state == MHB_CAMERA_STATE_INVALID) {
        sem_init(&s_command_sem, 0, 0);
        list_init(&s_free_list);
        list_init(&s_active_list);

        for (i = 0; i < MAX_COMMAND_ITEMS; i++) {
            s_commands[i].func = NULL;
            put_item(&s_commands[i], &s_free_list);
        }

        if (pthread_create(&thread, NULL, &mhb_camera_sm_command_thread, NULL) != 0) {
            CAM_ERR("Failed to start control command thread\n");
            return -1;
        }
        pthread_mutex_init(&s_sm_mutex, NULL);
        pthread_mutex_init(&s_command_mutex, NULL);
        s_state = MHB_CAMERA_STATE_OFF;
    }
    return 0;
}

int mhb_camera_sm_execute(mhb_camera_sm_event_t event)
{
    mhb_camera_sm_t *state_table;
    mhb_camera_sm_state_t next_state = MHB_CAMERA_STATE_INVALID;

    pthread_mutex_lock(&s_sm_mutex);

    CTRL_DBG("ev %s s_state %s\n", mhb_camera_sm_event_str(event),
            mhb_camera_sm_state_str(s_state));

    if (s_state != MHB_CAMERA_STATE_INVALID) {
        state_table = &mhb_camera_sm_table[s_state];
        /* process event */
        next_state = state_table->process_event(event);
    }

    if (s_state == MHB_CAMERA_STATE_INVALID ||
        next_state == MHB_CAMERA_STATE_INVALID) {
        pthread_mutex_unlock(&s_sm_mutex);
        CAM_ERR("ERROR: Invalid State/Event ev %d s_state %d\n"
                , event, s_state);
        return -EINVAL;
    }

    /* transition s_state */
    while (next_state != s_state) {
        s_state = next_state;
        CAM_DBG("%s\n", mhb_camera_sm_state_str(s_state));

        state_table = &mhb_camera_sm_table[next_state];
        if (state_table->enter) {
            next_state = state_table->enter(event);
            if (next_state == MHB_CAMERA_STATE_INVALID) {
                pthread_mutex_unlock(&s_sm_mutex);
                CAM_ERR("ERROR: Queue full. Cannot process ev %d s_state %d\n");
                return -EBUSY;
            }
         }
    }

    pthread_mutex_unlock(&s_sm_mutex);
    return 0;
}

