/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stddef.h>
#include <stdlib.h>

#include <nuttx/device.h>
#include <nuttx/power/battery_state.h>
#include <nuttx/list.h>

#include "battery_level.h"
#include "battery_temp.h"
#include "battery_voltage.h"

static sem_t sem = SEM_INITIALIZER(1);

struct notify_node_s
{
    batt_callback_t callback;
    void *arg;
    struct list_head list;
};

struct battery_info_s
{
    struct batt_state_s batt;
    struct list_head notify_list;
};

static struct battery_info_s *g_info;

static void battery_notify(void)
{
    struct notify_node_s *node;
    struct list_head *iter;

    list_foreach(&g_info->notify_list, iter) {
        node = list_entry(iter, struct notify_node_s, list);
        node->callback(node->arg, &g_info->batt);
    }
}

int battery_state_set_level(enum batt_level_e level)
{
    while (sem_wait(&sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    if (g_info->batt.level == level)
        goto done;

    g_info->batt.level = level;
    battery_notify();

done:
    sem_post(&sem);
    return 0;
}

int battery_state_set_temp(enum batt_temp_e temp)
{
    while (sem_wait(&sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    if (g_info->batt.temp == temp)
        goto done;

    g_info->batt.temp = temp;
    battery_notify();

done:
    sem_post(&sem);
    return 0;
}

int battery_state_set_voltage(enum batt_voltage_e voltage)
{
    while (sem_wait(&sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    if (g_info->batt.voltage == voltage)
        goto done;

    g_info->batt.voltage = voltage;

    battery_notify();

done:
    sem_post(&sem);
    return 0;
}

int battery_state_register(batt_callback_t callback, void *arg)
{
    struct notify_node_s *node;
    int retval = 0;

    while (sem_wait(&sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    if (!g_info) {
        retval = -EAGAIN;
        goto register_done;
    }

    node = zalloc(sizeof(*node));
    if (!node) {
        retval = -ENOMEM;
        goto register_done;
    }

    node->callback = callback;
    node->arg = arg;
    list_add(&g_info->notify_list, &node->list);
    callback(arg, &g_info->batt);

register_done:
    sem_post(&sem);
    return retval;
}

int battery_state_init(void)
{
    int ret = 0;

    g_info = zalloc(sizeof(*g_info));
    if (!g_info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    list_init(&g_info->notify_list);

    ret = battery_level_start();
    if (ret) {
        free(g_info);
        g_info = NULL;
        return ret;
    }

    ret = battery_temp_start();
    if (ret) {
        battery_level_stop();
        free(g_info);
        g_info = NULL;
        return ret;
    }

    ret = battery_voltage_start();
    if (ret) {
        battery_level_stop();
        battery_temp_stop();
        free(g_info);
        g_info = NULL;
        return ret;
    }

    return 0;
}
