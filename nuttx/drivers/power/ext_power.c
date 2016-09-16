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
#include <stdlib.h>
#include <string.h>

#include <nuttx/device.h>
#include <nuttx/device_ext_power.h>
#include <nuttx/power/ext_power.h>
#include <nuttx/list.h>

static sem_t sem = SEM_INITIALIZER(1);
static LIST_DECLARE(notify_list);
static struct device *dev[EXT_POWER_NUMBER_OF_SOURCES];

struct notify_node_s
{
    ext_power_notification callback;
    void *arg;
    struct list_head list;
};

static void ext_power_state_changed(void *arg)
{
    struct notify_node_s *node;
    struct list_head *iter;

    while (sem_wait(&sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    list_foreach(&notify_list, iter) {
        node = list_entry(iter, struct notify_node_s, list);
        node->callback(node->arg, dev);
    }

    sem_post(&sem);
}

static int ext_power_init(void)
{
    static sem_t init_sem = SEM_INITIALIZER(1);
    static bool initialized = false;
    int retval = 0;
    int i, j;

    while (sem_wait(&init_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    if (initialized)
        goto init_done;

    for (i = 0; i < EXT_POWER_NUMBER_OF_SOURCES; i++) {
        dev[i] = device_open(DEVICE_TYPE_EXT_POWER_HW, i);
        if (dev[i]) {
            retval = device_ext_power_register_callback(dev[i], ext_power_state_changed, NULL);
            if (retval) {
                dbg("failed to register callback for ext power source %d\n", i);
                for (j = 0; j <= i; j++) {
                    device_close(dev[i]);
                    dev[i] = NULL;
                }
                goto init_done;
            }
        } else
            dbg("external power source %d is not supported\n", i);
    }

    initialized = true;

init_done:
    sem_post(&init_sem);
    return retval;
}

int ext_power_register_callback(ext_power_notification callback, void *arg)
{
    struct notify_node_s *node;
    int retval = 0;

    while (sem_wait(&sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    retval = ext_power_init();
    if (retval)
        goto register_done;

    node = zalloc(sizeof(*node));
    if (!node) {
        retval = -ENOMEM;
        dbg("failed to allocate memory for node\n");
        goto register_done;
    }

    node->callback = callback;
    node->arg = arg;
    list_add(&notify_list, &node->list);
    callback(arg, dev);

register_done:
    sem_post(&sem);
    return retval;
}

int ext_power_set_max_output_voltage(ext_power_source_e source, int voltage)
{
    int retval = ext_power_init();

    if (retval)
        return retval;

    if (dev[source])
        return device_ext_power_set_max_output_voltage(dev[source], voltage);
    else
        return 0;
}
