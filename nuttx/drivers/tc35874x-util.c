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

#include <errno.h>
#include <debug.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/i2c.h>
#include <nuttx/list.h>

#include <arch/byteorder.h>

#include "tc35874x-util.h"
#include "camera_ext_dbg.h"

/*
 * This is the reference greybus driver for Toshiba Parallel to CSI-2 bridge
 * chip (TC35874X series).
 *
 * This file contains I2C communication utilities used by a driver talking
 * to TC35874X series chip.
 */

/*
 * Utilities to access I2C data in TC35874X specific way
 */
static int i2c_read(struct tc35874x_i2c_dev_info *i2c,
                    uint8_t *addr, int addr_len,
                    uint8_t *data, int data_len)
{
    struct i2c_msg_s msg[2];

    msg[0].addr   = i2c->i2c_addr;
    msg[0].flags  = 0;
    msg[0].buffer = addr;
    msg[0].length = addr_len;

    msg[1].addr   = i2c->i2c_addr;
    msg[1].flags  = I2C_M_READ;
    msg[1].buffer = data;
    msg[1].length = data_len;

    int ret = I2C_TRANSFER(i2c->i2c, msg, 2);
    if (ret != 0) {
        CAM_ERR("i2c read transfer failed %d\n", ret);
        return -1;
    }

    return 0;
}

/* 2 bytes value register read */
uint16_t tc35874x_read_reg2(struct tc35874x_i2c_dev_info* i2c, uint16_t regaddr)
{
    uint8_t addr[2];
    uint8_t data[2];
    uint16_t value = 0;

    addr[0] = (regaddr >> 8) & 0xFF;
    addr[1] = regaddr & 0xFF;

    memset(data, 0, sizeof(data));

    if (i2c_read(i2c, addr, sizeof(addr), data, sizeof(data)) == 0) {
        value = (data[0] << 8) + data[1];
        CAM_DBG("Read: 0x%04x -> 0x%04x\n", regaddr, value);
    }

    return value;
}

/* 4 bytes value register read */
uint32_t tc35874x_read_reg4(struct tc35874x_i2c_dev_info* i2c, uint16_t regaddr)
{
    uint8_t addr[2];
    uint8_t data[4];
    uint32_t value = 0;

    addr[0] = (regaddr >> 8) & 0xFF;
    addr[1] = regaddr & 0xFF;

    memset(data, 0, sizeof(data));

    if (i2c_read(i2c, addr, sizeof(addr), data, sizeof(data)) == 0) {
        value = (data[0] << 8) + data[1] + (data[2] << 24) + (data[3] << 16);
        CAM_DBG("Read: 0x%04x -> 0x%08x\n", regaddr, value);
    }

    return value;
}

static int i2c_write(struct tc35874x_i2c_dev_info *i2c, uint8_t *addr, int addr_len)
{
    struct i2c_msg_s msg;

    msg.addr   = i2c->i2c_addr;
    msg.flags  = 0;
    msg.buffer = addr;
    msg.length = addr_len;

    int ret = I2C_TRANSFER(i2c->i2c, &msg, 1);
    if (ret != 0) {
        CAM_ERR("i2c write transfer failed %d\n", ret);
        return -1;
    }

    return 0;
}

/* 2 bytes value register write */
int tc35874x_write_reg2(struct tc35874x_i2c_dev_info* i2c, uint16_t regaddr, uint16_t data)
{
    uint8_t addr[4];

    CAM_DBG("write 0x%04x to addr 0x%04x\n", data, regaddr);
    addr[0] = (regaddr >> 8) & 0xFF;
    addr[1] = regaddr & 0xFF;
    addr[2] = (data >> 8) & 0xFF;
    addr[3] = data & 0xFF;

    return i2c_write(i2c, addr, sizeof(addr));
}

/* 4 bytes value register write */
int tc35874x_write_reg4(struct tc35874x_i2c_dev_info* i2c, uint16_t regaddr, uint32_t data)
{
    uint8_t addr[6];

    printf("write 0x%08x to addr 0x%08x\n", data, regaddr);
    addr[0] = (regaddr >> 8) & 0xFF;
    addr[1] = regaddr & 0xFF;
    addr[2] = (data >> 8) & 0xFF;
    addr[3] = data & 0xFF;
    addr[4] = (data >> 24) & 0xFF;
    addr[5] = (data >> 16) & 0xFF;

    return i2c_write(i2c, addr, sizeof(addr));
}

/*
 * Utilities to configure bridge I2C register asynchronously
 */

struct command_item {
    struct list_head node;
    tc35874x_command_func func;
    void* data;
};

#define MAX_COMMAND_ITEMS 8

static struct command_item s_commands[MAX_COMMAND_ITEMS];

static struct list_head s_free_list;
static struct list_head s_active_list;
static sem_t s_command_sem;

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

static void *run_command_thread(void *arg)
{
    struct tc35874x_i2c_dev_info *i2c_info = (struct tc35874x_i2c_dev_info *)arg;
    struct command_item *item;
    tc35874x_command_func func;
    void *data;

    while (1) {
        sem_wait(&s_command_sem);

        irqstate_t flags = irqsave();
        item = get_item(&s_active_list);

        if (item != NULL) {
            func = item->func;
            data = item->data;
        } else
            func = NULL;

        item->func = NULL;
        item->data = NULL;
        put_item(item, &s_free_list);
        irqrestore(flags);

        if (func) {
            if (func(i2c_info, data) != 0) {
                CAM_ERR("Command execution error\n");
                /* TODO: send async errnor notification */
            }
        }
    }

    return NULL;
}

int tc35874x_start_i2c_control_thread(struct tc35874x_i2c_dev_info *i2c_info)
{
    int i;
    pthread_t thread;

    sem_init(&s_command_sem, 0, 0);
    list_init(&s_free_list);
    list_init(&s_active_list);

    for (i = 0; i < MAX_COMMAND_ITEMS; i++) {
        s_commands[i].func = NULL;
        s_commands[i].data = NULL;
        put_item(&s_commands[i], &s_free_list);
    }

    if (pthread_create(&thread, NULL, &run_command_thread, i2c_info) != 0) {
        CAM_ERR("Failed to start control command thread\n");
        return -1;
    }

    return 0;
}

int tc35874x_run_command(tc35874x_command_func func, void *data)
{
    irqstate_t flags = irqsave();
    struct command_item *item = get_item(&s_free_list);
    if (item == NULL) {
        irqrestore(flags);
        return -1;
    }

    item->func = func;
    item->data = data;
    put_item(item, &s_active_list);
    irqrestore(flags);

    sem_post(&s_command_sem);

    return 0;
}
