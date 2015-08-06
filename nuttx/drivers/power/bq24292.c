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

#include <debug.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>

#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/power/bq24292.h>

#include "bq24292_config.h"

static sem_t sem = SEM_INITIALIZER(1);
static sem_t pg  = SEM_INITIALIZER(0);

static FAR struct i2c_dev_s *i2c;

#define BQ24292_I2C_ADDR    0x6B

static void *pg_worker(void *v)
{
    do {
        // Configure registers every time power is attached
        bq24292_configure();
    } while(!sem_wait(&pg));

    return NULL;
}

static int pg_isr(int irq, void *context)
{
    sem_post(&pg);
    return OK;
}

static int reg_read(uint8_t reg)
{
    uint8_t val;
    int ret;

    ret = I2C_WRITEREAD(i2c, &reg, sizeof(reg), &val, sizeof(val));
    return ret ? ret : val;
}

static int reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = val;

    return I2C_WRITE(i2c, buf, sizeof(buf));
}

static int reg_modify(uint8_t reg, uint8_t mask, uint8_t set)
{
    int ret = reg_read(reg);
    if (ret < 0)
        return ret;

    return reg_write(reg, (ret & ~mask) | (set & mask));
}

static int configure_device(void)
{
    int i, ret;

    for (i = 0; i < bq24292_cfg_size; i++) {
        if (bq24292_cfg[i].mask == 0xFF)
            ret = reg_write(bq24292_cfg[i].reg, bq24292_cfg[i].set);
        else
            ret = reg_modify(bq24292_cfg[i].reg, bq24292_cfg[i].mask, bq24292_cfg[i].set);

        if (ret)
            return ret;
    }

    return 0;
}

int bq24292_reg_read(uint8_t reg)
{
    int ret;

    ret = sem_wait(&sem);
    if (ret < 0)
        return -errno;

    ret = i2c ? reg_read(reg) : -ENODEV;
    sem_post(&sem);
    return ret;
}

int bq24292_reg_write(uint8_t reg, uint8_t val)
{
    int ret;

    ret = sem_wait(&sem);
    if (ret < 0)
        return -errno;

    ret = i2c ? reg_write(reg, val) : -ENODEV;
    sem_post(&sem);
    return ret;
}


int bq24292_reg_modify(uint8_t reg, uint8_t mask, uint8_t set)
{
    int ret;

    ret = sem_wait(&sem);
    if (ret < 0)
        return -errno;

    ret = i2c ? reg_modify(reg, mask, set) : -ENODEV;
    sem_post(&sem);
    return ret;
}

int bq24292_set_chg(enum chg config)
{
    switch (config) {
    case BQ24292_CHG_OFF:
        return bq24292_reg_modify(0x01, 0x30, 0x00);
    case BQ24292_CHG_BATTERY:
        return bq24292_reg_modify(0x01, 0x30, 0x10);
    case BQ24292_OTG_500MA:
        return bq24292_reg_modify(0x01, 0x31, 0x30);
    case BQ24292_OTG_1300MA:
        return bq24292_reg_modify(0x01, 0x31, 0x31);
    default:
        return -EINVAL;
    }
}

int bq24292_configure(void)
{
    int ret;

    ret = sem_wait(&sem);
    if (ret < 0)
        return -errno;

    ret = i2c ? configure_device() : -ENODEV;
    sem_post(&sem);
    return ret;
}

int bq24292_driver_init(int16_t pg_n)
{
    int ret;
    pthread_t vbus_thread;

    ret = sem_wait(&sem);
    if (ret < 0) {
        dbg("failed to acquire semaphore\n");
        return -errno;
    }

    if (i2c)
        goto init_done;

    i2c = up_i2cinitialize(CONFIG_CHARGER_BQ24292_I2C_BUS);
    if (!i2c) {
        dbg("failed to init i2c\n");
        ret = -ENODEV;
        goto init_done;
    }

    ret = I2C_SETADDRESS(i2c, BQ24292_I2C_ADDR, 7);
    if (ret) {
        dbg("failed to set i2c address\n");
        goto init_done;
    }

    I2C_SETFREQUENCY(i2c, CONFIG_CHARGER_BQ24292_I2C_BUS_SPEED);

    if (pg_n < 0)
        goto init_done;

    ret = gpio_irqattach(pg_n, pg_isr);
    if (ret) {
        dbg("failed to register for irq\n");
        goto init_done;
    }

    ret = set_gpio_triggering(pg_n, IRQ_TYPE_EDGE_FALLING);
    if (ret) {
        dbg("failed to set irq edge\n");
        goto init_done;
    }

    ret = pthread_create(&vbus_thread, NULL, pg_worker, NULL);
    if (ret) {
        dbg("failed to create thread\n");
        goto init_done;
    }

    ret = pthread_detach(vbus_thread);
    if (ret)
        dbg("failed to detach thread\n");

init_done:
    sem_post(&sem);
    return ret;
}
