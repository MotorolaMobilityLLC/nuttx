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
#include <semaphore.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include <nuttx/clock.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/list.h>
#include <nuttx/power/bq24292.h>
#include <nuttx/util.h>
#include <nuttx/wqueue.h>

#include "bq24292_config.h"

struct notify_node
{
  bq24292_callback callback;
  void *arg;
  struct list_head list;
};

static LIST_DECLARE(notify_list);

static sem_t sem = SEM_INITIALIZER(1);
static struct work_s int_work;
static struct work_s pg_work;
static struct work_s fault_work;

static bool pg_n_connected;
static bool power_good;

static FAR struct i2c_dev_s *i2c;

#define BQ24292_I2C_ADDR            0x6B
#define BQ24292_PG_DELAY            MSEC2TICK(500)

#define BQ24292_REG_STATUS          0x08
#define BQ24292_REG_FAULT           0x09

#define BQ24292_STATUS_PG           0x04
#define BQ24292_FAULT_BOOST         0x40
#define BQ24292_FAULT_DELAY         MSEC2TICK(100)

int bq24292_register_callback(bq24292_callback cb, void *arg)
{
    struct notify_node *node;

    node = malloc(sizeof(*node));
    if (!node) {
        return -ENOMEM;
    }

    node->callback = cb;
    node->arg = arg;

    list_add(&notify_list, &node->list);

    return 0;
}

static void pg_worker(FAR void *arg)
{
    struct notify_node *node;
    struct list_head *iter;
    int regval;
    bool pg_now;

    /*
     * Interrupt may not go off if battery is completely depleted and input
     * power source is removed, thus current state cannot be compared with the
     * previous one to detect when power becomes good, so notify the detection
     * of a good power source when the power good bit is asserted.
     */

    /* Use status reg to detect power good unless pg_n line is connected */
    regval = bq24292_reg_read(BQ24292_REG_STATUS);
    if (regval < 0)
        /* If unable to communicate with the IC, assume power is not good */
        pg_now = false;
    else
        pg_now = (regval & BQ24292_STATUS_PG) > 0;
    /* Only invoke callback when PG status changes  */
    if (pg_now != power_good) {
        power_good = pg_now;
        // Configure registers every time power is attached
        if(power_good) bq24292_configure();

        list_foreach(&notify_list, iter) {
            node = list_entry(iter, struct notify_node, list);
            node->callback(power_good ? POWER_GOOD: POWER_NOT_GOOD,
                           node->arg);
        }
    }
}

static int read_fault_reg(void)
{
    /*
     * From the spec:
     * "To read the current fault status, the host has to read the register two
     * times consecutively. The 1st read reports the pre-existing fault register
     * status and the 2nd read reports the current fault register status."
     */
    (void) bq24292_reg_read(BQ24292_REG_FAULT);
    return bq24292_reg_read(BQ24292_REG_FAULT);
}

static void fault_worker(FAR void *arg)
{
    struct notify_node *node;
    struct list_head *iter;
    int regval;

    regval = read_fault_reg();
    if (regval > 0 && regval & BQ24292_FAULT_BOOST) {
        list_foreach(&notify_list, iter) {
            node = list_entry(iter, struct notify_node, list);
            node->callback(BOOST_FAULT, node->arg);
        }
    } else {
        dbg("fault gone\n");
    }
}

static void int_worker(FAR void *arg)
{
    int regval;

    /* Boost fault */
    regval = read_fault_reg();
    if (regval > 0 && regval & BQ24292_FAULT_BOOST) {
        vdbg("possible fault\n");
        if (work_available(&fault_work)) {
            work_queue(LPWORK, &fault_work, fault_worker, NULL,
                       BQ24292_FAULT_DELAY);
        }
    }

    if (pg_n_connected)
        return;

    if (!work_available(&pg_work))
        work_cancel(LPWORK, &pg_work);

    work_queue(LPWORK, &pg_work, pg_worker, NULL, BQ24292_PG_DELAY);
}

static int int_isr(int irq, void *context)
{

    if (work_available(&int_work))
        return work_queue(LPWORK, &int_work, int_worker, NULL, 0);
    else
        return OK;
}

static int pg_isr(int irq, void *context)
{
    /* Ensure input source detection is finished before PG worker runs */
    if (!work_available(&pg_work))
        work_cancel(LPWORK, &pg_work);
    return work_queue(LPWORK, &pg_work, pg_worker, NULL, BQ24292_PG_DELAY);
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

    dbg("Reconfiguring BQ24292\n");
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

// Supported input current limits in mA. Index corresonds to bit value.
static const int current_limits[] = {
    100, 150, 500, 900, 1200, 1500, 2000, 3000
};

int bq24292_set_input_current_limit(int limit)
{
    int i;

    /* Program max possible setting that is less or equal to the limit */
    for (i = ARRAY_SIZE(current_limits) -1; i >=0; i--) {
        if (limit >= current_limits[i]) {
            return bq24292_reg_modify(0x00, 0x07, i);
        }
    }

    return -EINVAL;
}

#define MIN_INPUT_VOLTAGE   3880
#define INPUT_VOLTAGE_STEP  80
int bq24292_set_input_voltage_limit(int limit)
{
    int val;
    int delta;

    /* Program min possible setting that is greater or equal to the limit */
    if (limit <= MIN_INPUT_VOLTAGE) {
        val = 0;
    } else {
        delta = limit - MIN_INPUT_VOLTAGE;
        val = delta / INPUT_VOLTAGE_STEP;
        if (delta % INPUT_VOLTAGE_STEP) {
            val +=1;
        }
        if (val > 15) {
            return -EINVAL;
        }
    }

    return bq24292_reg_modify(0x00, 0x78, val << 3);
}

#define MIN_CHARGE_CURRENT              512
#define MIN_CHARGE_CURRENT_BATFET_OCP   1024 /* lowest IC limit greater than 1A */
#define CHARGE_CURRENT_STEP             64
int bq24292_set_charge_current_limit(int limit)
{
    bool force_20pct = limit < MIN_CHARGE_CURRENT_BATFET_OCP;
    int val;
    int delta;

    /* Work-around for the bq24196 issue where charge current less than 1A may
     * result in triggering BATFET over-current protection */
    if (force_20pct)
        limit *=5;

    /* Do not program the limit that can result in BATFET OCP */
    if (limit < MIN_CHARGE_CURRENT_BATFET_OCP)
        return -EINVAL;

    /* Program max possible setting that is less or equal to the limit */
    delta = limit - MIN_CHARGE_CURRENT;
    val = delta / CHARGE_CURRENT_STEP;
    if (val > 63) {
        val = 63;
    }

    return bq24292_reg_modify(0x02, 0xFD, force_20pct ? (val << 2) + 1: val << 2);
}

#define MIN_CHARGE_VOLTAGE  3504
#define CHARGE_VOLTAGE_STEP 16
int bq24292_set_charge_voltage_limit(int limit)
{
    int val;
    int delta;

    /* Program max possible setting that is less or equal to the limit */
    if (limit < MIN_CHARGE_VOLTAGE) {
        return -EINVAL;
    }

    delta = limit - MIN_CHARGE_VOLTAGE;
    val = delta / CHARGE_VOLTAGE_STEP;
    if (val > 63) {
        val = 63;
    }

    return bq24292_reg_modify(0x04, 0xFC, val << 2);
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

int bq24292_driver_init(int16_t int_n, int16_t pg_n)
{
    int ret;

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

    if (pg_n >= 0) {
        ret = gpio_irqattach(pg_n, pg_isr);
        if (ret) {
            dbg("failed to register for pg irq\n");
            goto init_done;
        }

        ret = set_gpio_triggering(pg_n, IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING);
        if (ret) {
            dbg("failed to set pg irq edge\n");
            goto init_done;
        }

        pg_n_connected = true;
    }

    if (int_n >= 0) {
        ret = gpio_irqattach(int_n, int_isr);
        if (ret) {
            dbg("failed to register for int irq\n");
            goto init_done;
        }

        ret = set_gpio_triggering(int_n, IRQ_TYPE_EDGE_FALLING);
        if (ret) {
            dbg("failed to set int irq edge\n");
            goto init_done;
        }
    }

    /* disable ship mode, allow BATFET turn on */
    reg_modify(0x07, 0x20, 0x00);

    /* Perform initial configuration */
    (void) configure_device();

    power_good = false;

init_done:
    sem_post(&sem);
    return ret;
}
