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
#include <nuttx/power/bq25896.h>
#include <nuttx/util.h>
#include <nuttx/wqueue.h>

#define BQ25896_I2C_ADDR    (0x6B)

struct notify_node
{
  bq25896_callback callback;
  void *arg;
  struct list_head list;
};

static LIST_DECLARE(notify_list);

static sem_t sem = SEM_INITIALIZER(1);
static struct work_s int_work;
static struct work_s pg_work;
static struct work_s fault_work;

static bool pg_n_connected;

static FAR struct i2c_dev_s *i2c;

#define BQ25896_FAULT_DELAY         MSEC2TICK(100)

int bq25896_register_callback(bq25896_callback cb, void *arg)
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

    list_foreach(&notify_list, iter) {
        node = list_entry(iter, struct notify_node, list);
        node->callback(POWER_GOOD, node->arg);
    }
}

static int read_fault_reg(void)
{
    /*
     * From the spec:
     * "To read the current fault status, the host has to read REG0C two times
     * consecutively. The 1st read reports the pre-existing fault register
     * status and the 2nd read reports the current fault register status."
     */
    (void) bq25896_reg_read(BQ25896_REG0C);
    return bq25896_reg_read(BQ25896_REG0C);
}

static void fault_worker(FAR void *arg)
{
    struct notify_node *node;
    struct list_head *iter;
    int regval;

    regval = read_fault_reg();
    if (regval > 0 && regval & BQ25896_REG0C_BOOST_FAULT) {
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
    struct notify_node *node;
    struct list_head *iter;
    enum bq25896_event power_good_state;
    int regval;

    /* Boost fault */
    regval = read_fault_reg();
    if (regval > 0 && regval & BQ25896_REG0C_BOOST_FAULT) {
        vdbg("possible fault\n");
        if (work_available(&fault_work)) {
            work_queue(LPWORK, &fault_work, fault_worker, NULL,
                       BQ25896_FAULT_DELAY);
        }
    }

    /* Use status reg to detect power good unless pg_n line is connected */
    if (pg_n_connected)
        return;

    /*
     * Interrupt may not go off if battery is completely depleted and input
     * power source is removed, thus current state cannot be compared with the
     * previous one to detect when power becomes good, so notify the detection
     * of a good power source whenever the power good bit is asserted.
     */
    regval = bq25896_reg_read(BQ25896_REG0B);
    if (regval < 0)
        /* If unable to communicate with the IC, assume power is not good */
        return;

    /* Only interested in the power good bit */
    if (regval & BQ25896_REG0B_PG) {
        power_good_state = POWER_GOOD;
    } else {
        power_good_state = NO_POWER_GOOD;
    }
    vdbg("Report power_good = %d\n", power_good_state);

    list_foreach(&notify_list, iter) {
        node = list_entry(iter, struct notify_node, list);
        node->callback(power_good_state, node->arg);
    }
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
    if (work_available(&pg_work))
        return work_queue(LPWORK, &pg_work, pg_worker, NULL, 0);
    else
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

static int configure_device(const struct bq25896_config *cfg)
{
    int i, ret;

    if (!cfg)
        return 0;

    for (i = 0; i < cfg->reg_num; i++) {
        ret = reg_modify(cfg->reg[i].reg, cfg->reg[i].mask, cfg->reg[i].set);
        if (ret)
            return ret;
    }

    return 0;
}

int bq25896_reg_read(uint8_t reg)
{
    int ret;

    ret = sem_wait(&sem);
    if (ret < 0)
        return -errno;

    ret = i2c ? reg_read(reg) : -ENODEV;
    sem_post(&sem);
    return ret;
}

int bq25896_reg_write(uint8_t reg, uint8_t val)
{
    int ret;

    ret = sem_wait(&sem);
    if (ret < 0)
        return -errno;

    ret = i2c ? reg_write(reg, val) : -ENODEV;
    sem_post(&sem);
    return ret;
}

int bq25896_reg_modify(uint8_t reg, uint8_t mask, uint8_t set)
{
    int ret;

    ret = sem_wait(&sem);
    if (ret < 0)
        return -errno;

    ret = i2c ? reg_modify(reg, mask, set) : -ENODEV;
    sem_post(&sem);
    return ret;
}

int bq25896_configure(const struct bq25896_config *cfg)
{
    int ret;

    ret = sem_wait(&sem);
    if (ret < 0)
        return -errno;

    ret = i2c ? configure_device(cfg) : -ENODEV;
    sem_post(&sem);
    return ret;
}

int bq25896_driver_init(int16_t int_n, int16_t pg_n)
{
    int ret;

    ret = sem_wait(&sem);

    if (ret < 0) {
        dbg("failed to acquire semaphore\n");
        return -errno;
    }

    if (i2c)
        goto init_done;

    i2c = up_i2cinitialize(CONFIG_CHARGER_BQ25896_I2C_BUS);
    if (!i2c) {
        dbg("failed to init i2c\n");
        ret = -ENODEV;
        goto init_done;
    }

    I2C_SETADDRESS(i2c, BQ25896_I2C_ADDR, 7);
    I2C_SETFREQUENCY(i2c, CONFIG_CHARGER_BQ25896_I2C_BUS_SPEED);

    if (pg_n >= 0) {
        gpio_irqattach(pg_n, pg_isr);
        set_gpio_triggering(pg_n, IRQ_TYPE_EDGE_FALLING);
        pg_n_connected = true;
    }

    if (int_n >= 0) {
        gpio_irqattach(int_n, int_isr);
        set_gpio_triggering(int_n, IRQ_TYPE_EDGE_FALLING);
    }

init_done:
    sem_post(&sem);
    return ret;
}
