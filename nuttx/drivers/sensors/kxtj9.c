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
#include <stddef.h>
#include <string.h>

#include <nuttx/i2c.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/kxtj9.h>

/* OUTPUT REGISTERS */
#define XOUT_L          0x06
#define WHO_AM_I        0x0F
#define DCST_RESP       0x0C

/* CONTROL REGISTERS */
#define INT_REL         0x1A
#define CTRL_REG1       0x1B
#define INT_CTRL1       0x1E
#define DATA_CTRL       0x21
#define CTRL_REG2       0x1D

/* CONTROL REGISTER 1 BITS */
#define PC1_OFF         0x7F
#define PC1_ON          (1 << 7)

/* CTRL_REG1: set resolution, g-range, data ready enable */
/* Output resolution: 8-bit valid or 12-bit valid */
#define RES_8BIT        0
#define RES_12BIT       (1 << 6)

/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE           (1 << 5)

/* Output g-range: +/-2g, 4g, or 8g */
#define KXTJ9_G_2G      0
#define KXTJ9_G_4G      (1 << 3)
#define KXTJ9_G_8G      (1 << 4)

/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL       (1 << 3)
#define KXTJ9_IEA       (1 << 4)
#define KXTJ9_IEN       (1 << 5)

#define KXTJ9_SRST      0x80
#define WHO_AM_I_KXCJ9  0x0A

#define KXTJ9_I2C_ADDR      0x0E
#define ACCEL_NUM_RETRIES   5
#define KXTJ9_CTRL1_CONFIG  (RES_12BIT | KXTJ9_G_2G | DRDYE)

static sem_t sem = SEM_INITIALIZER(1);

struct kxtj9_data {
    struct i2c_dev_s *i2c;
    bool enable;
    uint8_t shift;
    uint8_t ctrl_reg1;
    uint8_t data_ctrl;
    uint8_t int_ctrl;
    bool power_enabled;
};

static FAR struct kxtj9_data *g_data;
static void kxtj9_soft_reset(void);
static void kxtj9_set_mode_standby(void);

static uint8_t kxtj9_reg_read(uint8_t reg, uint8_t *data, int len)
{
    uint8_t buf[1];
    int ret = 0;
    int index = ACCEL_NUM_RETRIES;
    buf[0] = reg;
    do {
        ret = I2C_WRITEREAD(g_data->i2c, buf, 1, data, len);
    } while (ret != 0 && index--);
    return ret;
}

static int kxtj9_reg_write8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2];
    int ret = 0;
    int index = ACCEL_NUM_RETRIES;

    buf[0] = reg;
    buf[1] = val;
    do {
        ret = I2C_WRITE(g_data->i2c, buf, 2);
    } while (ret != 0 && index--);
    return ret;
}

static void kxtj9_soft_reset()
{
    uint8_t wbuf[1];
    /* set accel into standby and known state by disabling PC1 */
    wbuf[0] = KXTJ9_CTRL1_CONFIG;
    kxtj9_reg_write8(CTRL_REG1, wbuf[0]);

    /* send the reset command */
    kxtj9_reg_read(CTRL_REG2, &wbuf[0], 1);
    wbuf[0] |= KXTJ9_SRST;
    kxtj9_reg_write8(CTRL_REG2, wbuf[0]);

    /* delay 10ms for the accel parts to re-initialize */
    usleep(10000);
}

static void kxtj9_set_mode_standby()
{
    uint8_t wbuf[1];
    /* set Accel into standby and known state by disabling PC1 */
    wbuf[0] = KXTJ9_CTRL1_CONFIG;
    kxtj9_reg_write8(CTRL_REG1, wbuf[0]);

    /* clear interrupts */
    wbuf[0] = 0;
    kxtj9_reg_write8(INT_CTRL1, wbuf[0]);
}

int kxtj9_driver_init(){
    int ret = 0;

    if (g_data != NULL)
        goto init_done;
    g_data = kmm_malloc(sizeof(struct kxtj9_data));
    memset(g_data, 0, sizeof(struct kxtj9_data));

    g_data->i2c = up_i2cinitialize(CONFIG_SENSOR_KXTJ9_I2C_BUS);
    if (!g_data->i2c) {
        dbg("failed to init i2c\n");
        ret = -ENODEV;
        goto init_done;
    }

    ret = I2C_SETADDRESS(g_data->i2c, KXTJ9_I2C_ADDR, 7);
    if (ret) {
        dbg("failed to set i2c address\n");
        goto init_done;
    }

    I2C_SETFREQUENCY(g_data->i2c, CONFIG_SENSOR_KXTJ9_I2C_BUS_SPEED);

init_done:
    return ret;
}

int kxtj9_configure(uint8_t odr){
    uint8_t wbuf[0];
    int ret;
    do {
        ret = sem_wait(&sem);
    } while (ret < 0 && errno == EINTR);

    kxtj9_soft_reset();
    kxtj9_set_mode_standby();

    /* read WHO_AM_I register, should return 0x0A */
    kxtj9_reg_read(WHO_AM_I, &wbuf[0], 1);
    if (wbuf[0] != WHO_AM_I_KXCJ9)
    {
        dbg("Not KXCJ9 chipset, WHO_AM_I register is 0x%2x.\n", wbuf[0]);
    }

    /* ensure that PC1 is cleared before updating control registers */
    kxtj9_reg_write8(CTRL_REG1, 0);

    /* 12Bit Res and -2G~+2G range */
    g_data->ctrl_reg1 = KXTJ9_CTRL1_CONFIG;
    kxtj9_reg_write8(CTRL_REG1, g_data->ctrl_reg1);

    g_data->data_ctrl = odr;
    kxtj9_reg_write8(DATA_CTRL, g_data->data_ctrl);

    /* In irq mode, populate INT_CTRL */
    g_data->int_ctrl = KXTJ9_IEN | KXTJ9_IEA | KXTJ9_IEL;
    kxtj9_reg_write8(INT_CTRL1, g_data->int_ctrl);

    sem_post(&sem);
    return 0;
}

int kxtj9_enable(bool on)
{
    uint8_t wbuf[1];
    int ret;
    do {
        ret = sem_wait(&sem);
    } while (ret < 0 && errno == EINTR);

    if (!on && g_data->power_enabled) {
        g_data->ctrl_reg1 &= PC1_OFF;
        kxtj9_reg_write8(CTRL_REG1, g_data->ctrl_reg1);
        g_data->power_enabled = false;
        dbg("KXTJ9 in disabled mode\n");
    } else if (on && !g_data->power_enabled){
        /* turn on outputs */
        g_data->ctrl_reg1 |= PC1_ON;
        kxtj9_reg_write8(CTRL_REG1, g_data->ctrl_reg1);
        /* clear initial interrupt if in irq mode */
        kxtj9_reg_read(INT_REL, wbuf, 1);
        g_data->power_enabled = true;
        dbg("KXTJ9 in operating mode\n");
    }

    sem_post(&sem);
    return 0;
}

int kxtj9_read_sensor_data(struct kxtj9_sensor_data *sensor_data)
{
    int16_t acc_data[3];
    uint8_t data;
    int ret;
    do {
        ret = sem_wait(&sem);
    } while (ret < 0 && errno == EINTR);
    kxtj9_reg_read(XOUT_L, (uint8_t *)acc_data, 6);

    /* 12 bit resolution, get rid of the lowest 4 bits */
    sensor_data->x = acc_data[0] >> 4;
    sensor_data->y = acc_data[1] >> 4;
    sensor_data->z = acc_data[2] >> 4;

    /* Read INT_REL to clear interrupt status */
    kxtj9_reg_read(INT_REL, &data, 1);
    sem_post(&sem);
    return 0;
}
