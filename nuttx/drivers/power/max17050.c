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

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c.h>
#include <nuttx/power/battery.h>

#define MAX17050_I2C_ADDR         0x36

#define MAX17050_REG_REP_SOC      0x06
#define MAX17050_REG_TEMP         0x08
#define MAX17050_REG_VCELL        0x09
#define MAX17050_REG_CURRENT      0x0A
#define MAX17050_REG_FULL_CAP     0x10
#define MAX17050_REG_MAX_VOLT     0x1B
#define MAX17050_REG_DEV_NAME     0x21

#define MAX17050_IC_VERSION     0x00AC

#define MAX17050_SNS_RESISTOR    10000

struct max17050_dev_s
{
    /* The common part of the battery driver visible to the upper-half driver */

    FAR const struct battery_operations_s *ops;
    sem_t batsem;

    /* Data fields specific to the lower half MAX17050 driver follow */

    FAR struct i2c_dev_s *i2c;
};

static int max17050_reg_read(FAR struct max17050_dev_s *priv, uint16_t reg)
{
    uint16_t reg_val;
    int ret;

    ret = I2C_WRITEREAD(priv->i2c, (uint8_t *)&reg, sizeof(reg),
                        (uint8_t *)&reg_val, sizeof(reg_val));
    return ret ? ret : reg_val;
}

static int max17050_online(struct battery_dev_s *dev, bool *status)
{
    *status = true;
    return OK;
}

static int max17050_voltage(struct battery_dev_s *dev, b16_t *voltage)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    ret = max17050_reg_read(priv, MAX17050_REG_VCELL);
    if (ret < 0)
        return ret;

    *voltage = ret * 625 / 8;

    lowsyslog("%s: %d mV\n", __func__, *voltage);
    return OK;
}

static int max17050_capacity(struct battery_dev_s *dev, b16_t *capacity)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    ret = max17050_reg_read(priv, MAX17050_REG_REP_SOC);
    if (ret < 0)
        return ret;

    *capacity = ret >> 8;

    lowsyslog("%s: %d percent\n", __func__, *capacity);
    return OK;
}

static int max17050_max_voltage(struct battery_dev_s *dev, b16_t *max_voltage)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    ret = max17050_reg_read(priv, MAX17050_REG_MAX_VOLT);
    if (ret < 0)
        return ret;

    *max_voltage = ret >> 8;
    *max_voltage *= 20000; /* Units of LSB = 20mV */

    lowsyslog("%s: %d mV\n", __func__, *max_voltage);
    return OK;
}

static int max17050_temperature(struct battery_dev_s *dev, b16_t *temperature)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    ret = max17050_reg_read(priv, MAX17050_REG_TEMP);
    if (ret < 0)
        return ret;

    *temperature = ret;
    /* The value is signed. */
    if (*temperature & 0x8000) {
        *temperature = (0x7fff & ~*temperature) + 1;
        *temperature *= -1;
    }

    /* The value is converted into deci-centigrade scale */
    /* Units of LSB = 1 / 256 degree Celsius */
    *temperature = *temperature * 10 / 256;

    lowsyslog("%s: %d\n", __func__, *temperature);
    return OK;
}

static int max17050_current(struct battery_dev_s *dev, b16_t *current)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    ret = max17050_reg_read(priv, MAX17050_REG_CURRENT);
    if (ret < 0)
        return ret;

    *current = ret;
    if (*current & 0x8000) {
        /* Negative */
        *current = ~*current & 0x7fff;
        *current += 1;
        *current *= -1;
    }
    *current *= 1562500 / MAX17050_SNS_RESISTOR;

    lowsyslog("%s: %d uA\n", __func__, *current);
    return OK;
}

static int max17050_full_capacity(struct battery_dev_s *dev, b16_t *capacity)
{
    FAR struct max17050_dev_s *priv = (FAR struct max17050_dev_s *)dev;
    int ret;

    ret = max17050_reg_read(priv, MAX17050_REG_FULL_CAP);
    if (ret < 0)
        return ret;

    *capacity = ret * 1000 / 2;

    lowsyslog("%s: %d mAh\n", __func__, *capacity);
    return OK;
}

static int max17050_state(struct battery_dev_s *dev, int *status)
{
    int ret;
    b16_t value;

    ret = max17050_capacity(dev, &value);
    if (ret < 0) {
        *status = BATTERY_UNKNOWN;
        return ret;
    }

    // TODO: Currently treating 100% capacity as full. Should it be based on
    // capacity or when input current drops below certain threshold?
    if (value >= 100) {
        *status = BATTERY_FULL;
        return OK;
    }

    ret = max17050_current(dev, &value);
    if (ret < 0) {
        *status = BATTERY_UNKNOWN;
        return ret;
    }

    // Positive current means the battery is charging
    *status = (value > 0) ? BATTERY_CHARGING : BATTERY_DISCHARGING;

    lowsyslog("%s: %d\n", __func__, *status);
    return OK;
}

static const struct battery_operations_s max17050_ops =
{
    .state = max17050_state,
    .online = max17050_online,
    .voltage = max17050_voltage,
    .capacity = max17050_capacity,
    .max_voltage = max17050_max_voltage,
    .temperature = max17050_temperature,
    .current = max17050_current,
    .full_capacity = max17050_full_capacity,
};

FAR struct battery_dev_s *max17050_initialize(FAR struct i2c_dev_s *i2c,
                                              uint32_t frequency)
{
    FAR struct max17050_dev_s *priv;
    int ret;

    priv = (FAR struct max17050_dev_s *)kmm_zalloc(sizeof(*priv));
    if (priv) {
        sem_init(&priv->batsem, 0, 1);
        priv->ops = &max17050_ops;
        priv->i2c = i2c;

        ret = I2C_SETADDRESS(i2c, MAX17050_I2C_ADDR, 7);
        if (ret)
            goto err;

        I2C_SETFREQUENCY(i2c, frequency);

        ret = max17050_reg_read(priv, MAX17050_REG_DEV_NAME);
        if (ret < 0)
            goto err;

        if (ret != MAX17050_IC_VERSION)
            goto err;

        // TODO: Initialize IC registers as needed
    }

    return (FAR struct battery_dev_s *)priv;

err:
    kmm_free(priv);
    return NULL;
}

