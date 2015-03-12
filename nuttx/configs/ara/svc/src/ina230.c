/*
 * Copyright (c) 2015 Google Inc.
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

/*
 * @file    configs/ara/svc/src/ina230.c
 * @brief   TI INA230 Driver
 * @author  Patrick Titiano
 */

#define DBG_COMP DBG_POWER

#include <ina230.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <up_debug.h>

#define INA230_CONFIG               0x00
#define INA230_SHUNT_VOLTAGE        0x01
#define INA230_BUS_VOLTAGE          0x02
#define INA230_POWER                0x03
#define INA230_CURRENT              0x04
#define INA230_CALIBRATION          0x05

/* CONFIG register bitfields */
#define INA230_CONFIG_POWER_MODE_MASK       ((uint16_t) 0x0007)
#define INA230_CONFIG_POWER_MODE_SHIFT      ((uint8_t) 0)
#define INA230_CONFIG_VSHUNT_CT_MASK        ((uint16_t) 0x0038)
#define INA230_CONFIG_VSHUNT_CT_SHIFT       ((uint8_t) 3)
#define INA230_CONFIG_VBUS_CT_MASK          ((uint16_t) 0x01C0)
#define INA230_CONFIG_VBUS_CT_SHIFT         ((uint8_t) 6)
#define INA230_CONFIG_AVG_MASK              ((uint16_t) 0x0E00)
#define INA230_CONFIG_AVG_SHIFT             ((uint8_t) 9)
#define INA230_CONFIG_RST_MASK              ((uint16_t) 0x8000)
#define INA230_CONFIG_RST_SHIFT             ((uint8_t) 15)

#define INA230_CALIBRATION_VALUE_MAX        ((uint16_t) 0x7FFF)
#define INA230_CALIBRATION_MULT             5120000 /* 0.00512 / uA / mohm */
#define INA230_VOLTAGE_LSB                  ((int32_t) 1250) /* 1.25mV */
#define INA230_POWER_CURRENT_RATIO          ((int32_t) 25)

/**
 * @brief           Read content from a given 16-bit register of a given
 *                  device on a given I2C bus.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: I2C bus device
 * @param[in]       addr: I2C device address
 * @param[in]       reg: I2C device register address
 * @param[out]      val: content read from I2C device register
 */
static int ina230_i2c_get(struct i2c_dev_s *dev,
                          uint8_t addr, uint8_t reg, uint16_t *val)
{
    int ret;
    uint8_t buf[2];

    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = &reg,
            .length = 1,
        }, {
            .addr = addr,
            .flags = I2C_M_READ,
            .buffer = buf,
            .length = 2,
        },
    };

    ret = I2C_TRANSFER(dev, msg, 2);
    if (ret == 0) {
        *val = (uint16_t) buf[1] + (((uint16_t) buf[0]) << (uint16_t) 8);
        dbg_verbose("%s(): addr=0x%02X, reg=0x%02hhX: buf[0]=0x%02hhX buf[1]=0x%02hhX read 0x%04hX\n",
                    __func__, addr, reg, buf[0], buf[1], *val);
    } else {
        *val = 0;
        dbg_error("%s(): addr=0x%02hhX, reg=0x%02hhX: failed!\n",
                  __func__, addr, reg);
    }

    return ret;
}

/**
 * @brief           Write data into a given 16-bit register of a given
 *                  device on a given I2C bus.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: I2C bus device
 * @param[in]       addr: I2C device address
 * @param[in]       reg: I2C device register address
 * @param[in]       val: data to be written into I2C device register
 */
static int ina230_i2c_set(struct i2c_dev_s *dev,
                          uint8_t addr, uint8_t reg, uint16_t val)
{
    int ret;
    uint8_t val_lsb = (uint8_t) (val & 0x00FF);
    uint8_t val_msb = (uint8_t) ((val & 0xFF00) >> 8);
    uint8_t cmd[3] = {reg, val_msb, val_lsb};
    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = cmd,
            .length = 3,
        },
    };

    dbg_verbose("%s(): addr=0x%02hhX: reg=0x%02hhX, val=0x%04hX\n",
                __func__, addr, reg, val);
    ret = I2C_TRANSFER(dev, msg, 1);

    return ret;
}

/**
 * @brief           Update register content of a given 16-bit register of a
 *                  given device on a given I2C bus.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: I2C bus device
 * @param[in]       addr: I2C device address
 * @param[in]       reg: I2C device register address
 * @param[in]       mask: mask to be applied to the register content
 * @param[in]       shift: shift to be applied to argument 'val'
 * @param[in]       val: data to be written into I2C device register
 */
static int ina230_update_reg(struct i2c_dev_s *dev, uint8_t addr, uint8_t reg,
                             uint16_t mask, uint8_t shift, uint16_t val)
{
    uint16_t content;
    int ret;

    dbg_verbose("%s(): addr=0x%02hhX: reg=0x%02hhX, mask=0x%02hhX, shift=%02hhu, val=0x%04hX\n",
                __func__, addr, reg, mask, shift, val);
    if (addr >= 0x7F) {
        return -EINVAL;
    }

    /* Get register content */
    ret = ina230_i2c_get(dev, addr, reg, &content);
    if (ret) {
        return ret;
    }
    /* Update register content */
    dbg_verbose("%s(): content=0x%04hX\n", __func__, content);
    content &= ~mask;
    content |= (val << shift);
    dbg_verbose("%s(): new content=0x%04hX\n", __func__, content);
    /* Write back new register content */
    return ina230_i2c_set(dev, addr, reg, content);
}

/**
 * @brief           Reset INA230 chip (restore default configuration).
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: INA230 device
 */
static int ina230_reset(ina230_device *dev)
{
    dbg_verbose("%s(): addr=0x%02hhX\n", __func__, dev->addr);
    return ina230_update_reg(dev->i2c_dev, dev->addr,
                             INA230_CONFIG,
                             INA230_CONFIG_RST_MASK,
                             INA230_CONFIG_RST_SHIFT,
                             1);
}

/**
 * @brief           Configure INA230 mode.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: INA230 device
 */
static int ina230_set_mode(ina230_device *dev)
{
    dbg_verbose("%s(): addr=0x%02hhX, mode=%02hhu\n",
                __func__, dev->addr, dev->mode);
    return ina230_update_reg(dev->i2c_dev, dev->addr,
                             INA230_CONFIG,
                             INA230_CONFIG_POWER_MODE_MASK,
                             INA230_CONFIG_POWER_MODE_SHIFT,
                             dev->mode);
}

/**
 * @brief           Set INA230 convertion time.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: INA230 device
 */
static int ina230_set_conversion_time(ina230_device *dev)
{
    int ret;

    dbg_verbose("%s(): addr=0x%02hhX, ct=%02hhu\n",
                __func__, dev->addr, dev->ct);
    ret = ina230_update_reg(dev->i2c_dev, dev->addr,
                             INA230_CONFIG,
                             INA230_CONFIG_VSHUNT_CT_MASK,
                             INA230_CONFIG_VSHUNT_CT_SHIFT,
                             dev->ct);
    ret = ret || ina230_update_reg(dev->i2c_dev, dev->addr,
                                   INA230_CONFIG,
                                   INA230_CONFIG_VBUS_CT_MASK,
                                   INA230_CONFIG_VBUS_CT_SHIFT,
                                   dev->ct);
    if (ret) {
        dbg_error("%s(): failed to set convertion time!\n", __func__);
        return -EIO;
    } else {
        return 0;
    }
}

/**
 * @brief           Set INA230 averaging sample count.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: INA230 device
 */
static int ina230_set_avg_sample_count(ina230_device *dev)
{
    dbg_verbose("%s(): addr=0x%02hhX, count=%02hhu\n",
                __func__, dev->addr, dev->count);

    return ina230_update_reg(dev->i2c_dev, dev->addr,
                             INA230_CONFIG,
                             INA230_CONFIG_AVG_MASK,
                             INA230_CONFIG_AVG_SHIFT,
                             dev->count);
}

/**
 * @brief           convert argument of type ina230_conversion_time to integer.
 * @return          conversion time in microseconds
 *                  0 in case of error
 * @param[in]       ct: conversion time
 */
uint32_t ina230_ct_to_int(ina230_conversion_time ct)
{
    static const int32_t ctus[ina230_ct_count] = {
        140,
        204,
        332,
        588,
        1100,
        2116,
        4156,
        8244
    };

    if (ct < ina230_ct_count) {
        dbg_verbose("%s(): ct=%u => %uus\n", __func__, ct, ctus[ct]);
        return ctus[ct];
    } else {
        dbg_error("%s(): invalid ct!!! (%u)\n", __func__, ct);
        return 0;
    }
}

/**
 * @brief           convert argument of type ina230_avg_count to integer.
 * @return          averaging sample count (integer value)
 *                  0 in case of error
 * @param[in]       avg_count: averaging sample count
 */
uint32_t ina230_avg_count_to_int(ina230_avg_count avg_count)
{
    static const int32_t avg_counts[ina230_avg_count_max] = {
        1,
        4,
        16,
        64,
        128,
        256,
        512,
        1024
    };

    if (avg_count < ina230_avg_count_max) {
        dbg_verbose("%s(): avg_count=%u => %u\n", __func__,
                    avg_count, avg_counts[avg_count]);
        return avg_counts[avg_count];
    } else {
        dbg_error("%s(): invalid avg_count!!! (%u)\n", __func__, avg_count);
        return 0;
    }
}

/**
 * @brief           Return the time between 2 measurement samples,
 *                  considering conversion time and averaging sample count.
 * @return          time between 2 measurement samples in microseconds
 *                  0 in case of error
 * @param[in]       ct: conversion time
 * @param[in]       avg_count: averaging sample count
 * @param[in]       mode: power mode
 */
uint32_t ina230_get_sampling_time(ina230_conversion_time ct,
                                 ina230_avg_count avg_count,
                                 ina230_power_mode mode)
{
    uint32_t stime;

    switch (mode) {
    case ina230_power_down:
        stime = 0;
        break;
    case ina230_shunt_trig:
    case ina230_shunt_cont:
    case ina230_bus_trig:
    case ina230_bus_cont:
        stime = ina230_ct_to_int(ct) * ina230_avg_count_to_int(avg_count);
        break;
    case ina230_shunt_bus_trig:
    case ina230_shunt_bus_cont:
        stime = 2 * ina230_ct_to_int(ct) * ina230_avg_count_to_int(avg_count);
        break;
    default:
        dbg_error("%s(): invalid argument!!!\n", __func__);
        return 0;
    }


    dbg_verbose("%s(): ct=%u (%uus), avg_count=%u (%u) mode=%u => stime=%uus\n",
                __func__, ct, ina230_ct_to_int(ct),
                avg_count, ina230_avg_count_to_int(avg_count), mode, stime);
    return stime;
}

/**
 * @brief           Set INA230 current LSB (Least Significant Bit).
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: INA230 device
 */
static int ina230_set_current_lsb(ina230_device *dev)
{
    uint16_t cal;

    dbg_verbose("%s(): addr=0x%02hhX, mohm=%u, current_lsb=%u\n",
                __func__, dev->addr, dev->mohm, dev->current_lsb);
    /* Convert to CALIBRATION value (equation 1 in datasheet) */
    cal = INA230_CALIBRATION_MULT / (dev->mohm * dev->current_lsb);
    dbg_verbose("%s(): cal=%u\n", __func__, cal);
    return ina230_i2c_set(dev->i2c_dev, dev->addr, INA230_CALIBRATION, cal);
}

/**
 * @brief           Initialize INA230 device.
 * @warning         A non-negligible time is required after initialization
 *                  before being able to retrieve the first measurements.
 *                  It is proportional to the configured conversion time and
 *                  averaging count. e.g. for a conversion time of 8.244ms and
 *                  an averaging of 128 samples, the first measurement will be
 *                  available only after at least 128 * 8.244ms = 1055ms (1s!)
 *                  Also, the same delay will be necessary between measurements,
 *                  or previous value will be read.
 *                  Therefore a relevant delay shall be used:
 *                   - Before starting collecting measurements
 *                     (otherwise 0 will be read)
 *                   - Between 2 measurements
 *                     (otherwise previous sample will be read)
 * @return          0 on success, standard error codes otherwise
 * @param[in]       i2c_dev: I2C bus device
 * @param[in]       addr: INA230 device I2C address on bus
 * @param[in]       mohm: shunt resistor value (in milli-ohms)
 * @param[in]       current_lsb: current measurement LSB
 * @param[in]       ct: convertion time to use
 * @param[in]       count: averaging sample count
 * @param[in]       mode: INA230 mode
 */
ina230_device *ina230_init(struct i2c_dev_s *i2c_dev, uint8_t addr,
                           uint32_t mohm, uint32_t current_lsb,
                           ina230_conversion_time ct,
                           ina230_avg_count count,
                           ina230_power_mode mode)
{
    ina230_device *ina230_dev;
    int ret = 0;

    dbg_verbose("%s(): initializing INA230 device...\n", __func__);
    if ((!i2c_dev)
        || (addr >= 0x7F)
        || (ct >= ina230_ct_count)
        || (count >= ina230_avg_count_max)
        || (mode >= ina230_power_mode_count)) {
        dbg_error("%s(): invalid argument!\n", __func__);
        return NULL;
    }
    ina230_dev = malloc(sizeof(ina230_device));
    if (!ina230_dev) {
        dbg_error("%s(): failed to alloc device struct!\n", __func__);
        return NULL;
    }
    dbg_verbose("%s(): init options: addr=0x%02X, mohm=%u, lsb=%uuA, ct=%u, avg_count=%u, mode=%u\n",
                __func__, addr, mohm, current_lsb, ct, count, mode);
    ina230_dev->i2c_dev = i2c_dev;
    ina230_dev->addr = addr;
    ina230_dev->mohm = mohm;
    ina230_dev->current_lsb = current_lsb;
    ina230_dev->mode = mode;
    ina230_dev->ct = ct;
    ina230_dev->count = count;

    ret = ina230_reset(ina230_dev);
    ret = ret || ina230_set_conversion_time(ina230_dev);
    ret = ret || ina230_set_avg_sample_count(ina230_dev);
    ret = ret || ina230_set_current_lsb(ina230_dev);
    ret = ret || ina230_set_mode(ina230_dev);
    if (ret) {
        dbg_error("%s(): Failed to init device! (%d)\n", __func__, ret);
        free(ina230_dev);
        return NULL;
    }

    dbg_verbose("%s(): ina230 device successfully created.\n", __func__);
    return ina230_dev;
}

/**
 * @brief           Return latest sample measurements from INA230 device.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: INA230 device
 * @param[in]       m: measurement data (voltage, current, power)
 */
int ina230_get_data(ina230_device *dev, pwr_measure *m)
{
    int ret;
    int16_t raw_vbus, raw_current, raw_power;

    if ((!dev) || (!m)) {
        return -EINVAL;
    }
    m->uV = 0;
    m->uA = 0;
    m->uW = 0;

    ret = ina230_i2c_get(dev->i2c_dev, dev->addr,
                         INA230_BUS_VOLTAGE, (uint16_t *) &raw_vbus);
    ret = ret || ina230_i2c_get(dev->i2c_dev, dev->addr,
                                INA230_CURRENT, (uint16_t *) &raw_current);
    ret = ret || ina230_i2c_get(dev->i2c_dev, dev->addr,
                                INA230_POWER, (uint16_t *) &raw_power);
    if (ret) {
        dbg_error("%s(): failed to read data registers! (%d)\n", __func__, ret);
        return -EIO;
    }
    dbg_verbose("%s(): addr=0x%02X raw_vbus=0x%04X raw_current=0x%04X raw_power=0x%04X\n",
                __func__, dev->addr, raw_vbus, raw_current, raw_power);

    /* VBUS LSB = 1.25 mV. */
    m->uV = (int32_t) raw_vbus * INA230_VOLTAGE_LSB;

    /* Current register LSB programmed during init. Get it from structure. */
    m->uA = (int32_t) raw_current * dev->current_lsb;

    /*
     * The Power register LSB is internally programmed to equal 25 times
     * the programmed value of the Current_LSB.
     */
    m->uW = (int32_t) raw_power * INA230_POWER_CURRENT_RATIO * dev->current_lsb;

    dbg_verbose("%s(): addr=0x%02X ret=%d => %duV %duA %duW\n",
                __func__, dev->addr, ret, m->uV, m->uA, m->uW);
    return ret;
}

/**
 * @brief           Denitialize INA230 device.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       dev: INA230 device
 */
int ina230_deinit(ina230_device *dev)
{
    int ret;

    if (!dev) {
        return -EINVAL;
    }

    /* Reset INA230 device */
    ret = ina230_reset(dev);
    if (ret) {
        return ret;
    }
    /* Put device in power down mode */
    dev->mode = ina230_power_down;
    ret = ina230_set_mode(dev);
    if (ret) {
        return ret;
    }
    /* Free allocated memory */
    free(dev);

    return 0;
}
