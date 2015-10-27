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
 * @file    configs/ara/svc/src/pwr_mon.h
 * @brief   ARA Power Measurement Library
 * @author  Patrick Titiano
 */

#define DBG_COMP ARADBG_POWER

#include <pwr_mon.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <nuttx/i2c.h>

#include <nuttx/gpio.h>
#include <nuttx/gpio/tca64xx.h>

#include <ara_debug.h>
#include "ara_board.h"

#define INVALID_I2C_ADDR            0xFF
#define INA230_SHUNT_VALUE          2 /* mohm */

static struct i2c_dev_s *i2c_dev;
static uint32_t pwrmon_current_lsb;
static ina230_conversion_time pwrmon_ct;
static ina230_avg_count pwrmon_avg_count;
static int current_dev;

/**
 * @brief           Return the device name (string) given its ID.
 * @return          device name (string) on success, NULL in case of error.
 * @param[in]       dev: device ID
 */
const char *pwrmon_dev_name(uint8_t dev)
{
    if (dev >= pwrmon_num_devs) {
        return NULL;
    }

    return pwrmon_devs[dev].name;
}

/**
 * @brief           Return the power rail name (string) given its ID.
 * @return          power rail name (string) on success, NULL in case of error.
 * @param[in]       dev: device ID
 * @param[in]       rail: rail ID
 */
const char *pwrmon_rail_name(uint8_t dev, uint8_t rail)
{
    if (dev >= pwrmon_num_devs || rail >= pwrmon_devs[dev].num_rails) {
        return NULL;
    }

    return pwrmon_devs[dev].rails[rail].name;
}

/**
 * @brief           Return the device ID and rail ID, given a power rail name.
 * @return          device ID & power rail ID on success, -ENODEV if not found.
 * @param[in]       name: power rail name
 * @param[out]      dev: device ID
 * @param[out]      rail: power rail ID
 */
int pwrmon_rail_id(const char *name, uint8_t *dev, uint8_t *rail)
{
    int i, j;

    for (i = 0; i < pwrmon_num_devs; i++) {
        for (j = 0; j < pwrmon_devs[i].num_rails; j++) {
            if (strcmp(pwrmon_devs[i].rails[j].name, name) == 0) {
                *dev = i;
                *rail = j;
                dbg_verbose("%s(): name=%s => device=%u rail=%u\n",
                            __func__, name, *dev, *rail);

                return 0;
            }
        }
    }

    return -ENODEV;
}

/**
 * @brief           Return the device ID given a device name.
 * @return          device ID (>= 0) on success, standard error codes otherwise.
 *                  -ENODEV if not found
 * @param[in]       name: power rail name
 * @param[out]      dev: device ID
 */
int pwrmon_device_id(const char *name, uint8_t *dev)
{
    int i;

    for (i = 0; i < pwrmon_num_devs; i++) {
        if (strcmp(pwrmon_devs[i].name, name) == 0) {
            *dev = i;
            dbg_verbose("%s(): name=%s => device=%u\n", __func__, name, *dev);

            return 0;
        }
    }

    return -ENODEV;
}

/**
 * @brief           Program I2C channel multiplexer.
 * @return          0 on success, standard error codes otherwise.
 * @param[in]       dev: device ID
 */
static int pwrmon_ina230_select(uint8_t dev)
{
    int status;

    if (dev == current_dev) {
        dbg_verbose("%s(): device already selected (%u).\n", __func__, dev);
        return 0;
    }

    status = pwrmon_do_i2c_sel(dev);
    if (status < 0) {
        return status;
    }

    /* Save current selected device */
    current_dev = dev;

    return 0;
}

/**
 * @brief           Return the INA230 I2C address given device and rail IDs.
 * @return          INA230 I2C address on success, INVALID_I2C_ADDR otherwise.
 * @param[in]       dev: device ID
 * @param[in]       rail: power rail ID
 */
static uint8_t pwrmon_i2c_addr_get(uint8_t dev, uint8_t rail)
{
    if (dev >= pwrmon_num_devs || rail >= pwrmon_devs[dev].num_rails) {
        return -EINVAL;
    }

    return pwrmon_devs[dev].rails[rail].i2c_addr;
}

/**
 * @brief           Return the power rail count of the given device.
 * @return          power rail count (>0) on success, -EINVAL otherwise.
 * @param[in]       dev: device ID
 */
int pwrmon_dev_rail_count(uint8_t dev)
{
    if (dev >= pwrmon_num_devs) {
        return -ENODEV;
    }

    return pwrmon_devs[dev].num_rails;
}

/**
 * @brief           Initialize the power measurement HW and SW library.
 *                  To be called once, before any other call to the library.
 * @return          0 on success, standard error codes otherwise.
 * @param[in]       current_lsb_uA: current measurement precision (LSB) in uA
 * @param[in]       ct: sampling conversion time to be used
 * @param[in]       avg_count: averaging sample count (>0)
 */
int pwrmon_init(uint32_t current_lsb_uA,
               ina230_conversion_time ct,
               ina230_avg_count avg_count)
{
    dbg_verbose("%s(): Initializing with options lsb=%uuA, ct=%u, avg_count=%u...\n",
                __func__, current_lsb_uA, ct, avg_count);
    /* Initialize I2C internal structs */
    i2c_dev = up_i2cinitialize(pwrmon_i2c_bus);
    if (!i2c_dev) {
        dbg_error("%s(): Failed to get I2C bus %u\n", __func__, pwrmon_i2c_bus);
        return -ENXIO;
    }

    pwrmon_current_lsb = current_lsb_uA;
    if (ct >= ina230_ct_count) {
        dbg_error("%s(): invalid conversion time! (%u)\n", __func__, ct);
        up_i2cuninitialize(i2c_dev);
        return -EINVAL;
    }
    if (avg_count >= ina230_avg_count_max) {
        dbg_error("%s(): invalid average count! (%u)\n", __func__, avg_count);
        up_i2cuninitialize(i2c_dev);
        return -EINVAL;
    }
    pwrmon_ct = ct;
    pwrmon_avg_count = avg_count;

    current_dev = -1;

    pwrmon_init_i2c_sel();

    dbg_verbose("%s(): done.\n", __func__);

    return OK;
}

/**
 * @brief           Deinitialize power measurement HW.
 *                  Last function to be called when measurements completed.
 */
void pwrmon_deinit(void)
{
    pwrmon_reset_i2c_sel();

    /* Release I2C resource */
    up_i2cuninitialize(i2c_dev);

    return;
}

/**
 * @brief           Initialize HW & SW structure to measure a given power rail.
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
 * @return          Initialized power rail device structure
 * @param[in]       dev: device ID
 * @param[in]       rail: power rail ID
 */
pwrmon_rail *pwrmon_init_rail(uint8_t dev, uint8_t rail)
{
    pwrmon_rail *pwrmon_r = NULL;
    ina230_device *ina230_dev = NULL;
    uint8_t addr;
    int ret;

    if (dev >= pwrmon_num_devs) {
        dbg_error("%s(): invalid dev! (%hhu)\n", __func__, dev);
        goto pwrmon_init_device_end;
    }

    if (pwrmon_dev_rail_count(dev) == -EINVAL) {
        dbg_error("%s(): invalid rail! (%hhu)\n", __func__, rail);
        goto pwrmon_init_device_end;
    }

    dbg_verbose("%s(%u, %u): initializing %s rail...\n",
                __func__, dev, rail, pwrmon_rail_name(dev, rail));
    /* Configure I2C Mux */
    ret = pwrmon_ina230_select(dev);
    if (ret) {
        dbg_error("%s(): failed to configure i2c mux! (%d)\n", __func__, ret);
        goto pwrmon_init_device_end;
    }
    /* Retrieve device I2C address */
    addr = pwrmon_i2c_addr_get(dev, rail);
    if (addr == INVALID_I2C_ADDR) {
        dbg_error("%s(): failed to Retrieve i2c addr!\n", __func__);
        goto pwrmon_init_device_end;
    }
    /* Init device */
    dbg_verbose("%s(): calling ina230_init() with addr=0x%02X, mohm=%u, lsb=%uuA, ct=%u, avg_count=%u, mode=%u\n",
                __func__, addr, INA230_SHUNT_VALUE, pwrmon_current_lsb, pwrmon_ct, pwrmon_avg_count, ina230_shunt_bus_cont);
    ina230_dev = ina230_init(i2c_dev, addr,
                           INA230_SHUNT_VALUE, pwrmon_current_lsb,
                           pwrmon_ct,
                           pwrmon_avg_count,
                           ina230_shunt_bus_cont);
    if (ina230_dev == NULL) {
        dbg_error("%s(): failed to init device!\n", __func__);
        goto pwrmon_init_device_end;
    }
    /* Allocating memory for structure */
    pwrmon_r = malloc(sizeof(pwrmon_rail));
    if (!pwrmon_r) {
        dbg_error("%s(): failed to alloc memory!\n", __func__);
        ina230_deinit(ina230_dev);
        goto pwrmon_init_device_end;
    }
    pwrmon_r->ina230_dev = ina230_dev;
    pwrmon_r->dev = dev;
    pwrmon_r->rail = rail;
    pwrmon_r->dev_name = pwrmon_dev_name(dev);
    pwrmon_r->rail_name = pwrmon_rail_name(dev, rail);
    dbg_verbose("%s(): %s device init done.\n",
                __func__, pwrmon_rail_name(dev, rail));

pwrmon_init_device_end:
    return pwrmon_r;
}

/**
 * @brief           Return the time between 2 measurement samples.
 * @return          time between 2 measurement samples in microseconds
 *                  0 in case of error
 * @param[in]       pwrmon_r: power rail device structure
 */
uint32_t pwrmon_get_sampling_time(pwrmon_rail *pwrmon_r)
{
    if (!pwrmon_r) {
        dbg_error("%s(): invalid pwrmon_r!\n", __func__);
        return 0;
    }

    return ina230_get_sampling_time(pwrmon_r->ina230_dev->ct,
                                    pwrmon_r->ina230_dev->count,
                                    ina230_shunt_bus_cont);
}

/**
 * @brief           Deinitialize HW & SW structure of a given power rail.
 * @param[in]       pwrmon_r: power rail device structure
 */
void pwrmon_deinit_rail(pwrmon_rail *pwrmon_r)
{
    int ret;

    if (!pwrmon_r) {
        dbg_error("%s(): invalid pwrmon_r!\n", __func__);
        return;
    }

    dbg_verbose("%s(): deinitializing %s device...\n",
                __func__, pwrmon_r->rail_name);
    /* Configure I2C Mux */
    ret = pwrmon_ina230_select(pwrmon_r->dev);
    if (ret) {
        dbg_error("%s(): failed to configure i2c mux! (%d)\n",
                  __func__, ret);
        return;
    }
    /* Deinit device */
    ina230_deinit(pwrmon_r->ina230_dev);
    /* Free memory */
    free(pwrmon_r);
    dbg_verbose("%s(): device deinit done.\n", __func__);
}

/**
 * @brief           Return sampled data of a given power rail.
 * @return          0 on success, standard error codes otherwise.
 * @param[in]       pwrmon_r: power rail device structure
 * @param[out]      m: power measurement data (voltage, current, power)
 */
int pwrmon_measure_rail(pwrmon_rail *pwrmon_r, ina230_sample *m)
{
    int ret;

    if ((!pwrmon_r) || (!m)) {
        dbg_error("%s(): NULL pointer!\n", __func__);
        return -EINVAL;
    }
    dbg_verbose("%s(): measuring %s rail...\n", __func__, pwrmon_r->rail_name);

    /* Configure I2C Mux */
    ret = pwrmon_ina230_select(pwrmon_r->dev);
    if (ret) {
        dbg_error("%s(): failed to configure i2c mux! (%d)\n",
                  __func__, ret);
        return ret;
    }
    /* Get measurement data */
    ret = ina230_get_data(pwrmon_r->ina230_dev, m);
    if (ret) {
        dbg_error("%s(): failed to retrieve measurement data! (%d)\n",
                  __func__, ret);
    } else {
        dbg_verbose("%s(): %s measurement: %duV %duA %duW\n", __func__,
                    pwrmon_r->rail_name, m->uV, m->uA, m->uW);
    }

    return ret;
}
