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
 * @file    configs/ara/svc/src/ina230.h
 * @brief   TI INA230 Driver
 * @author  Patrick Titiano
 */

#ifndef __INA_230_H__
#define __INA_230_H__

#include <stdint.h>
#include <pwr_measure.h>
#include <nuttx/i2c.h>

typedef enum {
    ina230_ct_140us = 0x0,          /* 140us */
    ina230_ct_204us = 0x1,          /* 204us */
    ina230_ct_332us = 0x2,          /* 332us */
    ina230_ct_588us = 0x3,          /* 588us */
    ina230_ct_1_1ms = 0x4,          /* 1.1ms */
    ina230_ct_2_116ms = 0x5,        /* 2.116ms */
    ina230_ct_4_156ms = 0x6,        /* 4.156ms */
    ina230_ct_8_244ms = 0x7,        /* 8.244ms */
    ina230_ct_count = 0x8
} ina230_conversion_time;

typedef enum {
    ina230_avg_count_1 = 0x0,
    ina230_avg_count_4 = 0x1,
    ina230_avg_count_16 = 0x2,
    ina230_avg_count_64 = 0x3,
    ina230_avg_count_128 = 0x4,
    ina230_avg_count_256 = 0x5,
    ina230_avg_count_512 = 0x6,
    ina230_avg_count_1024 = 0x7,
    ina230_avg_count_max = 0x8
} ina230_avg_count;

typedef enum {
    ina230_power_down = 0x0,        /* Power-Down */
    ina230_shunt_trig = 0x1,        /* Shunt Voltage, triggered */
    ina230_bus_trig = 0x2,          /* Bus Voltage, triggered */
    ina230_shunt_bus_trig = 0x3,    /* Shunt and Bus, triggered */
    ina230_shunt_cont = 0x5,        /* Shunt Voltage, continuous */
    ina230_bus_cont = 0x6,          /* Bus Voltage, continuous */
    ina230_shunt_bus_cont = 0x7,    /* Shunt and Bus, continuous */
    ina230_power_mode_count = 0x8
} ina230_power_mode;

typedef struct
{
    struct i2c_dev_s *i2c_dev;  /* Nuttx I2C bus handler */
    uint8_t addr;               /* I2C device address */
    uint32_t mohm;              /* Shunt resistor value in milliohms */
    uint32_t current_lsb;       /* Current LSB in microamps */
    ina230_power_mode mode;     /* Acquisition mode */
    ina230_conversion_time ct;  /* Conversion time */
    ina230_avg_count count;     /* Average sample count */
} ina230_device;

ina230_device *ina230_init(struct i2c_dev_s *dev, uint8_t addr,
                           uint32_t mohm, uint32_t current_lsb,
                           ina230_conversion_time ct,
                           ina230_avg_count count,
                           ina230_power_mode mode);
uint32_t ina230_ct_to_int(ina230_conversion_time ct);
uint32_t ina230_avg_count_to_int(ina230_avg_count avg_count);
uint32_t ina230_get_sampling_time(ina230_conversion_time ct,
                                 ina230_avg_count avg_count,
                                 ina230_power_mode mode);
int ina230_get_data(ina230_device *dev, pwr_measure *m);
int ina230_deinit(ina230_device *dev);

#endif
