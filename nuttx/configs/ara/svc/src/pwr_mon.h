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

#ifndef __PWR_MON_H__
#define __PWR_MON_H__

#include <stdint.h>
#include <sys/types.h>
#include <nuttx/sensors/ina230.h>

struct pwrmon_rail_ctx {
    const char *name;
    uint8_t i2c_addr;
};

struct pwrmon_dev_ctx {
    const char *name;
    /*
     * Bitmask containing the description of GPIO lines to drive in order
     * select specific power measurement devices. This is board-specific
     * and is interpreted by the board files.
     */
    uint32_t i2c_sel;
    struct pwrmon_rail_ctx rails[INA230_MAX_DEVS];
    size_t num_rails;
};

#define DEFINE_PWR_RAIL(_name, _i2c_addr)                               \
    {                                                                   \
        .name = _name,                                                  \
        .i2c_addr = _i2c_addr,                                          \
    }

typedef struct {
    ina230_device *ina230_dev;
    uint8_t dev;
    uint8_t rail;
    const char *dev_name;
    const char *rail_name;
} pwrmon_rail;

extern const struct pwrmon_dev_ctx pwrmon_devs[];
extern const size_t pwrmon_num_devs;
extern const int pwrmon_i2c_bus;

int pwrmon_init(uint32_t current_lsb_uA,
               ina230_conversion_time ct,
               ina230_avg_count avg_count);
pwrmon_rail *pwrmon_init_rail(uint8_t dev, uint8_t rail);
uint32_t pwrmon_get_sampling_time(pwrmon_rail *pwrmon_r);
int pwrmon_measure_rail(pwrmon_rail *pwrmon_dev, ina230_sample *m);
const char *pwrmon_dev_name(uint8_t dev);
const char *pwrmon_rail_name(uint8_t dev, uint8_t rail);
int pwrmon_device_id(const char *name, uint8_t *dev);
int pwrmon_rail_id(const char *name, uint8_t *dev, uint8_t *rail);
int pwrmon_dev_rail_count(uint8_t dev);
void pwrmon_deinit_rail(pwrmon_rail *pwrmon_dev);
void pwrmon_deinit(void);

/*
 * These functions are board-specific and must be implemented
 * in appropriate board files.
 */

/* Should set the i2c selection gpios to their default values. */
extern void pwrmon_reset_i2c_sel(void);
/* Should set the i2c selection gpios' direction. */
extern void pwrmon_init_i2c_sel(void);
/* Should select given i2c line. */
extern int pwrmon_do_i2c_sel(uint8_t dev);

#endif
