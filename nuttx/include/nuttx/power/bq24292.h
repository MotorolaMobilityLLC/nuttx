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

#ifndef __INCLUDE_NUTTX_POWER_BQ24292_H
#define __INCLUDE_NUTTX_POWER_BQ24292_H

#include <stdint.h>

enum bq24292_event {
    POWER_GOOD,     /* Good power input source */
    POWER_NOT_GOOD,
    BOOST_FAULT,    /* VBUS over-current or over-voltage in boost mode */
};

typedef void (*bq24292_callback)(enum bq24292_event event, void *arg);

int bq24292_reg_read(uint8_t reg);
int bq24292_reg_write(uint8_t reg, uint8_t val);
int bq24292_reg_modify(uint8_t reg, uint8_t mask, uint8_t set);
int bq24292_driver_init(int16_t int_n, int16_t pg_n);
int bq24292_configure(void);
int bq24292_register_callback(bq24292_callback cb, void *arg);

enum chg {
    BQ24292_CHG_OFF,
    BQ24292_CHG_BATTERY,
    BQ24292_OTG_500MA,
    BQ24292_OTG_1300MA
};
int bq24292_set_chg(enum chg config);
int bq24292_set_input_current_limit(int limit); // in mA
int bq24292_set_input_voltage_limit(int limit); // in mV
int bq24292_set_charge_current_limit(int limit); // in mA
int bq24292_set_charge_voltage_limit(int limit); // in mV

#endif // __INCLUDE_NUTTX_POWER_BQ24292_H
