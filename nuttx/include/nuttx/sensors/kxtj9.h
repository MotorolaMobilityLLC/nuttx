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

#ifndef __INCLUDE_NUTTX_SENSOR_KXTJ9_H
#define __INCLUDE_NUTTX_SENSOR_KXTJ9_H

#include <stdint.h>

/* DATA CONTROL REGISTER BITS */
#define ODR0_781F       8
#define ODR1_563F       9
#define ODR3_125F       10
#define ODR6_25F        11
#define ODR12_5F        0
#define ODR25F          1
#define ODR50F          2
#define ODR100F         3
#define ODR200F         4
#define ODR400F         5
#define ODR800F         6

struct kxtj9_sensor_data
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

int kxtj9_driver_init(void);
int kxtj9_configure(uint8_t odr);
int kxtj9_enable(bool on);
int kxtj9_read_sensor_data(struct kxtj9_sensor_data *sensor_data);

#endif // __INCLUDE_NUTTX_SENSOR_KXTJ9_H

