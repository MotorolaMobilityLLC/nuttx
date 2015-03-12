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
 * @file    configs/ara/svc/src/tca6424.h
 * @brief   TCA6424 GPIO Expander Driver
 * @author  Patrick Titiano
 */

#ifndef _TSB_TCA6424_H_
#define _TSB_TCA6424_H_

#include <stdint.h>
#include <nuttx/i2c.h>

typedef struct
{
    struct i2c_dev_s *i2c_dev;  /* Nuttx I2C bus handler */
    uint8_t addr;               /* I2C device address */
} tca6424_device;


tca6424_device *tca6424_init(struct i2c_dev_s *i2c_dev, uint8_t addr);

int tca6424_set_direction_in(tca6424_device *dev, uint8_t which);
int tca6424_set_direction_out(tca6424_device *dev, uint8_t which);
int tca6424_get_direction(tca6424_device *dev, uint8_t which);
int tca6424_set_polarity_inverted(tca6424_device *dev,
                                  uint8_t which, uint8_t inverted);
int tca6424_get_polarity_inverted(tca6424_device *dev, uint8_t which);
int tca6424_set(tca6424_device *dev, uint8_t which, uint8_t val);
int tca6424_get(tca6424_device *dev, uint8_t which);

void tca6424_deinit(tca6424_device *dev);

#endif
