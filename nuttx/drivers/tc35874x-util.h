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
#ifndef __TC35874X_H__
#define __TC35874X_H__

#include <stdint.h>

struct tc35874x_i2c_dev_info {
    struct i2c_dev_s *i2c;
    uint16_t i2c_addr;
};

uint16_t tc35874x_read_reg2(struct tc35874x_i2c_dev_info *i2c, uint16_t regaddr);
uint32_t tc35874x_read_reg4(struct tc35874x_i2c_dev_info *i2c, uint16_t regaddr);
int tc35874x_write_reg2(struct tc35874x_i2c_dev_info *i2c, uint16_t regaddr, uint16_t data);
int tc35874x_write_reg4(struct tc35874x_i2c_dev_info *i2c, uint16_t regaddr, uint32_t data);

typedef int (*tc35874x_command_func)(struct tc35874x_i2c_dev_info *i2c, void *data);

int tc35874x_start_i2c_control_thread(struct tc35874x_i2c_dev_info *i2c_info);
int tc35874x_run_command(tc35874x_command_func func, void* data);

#endif /*  __TC35874X_H__ */
