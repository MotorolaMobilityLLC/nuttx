/**
 * Copyright (c) 2014-2015 Google Inc.
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
 *
 * @brief LG4892 LCD Panel & Touchscreen Driver
 */

#ifndef _LG4892_H_
#define _LG4892_H_

#include <stdint.h>

/* Touchscreen controller boot modes */
#define LG4892_TS_BOOT_MODE_BOOTLOADER          0x11
#define LG4892_TS_BOOT_MODE_APPLICATION         0x22
#define LG4892_TS_BOOT_MODE_VERIFYING           0x33

int lg4892_gpio_init(void);
int lg4892_enable(bool enable);

int lg4892_ts_init(uint8_t i2cbus);
int lg4892_ts_get_boot_mode(uint8_t *mode);
int lg4892_ts_get_operation_mode(uint8_t *mode);
int lg4892_ts_get_fw_version(uint8_t *app_maj, uint8_t *app_min,
                             uint8_t *core_maj, uint8_t *core_min);
int lg4892_ts_get_bin_fw_version(uint8_t *app_maj, uint8_t *app_min,
                                 uint8_t *core_maj, uint8_t *core_min);
int lg4892_ts_firmware_update(void);
int lg4892_ts_get_row_col(uint8_t *row, uint8_t *col);
int lg4892_ts_get_ctrl_state(uint8_t *ctrl_state);
int lg4892_ts_get_resolution(uint16_t *x, uint16_t *y);
int lg4892_ts_get_contact_threshold(uint8_t *thres);
int lg4892_ts_get_events(uint8_t *buf, uint8_t *sz);
void lg4892_ts_close(void);

#endif
