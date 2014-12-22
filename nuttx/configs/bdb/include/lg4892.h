/**
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 * @author Patrick Titiano
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
