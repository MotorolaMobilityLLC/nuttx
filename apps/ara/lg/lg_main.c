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
 * @brief LG4982 LCD Panel and Touchscreen Test Application
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <nuttx/i2c.h>
#include <arch/board/lg4892.h>

#define APB2_I2C0               0

int lg_help(void)
{
    printf("\ninvalid(s) argument(s)!\n");
    printf("Syntax:\n");
    printf("\tlg [--fwupdate]\n");
    printf("\tBy default touchscreen firmware will not be updated.\n");
    printf("\tUse optional \"--fwupdate\" to allow firmware update.\n\n");

    return -EINVAL;
}

int lg_fwupdate(void)
{
    int ret;
    uint8_t app_maj, app_min, core_maj, core_min;
    uint8_t bin_app_maj, bin_app_min, bin_core_maj, bin_core_min;

    ret = lg4892_ts_get_bin_fw_version(&bin_app_maj, &bin_app_min,
                                       &bin_core_maj, &bin_core_min);
    if (ret != 0) {
        printf("Failed to get BIN firmware version! (%d)\n", ret);
        return ret;
    }

    ret = lg4892_ts_firmware_update();
    if (ret != 0) {
        printf("Failed to update TS firmware! (%d)\n", ret);
        return ret;
    }

    ret = lg4892_ts_get_fw_version(&app_maj, &app_min, &core_maj, &core_min);
    if (ret != 0) {
        printf("Failed to get new TS firmware version! (%d)\n", ret);
        return ret;
    }
    printf("New TS Firmware version: 0x%02X 0x%02X 0x%02X 0x%02X\n",
        app_maj, app_min, core_maj, core_min);

    if ((app_maj == app_maj) ||
        (app_min == app_min) ||
        (core_maj == core_maj) ||
        (core_min == bin_core_min)) {
        printf("TS firmware update succeeded.\n");
    } else {
        printf("TS firmware version mismatch, update failed!\n");
        return -EIO;
    }

    return 0;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int lg_main(int argc, char *argv[])
#endif
{
    int ret;
    uint8_t row, col;
    uint8_t app_maj, app_min, core_maj, core_min;
    uint8_t bin_app_maj, bin_app_min, bin_core_maj, bin_core_min;
    uint8_t ctrl_state;
    uint16_t x, y;
    uint8_t mode;
    uint8_t thres;
    uint8_t *buf = NULL;
    uint8_t sz;
    uint8_t i = 0;
    uint8_t fwupdate = 0;

    if (argc == 1) {
        fwupdate = 0;
    } else if (argc == 2) {
        if (strcmp(argv[1], "--fwupdate") == 0) {
            fwupdate = 1;
        } else {
            return lg_help();
        }
    } else {
        return lg_help();
    }

    lg4892_gpio_init();
    lg4892_enable(1);

    /*
     * TODO: complete LCD Panel initialization
     *   - Enable PWM
     *   - DSI init
     */

    ret = lg4892_ts_init(APB2_I2C0);
    if (ret != 0) {
        printf("TS init failed! (%d)\n", ret);
        lg4892_ts_close();
        return ret;
    }
    printf("TS initialized.\n");

    ret = lg4892_ts_get_bin_fw_version(&bin_app_maj, &bin_app_min,
                                       &bin_core_maj, &bin_core_min);
    if (ret != 0) {
        printf("Failed to get BIN firmware version! (%d)\n", ret);
        lg4892_ts_close();
        return ret;
    }
    printf("TS BIN firmware version: 0x%02X 0x%02X 0x%02X 0x%02X\n",
        bin_app_maj, bin_app_min, bin_core_maj, bin_core_min);

    ret = lg4892_ts_get_fw_version(&app_maj, &app_min, &core_maj, &core_min);
    if (ret != 0) {
        printf("Failed to get TS firmware version! (%d)\n", ret);
        lg4892_ts_close();
        return ret;
    }
    printf("TS Firmware version: 0x%02X 0x%02X 0x%02X 0x%02X\n",
        app_maj, app_min, core_maj, core_min);
    if ((app_maj != bin_app_maj) || (app_min != bin_app_min) ||
        (core_maj != bin_core_maj) || (core_min != bin_core_min)) {
        if (fwupdate == 1) {
            printf("TS firmware version does not match BIN version, updating it...\n");
            ret = lg_fwupdate();
            if (ret != 0) {
                printf("Failed to update TS firmware! (%d)\n", ret);
                lg4892_ts_close();
                return ret;
            }
        } else {
            printf("WARNING: TS firmware version does not match BIN version.\n");
        }
    } else {
        printf("TS firmware version matches BIN version, no need to update.\n");
    }

    ret = lg4892_ts_get_boot_mode(&mode);
    if (ret != 0) {
        printf("Failed to retrieve boot mode! (%d)\n", ret);
    } else {
        switch (mode) {
        case LG4892_TS_BOOT_MODE_BOOTLOADER:
            printf("TS BOOT mode: BOOTLOADER\n");
            break;
        case LG4892_TS_BOOT_MODE_APPLICATION:
            printf("TS BOOT mode: APPLICATION\n");
            break;
        case LG4892_TS_BOOT_MODE_VERIFYING:
            printf("TS BOOT mode: VERIFYING\n");
            break;
        default:
            printf("TS BOOT mode: unknown! (0x%02X)\n", mode);
        }
    }

    ret = lg4892_ts_get_ctrl_state(&ctrl_state);
    if (ret != 0) {
        printf("Failed to retrieve TS control state! (%d)\n", ret);
    } else {
        printf("TS control state: 0x%02X\n", ctrl_state);
    }

    ret = lg4892_ts_get_resolution(&x, &y);
    if (ret != 0) {
        printf("Failed to retrieve TS resolution! (%d)\n", ret);
    } else {
        printf("TS resolution: X=%u Y=%u\n", x, y);
    }

    ret = lg4892_ts_get_contact_threshold(&thres);
    if (ret != 0) {
        printf("Failed to retrieve TS threshold! (%d)\n", ret);
    } else {
        printf("TS threshold: %u\n", thres);
    }

    ret = lg4892_ts_get_operation_mode(&mode);
    if (ret != 0) {
        printf("Failed to retrieve TS operation mode! (%d)\n", ret);
    } else {
        printf("TS operation mode: 0x%02X\n", mode);
    }

    ret = lg4892_ts_get_row_col(&row, &col);
    if (ret != 0) {
        printf("Failed to get TS row and column number! (%d)\n", ret);
        lg4892_ts_close();
        return ret;
    }
    printf("TS row: %u\nTS column: %u\n", row, col);

    ret = lg4892_ts_get_events(buf, &sz);
    if (ret < 0) {
        printf("Failed to retrieve TS events! (%d)\n", ret);
    } else if (ret == 0) {
        printf("There was no events to retrieve.\n");
    } else {
        printf("TS events retrieved (%d): ", sz);
        for (i = 0; i < sz; i++) {
            printf("0X%02X ", buf[i]);
        }
        printf("\n");
        free(buf);
    }

    lg4892_ts_close();
    lg4892_enable(0);

    return 0;
}
