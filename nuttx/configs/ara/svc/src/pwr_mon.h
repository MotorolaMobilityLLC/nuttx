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
#include <nuttx/sensors/ina230.h>

#define DEV_MAX_RAIL_COUNT          8

#define DEV_NAME_MAX_LENGTH         8
#define RAIL_NAME_MAX_LENGTH        20

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
    struct pwrmon_rail_ctx rails[DEV_MAX_RAIL_COUNT];
    size_t num_rails;
};

#define DEFINE_PWR_RAIL(_name, _i2c_addr)                               \
    {                                                                   \
        .name = _name,                                                  \
        .i2c_addr = _i2c_addr,                                          \
    }

typedef enum {
    DEV_SW,
    DEV_APB1,
    DEV_APB2,
    DEV_APB3,
    DEV_GPB1,
    DEV_GPB2,
#ifdef CONFIG_ARCH_BOARD_ARA_SDB_SVC
    DEV_SVC,
#endif
    DEV_COUNT,
} device;

typedef enum {
    VSW_1P1_PLL,
    VSW_1P1_CORE,
    VSW_1P8_UNIPRO,
    VSW_1P8_IO,
    VSW_COUNT
} sw_pwr_rail;

typedef enum {
    VAPB_1P1_CORE,
    VAPB_1P1_PLL1,
    VAPB_1P2_CDSI_PLL,
    VAPB_1P2_CDSI,
    VAPB_1P2_HSIC,
    VAPB_1P8_UNIPRO,
    VAPB_1P8_IO,
    VAPB_1P1_PLL2,
    VAPB_COUNT
} apb_pwr_rail;

typedef enum {
    VGPB_1P1_CORE,
    VGPB_1P1_PLL1,
    VGPB_SDIO,
    VGPB_1P2_HSIC,
    VGPB_1P8_UNIPRO,
    VGPB_1P8_IO,
    VGPB_1P1_PLL2,
    VGPB_COUNT
} gpb_pwr_rail;

#ifdef CONFIG_ARCH_BOARD_ARA_SDB_SVC
typedef enum {
    SVC_1P8_VDD,
    SVC_1P8_VBAT,
    SVC_1P8_VDDA,
    SVC_1P8_VREF,
    SVC_COUNT
} svc_pwr_rail;
#endif

typedef struct {
    ina230_device *ina230_dev;
    uint8_t dev;
    uint8_t rail;
    const char *dev_name;
    const char *rail_name;
} arapm_rail;

int arapm_init(uint32_t current_lsb_uA,
               ina230_conversion_time ct,
               ina230_avg_count avg_count);
arapm_rail *arapm_init_rail(uint8_t dev, uint8_t rail);
uint32_t arapm_get_sampling_time(arapm_rail *arapm_r);
int arapm_measure_rail(arapm_rail *arapm_dev, ina230_sample *m);
const char *arapm_dev_name(uint8_t dev);
const char *arapm_rail_name(uint8_t dev, uint8_t rail);
int arapm_device_id(const char *name, uint8_t *dev);
int arapm_rail_id(const char *name, uint8_t *dev, uint8_t *rail);
int arapm_dev_rail_count(uint8_t dev);
void arapm_deinit_rail(arapm_rail *arapm_dev);
void arapm_deinit(void);

#endif
