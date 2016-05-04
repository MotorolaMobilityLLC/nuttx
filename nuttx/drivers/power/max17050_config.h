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

#ifndef __DRIVERS_POWER_MAX17050_CONFIG_H
#define __DRIVERS_POWER_MAX17050_CONFIG_H

#include <stdint.h>

#define MAX17050_CELL_DATA_SIZE     48

// Data programmed to the fuel gauge after its POR
struct max17050_config {
    uint16_t version;
    // Application specific settings
    uint16_t config;
    uint16_t relax_cfg;
    uint16_t filter_cfg;
    uint16_t learn_cfg;
    uint16_t full_soc_thr;
    // Thermistor attributes
    uint16_t tgain;
    uint16_t toff;
    // Sense resistor
    uint16_t sns_resistor;  // in uOhm
    // Battery attributes
    uint16_t qr_table_00;
    uint16_t qr_table_10;
    uint16_t qr_table_20;
    uint16_t qr_table_30;
    uint16_t rcomp0;
    uint16_t tempco;
    uint16_t capacity;
    uint16_t vf_fullcap;
    uint16_t ichg_term;
    uint32_t voltage_max_design; // in uV units
#ifdef CONFIG_BATTERY_MAX17050_EMPTY_CFG
    uint16_t iavg_empty;
    uint16_t v_empty;
#endif
    int cell_data[MAX17050_CELL_DATA_SIZE];
};

#endif  /* define __DRIVERS_POWER_MAX17050_CONFIG_H */
