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

#include "max17050_config.h"

// Battery Configurations

// Use version MSB to identify battery and LSB as the version of the config for
// this battery. Incrementing version number forces re-programming of config
// data, otherwise data is only programmed after Power-On reset of the device.
const struct max17050_config max17050_cfg = {
    .version        = 0x0200,
    // Application specific settings
    .config         = 0x0210,   // Enable Temperature channel with updates every 1.4S
    .relax_cfg      = 0x023B,   // Load = 5 mA, dV = 3.75 mV, dT = 6 mins
    .filter_cfg     = 0xCEA4,
    .learn_cfg      = 0x2602,
    .full_soc_thr   = 0x5A00,
    // Thermistor attributes
    .tgain          = 0xE71C,   // 2s complement in 1/64 C units (Beta = 3974)
    .toff           = 0x251A,   // in 1/128 C units = 74.08C
    // Sense resistor
    .sns_resistor   = 10000,    // in uOhm
    // Battery attributes
    .qr_table_00    = 0x2280,
    .qr_table_10    = 0x1000,
    .qr_table_20    = 0x0681,
    .qr_table_30    = 0x0682,
    .rcomp0         = 0x0070,
    .tempco         = 0x263D,
    // capacity units 5.0uVh/Rsense (i.e. 0.5mAh for Rsense of 10mOhm)
    .capacity       = 0x1158,   // 4440 = 2220 mAh for 10mOhm
    .vf_fullcap     = 0x1158,   // 4440 = 2220 mAh for 10mOhm
    .ichg_term      = 0x058D,   // 222mA (units of 1.5625uV/Rsense)
    .voltage_max_design = 4350000, // uV units
    .cell_data = {
       0x9760, 0xa510, 0xb100, 0xb600, 0xb7a0, 0xb900, 0xba70, 0xbc70,
       0xbde0, 0xbfc0, 0xc250, 0xc510, 0xc990, 0xcea0, 0xd040, 0xd750,
       0x0060, 0x0120, 0x0200, 0x0710, 0x0e80, 0x0df0, 0x1430, 0x1bd0,
       0x1520, 0x0d70, 0x0950, 0x08e0, 0x0800, 0x0780, 0x06b0, 0x01e0,
       0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
       0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100
    }
};
