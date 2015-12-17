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
    .version        = 0xFF00,
    // Application specific settings
    .config         = 0x0210,   // Enable Temperature channel with updates every 1.4S
    .relax_cfg      = 0x023B,   // Load = 5 mA, dV = 3.75 mV, dT = 6 mins
    .filter_cfg     = 0x87A4,
    .learn_cfg      = 0x2606,
    .full_soc_thr   = 0x5F00,
    // Thermistor attributes
    .tgain          = 0xE71C,
    .toff           = 0x251A,
    // Sense resistor
    .sns_resistor   = 10000,    // in uOhm
    // Battery attributes
    .qr_table_00    = 0x1B94,
    .qr_table_10    = 0x0E94,
    .qr_table_20    = 0x0594,
    .qr_table_30    = 0x0294,
    .rcomp0         = 0x0052,
    .tempco         = 0x333A,
     // Capacity is 0.5mAh with 10mOhm sense resistor
    .capacity       = 606,      // 303 mA
    .vf_fullcap     = 606,      // 303 mA
    .ichg_term      = 0x0060,   // 15 mA
    .voltage_max_design = 4200000, // uV units
    .cell_data = {
        0x9cf0, 0xa310, 0xb180, 0xb360, 0xb6a0, 0xb6e0, 0xba10, 0xbb10,
        0xbbd0, 0xbc10, 0xc200, 0xc250, 0xc7c0, 0xc820, 0xd050, 0xd710,
        0x0140, 0x0110, 0x0210, 0x02c0, 0x1000, 0x0a00, 0x06c0, 0x31a0,
        0x0af0, 0x1170, 0x0cb0, 0x08d0, 0x0aa0, 0x0880, 0x07b0, 0x07b0,
        0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
        0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100
    }
};
