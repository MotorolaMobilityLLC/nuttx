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

#include <stdint.h>

#include "bq24292_config.h"

// Charging Configurations
#ifdef CONFIG_CHARGER_BQ24292_SAMPLE_CFG
struct bq24292_config bq24292_cfg[] = {
    // reg  mask  set
    { 0x00, 0xFF, 0x3D },   // EN_HIZ = 0, VINDMP = 4.44V, IINLIM = 1.5A
    { 0x01, 0x0F, 0x0B },   // SYS_MIN = 3.5V, BOOST_LIM = 1.3A
    { 0x02, 0xFF, 0x5C },   // ICHG = 1472mA, FORCE_20PCT = 0
    { 0x03, 0xFF, 0x11 },   // IPRECHG = 256mA, ITERM = 256mA
    { 0x04, 0xFF, 0xAC },   // VREG = 4192 mV, BATLOWV = 2.8V, VRECHG = 100 mV
    { 0x05, 0xFF, 0x80 },   // EN_TERM = 1, TERM_STAT = 0, WATCHDOG = DISABLE, CHG_TIMER = 8hrs
    { 0x07, 0x4F, 0x4B },   // TMR2X_EN = 1, CHGRG_FAULT_INT = 1, BAT_FAULT_INT = 1
};

int bq24292_cfg_size = sizeof(bq24292_cfg) / sizeof(struct bq24292_config);
#else
// Place charging configurations for actual batteries here
#endif