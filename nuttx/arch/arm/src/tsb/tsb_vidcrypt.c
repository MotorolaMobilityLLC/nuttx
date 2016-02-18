/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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

#include <stdlib.h>

#include <nuttx/arch.h>
#include <arch/tsb/cdsi.h>
#include <arch/tsb/vidcrypt.h>

#include "chip.h"
#include "up_arch.h"

#define TSB_VIDCRYPT_CONFIG       0x00000000
    #define TSB_VIDCRYPT_CONFIG_CDSI0_MASK  0x3
    #define TSB_VIDCRYPT_CONFIG_CDSI0_SHIFT 0
    #define TSB_VIDCRYPT_CONFIG_CDSI1_MASK  0xc
    #define TSB_VIDCRYPT_CONFIG_CDSI1_SHIFT 2
    #define TSB_VIDCRYPT_CONFIG_TX          0x0
    #define TSB_VIDCRYPT_CONFIG_RX          0x1
#define TSB_VIDCRYPT_ENABLE       0x00000004
    #define TSB_VIDCRYPT_ENABLE_CDSI0_MASK  0x1
    #define TSB_VIDCRYPT_ENABLE_CDSI0_SHIFT 0
    #define TSB_VIDCRYPT_ENABLE_CDSI1_MASK  0x4
    #define TSB_VIDCRYPT_ENABLE_CDSI1_SHIFT 2
    #define TSB_VIDCRYPT_ENABLE_EN          1
#define TSB_VIDCRYPT_DISABLE      0x00000008

static uint32_t tsb_vidcrypt_read(uint32_t offset)
{
    return getreg32(VIDCRYPT_BASE + offset);
}

static void tsb_vidcrypt_write(uint32_t offset, uint32_t v)
{
    putreg32(v, VIDCRYPT_BASE + offset);
}

int tsb_vidcrypt_set_mode(uint32_t cdsi, uint32_t tx)
{
    uint32_t direction = (tx ? TSB_VIDCRYPT_CONFIG_TX : TSB_VIDCRYPT_CONFIG_RX);
    uint32_t shift = (cdsi == TSB_CDSI0 ? TSB_VIDCRYPT_CONFIG_CDSI0_SHIFT : TSB_VIDCRYPT_CONFIG_CDSI1_SHIFT);
    uint32_t mask = (cdsi == TSB_CDSI0 ? TSB_VIDCRYPT_CONFIG_CDSI0_MASK : TSB_VIDCRYPT_CONFIG_CDSI1_MASK);

    uint32_t value = tsb_vidcrypt_read(TSB_VIDCRYPT_CONFIG);

    value &= ~mask;
    value |= (direction << shift);

    tsb_vidcrypt_write(TSB_VIDCRYPT_CONFIG, value);
    return 0;
}

int tsb_vidcrypt_enable(uint32_t cdsi)
{
    uint32_t value = tsb_vidcrypt_read(TSB_VIDCRYPT_ENABLE);

    uint32_t shift = (cdsi == TSB_CDSI0 ? TSB_VIDCRYPT_ENABLE_CDSI0_SHIFT : TSB_VIDCRYPT_ENABLE_CDSI1_SHIFT);
    uint32_t mask = (cdsi == TSB_CDSI0 ? TSB_VIDCRYPT_ENABLE_CDSI0_MASK : TSB_VIDCRYPT_ENABLE_CDSI1_MASK);

    value &= ~mask;
    value |= (TSB_VIDCRYPT_ENABLE_EN << shift);

    tsb_vidcrypt_write(TSB_VIDCRYPT_ENABLE, value);

    return 0;
}

int tsb_vidcrypt_disable(uint32_t cdsi)
{
    uint32_t value = tsb_vidcrypt_read(TSB_VIDCRYPT_DISABLE);

    uint32_t shift = (cdsi == TSB_CDSI0 ? TSB_VIDCRYPT_ENABLE_CDSI0_SHIFT : TSB_VIDCRYPT_ENABLE_CDSI1_SHIFT);
    uint32_t mask = (cdsi == TSB_CDSI0 ? TSB_VIDCRYPT_ENABLE_CDSI0_MASK : TSB_VIDCRYPT_ENABLE_CDSI1_MASK);

    value &= ~mask;
    value |= (TSB_VIDCRYPT_ENABLE_EN << shift);

    tsb_vidcrypt_write(TSB_VIDCRYPT_DISABLE, value);

    return 0;
}
