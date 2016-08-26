/****************************************************************************
 *
 *   Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_TFA9890_REGISTERS_H
#define __INCLUDE_TFA9890_REGISTERS_H

#include <nuttx/util.h>
#include <nuttx/tfa9890.h>

#define TFA9890_MUSIC_VOL_STEP    15
#define TFA9890_VOICE_VOL_STEP    15
#define TFA9890_RINGTONE_VOL_STEP 15

const struct tfa9890_registers tfa9890_registers_init_list[] = {
    {
        .reg = TFA9890_REG_I2S_CTRL,
        .val = 0x889b,
    },
    {
        .reg = TFA9890_REG_BS_CTRL,
        .val = 0x138e,
    },
    {
        .reg = TFA9890_REG_VOLUME_CTRL,
        .val = 0x000f,
    },
    {
        .reg = TFA9890_REG_DC_TO_DC_CTRL,
        .val = 0x8ff6,
    },
    {
        .reg = TFA9890_REG_SYSTEM_CTRL_1,
        .val = 0x827d,
    },
    {
        .reg = TFA9890_REG_SYSTEM_CTRL_2,
        .val = 0x38d2,
    },
    {
        .reg = TFA9890_PWM_CTL_REG,
        .val = 0x308,
    },
    {
        .reg = TFA9890_REG_CURRENT_SENSE,
        .val = 0x7be1,
    },
    {
        .reg = TFA9890_CURRT_SNS2_REG,
        .val = 0x340,
    },
    {
        .reg = TFA9890_REG_CLIP_SENSE,
        .val = 0xAD93,
    },
    {
        .reg = TFA9890_DEM_CTL_REG,
        .val = 0x00,
    },
};

/* Presets index tables,  units are millibles */
const int tfa9890_presets_music[TFA9890_MUSIC_VOL_STEP] = {
    0, -150, -300, -450, -600, -750, -1050, -1350, -1650, -1950, -2250, -2750, -3250, -4750, -12750
    };

const int tfa9890_presets_voice[TFA9890_VOICE_VOL_STEP] = {
    0, -150, -300, -450, -600, -750, -1050, -1350, -1650, -1950, -2250, -2750, -3250, -4750, -12750
    };

const int tfa9890_presets_ringtone[TFA9890_RINGTONE_VOL_STEP] = {
    0, -150, -300, -450, -600, -750, -1050, -1350, -1650, -1950, -2250, -2750, -3250, -4750, -12750
    };

#if defined (CONFIG_MODS_AUDIO_TFA9890_STEREO)
/* register initialization for stereo set up */
const struct tfa9890_registers tfa9890_registers_left_init[] = {
    {
        /* left channel audio input and gain input on right channel */
        .reg = TFA9890_REG_I2S_CTRL,
        .val = 0x88ab,

    },
    {
        /* DATA0 configuration: gain on left channel channel and dsp
         * output on right channel.
         */
        .reg = TFA9890_REG_SYSTEM_CTRL_2,
        .val = 0x38d1,
    },
};

const struct tfa9890_registers tfa9890_registers_right_init[] = {
    {
       /* right channel audio input and gain input on left channel */
        .reg = TFA9890_REG_I2S_CTRL,
        .val = 0x8893,
    },
    {
        /* DATA0 configuration: gain on right channel channel and dsp
         * output on left channel.
         */
        .reg = TFA9890_REG_SYSTEM_CTRL_2,
        .val = 0x38ca,
    },
};
#endif

#endif
