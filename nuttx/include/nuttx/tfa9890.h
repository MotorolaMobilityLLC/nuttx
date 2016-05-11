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

#ifndef __INCLUDE_TFA9890_H
#define __INCLUDE_TFA9890_H

#include <nuttx/util.h>

#define TFA9890_MAX_I2C_SIZE    252

enum tfa9890_lowerhalf_cfg_cmd {
    TFA9890_VOLUME,
    TFA9890_USECASE,
    TFA9890_SYS_VOL,
};

struct tfa9890_registers {
    uint16_t reg;
    uint16_t val;
};

struct tfa9890_lowerhalf_s;
struct tfa9890_caps_s {
    enum tfa9890_lowerhalf_cfg_cmd type;
    int param;
};

struct tfa9890_ops_s {
    int (*configure)(FAR struct tfa9890_lowerhalf_s *dev,
                  FAR const struct tfa9890_caps_s *caps);
    int (*stop)(FAR struct tfa9890_lowerhalf_s *dev);
    int (*start)(FAR struct tfa9890_lowerhalf_s *dev);
};

struct tfa9890_lowerhalf_s {
    const struct tfa9890_ops_s *ops;
    void *priv;
};

/*******************  Registers ******************/
#define TFA9890_REG_SYSTEM_STATUS        0x00
#define TFA9890_REG_BATTERY_STATUS       0x01
#define TFA9890_REG_TEMP_STATUS          0x02
#define TFA9890_REG_ID                   0x03
#define TFA9890_REG_I2S_CTRL             0x04
#define TFA9890_REG_BS_CTRL              0x05
#define TFA9890_REG_VOLUME_CTRL          0x06
#define TFA9890_REG_DC_TO_DC_CTRL        0x07
#define TFA9890_REG_SPEAKER_CTRL         0x08
#define TFA9890_REG_SYSTEM_CTRL_1        0x09
#define TFA9890_REG_SYSTEM_CTRL_2        0x0A
#define TFA9890_REG_MTP_KEY              0x0B
#define TFA9890_REG_INTERRUPT_CTRL       0x0F
#define TFA9890_PWM_CTL_REG              0x41
#define TFA9890_REG_CURRENT_SENSE        0x47
#define TFA9890_CURRT_SNS2_REG           0x48
#define TFA9890_REG_CLIP_SENSE           0x49
#define TFA9890_DEM_CTL_REG              0x4E
#define TFA9890_REG_I2C_TO_MTP           0x62
#define TFA9890_REG_COOLFLUX_CTRL        0x70
#define TFA9890_REG_CF_MEM_ADDR          0x71
#define TFA9890_REG_CF_ACT_MEM           0x72
#define TFA9890_REG_CF_STATUS            0x73
#define TFA9890_REG_MTP                  0x80


#define TFA9890_I2SSR_8KHZ               0
#define TFA9890_I2SSR_11KHZ              1
#define TFA9890_I2SSR_12KHZ              2
#define TFA9890_I2SSR_16KHZ              3
#define TFA9890_I2SSR_22KHZ              4
#define TFA9890_I2SSR_24KHZ              5
#define TFA9890_I2SSR_32KHZ              6
#define TFA9890_I2SSR_44KHZ              7
#define TFA9890_I2SSR_48KHZ              8

#define TFA9890_I2SDOC_I2S               0
#define TFA9890_I2SDOC_DATAL1            1
#define TFA9890_I2SDOC_DATAL2            2
#define TFA9890_I2SDOC_DATAL3            3

#define TFA9890_CHSA_LEFT_CHAN           0
#define TFA9890_CHSA_RIGHT_CHAN          1
#define TFA9890_CHSA_COOLFLUX_1          2  /* default */
#define TFA9890_CHSA_COOLFLUX_2          3


/* enable I2S left channel input */
#define TFA9890_I2S_LEFT_IN              (0x1)
/* enable I2S right channel input */
#define TFA9890_I2S_RIGHT_IN             (0x2)
/* route gain info on datao pin on left channel */
#define TFA9890_DOLS_GAIN                (0x1)
/* route gain info on datao pin on right channel */
#define TFA9890_DORS_GAIN                (0x1)
/* select gain input channel for stereo config */
#define TFA9890_GAIN_IN                  (0x1)

/* Gets the speaker calibration impedance (@25 degrees celsius) */
#define TFA9890_PARAM_GET_RE0          0x85
/* Gets current LoudSpeaker Model. */
#define TFA9890_PARAM_GET_LSMODEL      0x86
#define TFA9890_PARAM_GET_STATE        0xC0
#define TFA9890_PARAM_GET_LSMODELW     0xC1
#define TFA9890_FW_PARAM_GET_STATE     0x84
#define TFA9890_PARAM_GET_FEATURE_BITS 0x85
#define TFA9890_PARAM_GET_CFGPST       0x80


/* RPC command ID's*/
#define TFA9890_PARAM_SET_LSMODEL   0x06
#define TFA9890_PARAM_SET_EQ        0x0A
#define TFA9890_PARAM_SET_PRESET    0x0D
#define TFA9890_PARAM_SET_CONFIG    0x0E
#define TFA9890_PARAM_SET_DBDRC     0x0F
#define TFA9890_PARAM_SET_AGCINS    0x10


#define TFA9890_CF_CONTROLS         0x70
#define TFA9890_CF_MAD              0x71
#define TFA9890_CF_MEM              0x72
#define TFA9890_CF_STATUS           0x73
#define TFA9890_MTP_REG             0x80
/* DSP patch header lenght */
#define TFA9890_PATCH_HEADER        0x6

/* DSP Module IDs */
#define TFA9890_DSP_MOD_FRAMEWORK           0
#define TFA9890_DSP_MOD_SPEAKERBOOST        1
#define TFA9890_DSP_MOD_BIQUADFILTERBANK    2

/* DSP Write/Read */
#define TFA9890_DSP_WRITE           0
#define TFA9890_DSP_READ            1

/* RPC Status results */
#define TFA9890_STATUS_OK           0
#define TFA9890_INVALID_MODULE_ID   2
#define TFA9890_INVALID_PARAM_ID    3
#define TFA9890_INVALID_INFO_ID     4

/* memory address to be accessed on dsp
 * (0 : Status, 1 : ID, 2 : parameters) */
#define TFA9890_CF_MAD_STATUS   0x0
#define TFA9890_CF_MAD_ID       0x1
#define TFA9890_CF_MAD_PARAM    0x2

/* DSP control Register */
#define TFA9890_CF_CTL_MEM_REQ  0x2
#define TFA9890_CF_INT_REQ      (1 << 4)
#define TFA9890_CF_ACK_REQ      (1 << 8)

/* MTP memory unlock key */
#define TFA9890_MTK_KEY     0x005A

/* Status Masks */
#define TFA9890_SYSCTRL_CONFIGURED  BIT(5)
#define TFA9890_STATUS_PLLS         BIT(1)
#define TFA9890_STATUS_CLKS         BIT(6)
#define TFA9890_STATUS_MTPB         BIT(8)
#define TFA9890_STATUS_CIMTP        BIT(11)
#define TFA9890_STATUS_MTPEX        BIT(1)
#define TFA9890_STATUS_ACS          BIT(11)
#define TFA9890_STATUS_DC2DC        BIT(4)
#define TFA9890_STATUS_AMPE         BIT(3)
#define TFA9890_STATUS_MUTE         BIT(5)
#define TFA9890_STATUS_CF           BIT(8)
#define TFA9890_STATUS_TEMP         (0x1ff)
#define TFA9890_STATUS_VDDS         BIT(0)
#define TFA9890_STATUS_AMP_SWS      BIT(12)
#define TFA9890_STATUS_SPKS         BIT(10)
#define TFA9890_STATUS_WDS          BIT(13)
#define TFA9890_STATUS_ARFS         BIT(15)

/* param masks */
#define TFA9890_MTPOTC          (0x1)

/* speaker impedance exponent */
#define TFA9890_SPKR_IMP_EXP    9

/* offsets */
#define TFA9890_RESET_OFFSET        BIT(0)
#define TFA9890_VOLUME_CTRL_OFFSET  BIT(3)
#define TFA9890_PWDN_OFFSET         (0)
#define TFA9890_AMP_OFFSET          BIT(0) | BIT(1)
#define TFA9890_SBSL_OFFSET         BIT(0) | BIT(2)
#define TFA9890_I2DOE_OFFSET        BIT(11)
#define TFA9890_I2S_IN_OFFSET       BIT(0) | BIT(1)
#define TFA9890_GAIN_IN_OFFSET      BIT(0) | BIT(2)
#define TFA9890_DORS_GAIN_OFFSET    BIT(0) | BIT(1)
#define TFA9890_DOLS_GAIN_OFFSET    0

/* DSP Firmware Size in bytes*/
#define TFA9890_SPK_FW_SIZE         424
#define TFA9890_CFG_FW_SIZE         166
#define TFA9890_PST_FW_SIZE         88
#define TFA9890_COEFF_FW_SIZE       181
#define TFA9890_DEV_STATE_SIZE      24
#define TFA9890_SPK_EX_FW_SIZE      424

/* use case eq profiles */
#define TFA9890_EQ_MUSIC                        BIT(0)
#define TFA9890_EQ_VOICE                        BIT(1)
#define TFA9890_EQ_LOW_LATENCY_SYSTEM_SOUNDS    BIT((2)

enum
{
    TFA9890_LEFT,
    TFA9890_RIGHT,
    TFA9890_MONO,
};

enum
{
    TFA9890_MUSIC,
    TFA9890_VOICE,
    TFA9890_RINGTONE,
};

#endif /* __INCLUDE_TFA9890_H */
