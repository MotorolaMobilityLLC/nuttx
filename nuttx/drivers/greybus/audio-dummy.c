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
#include <string.h>
#include <errno.h>

#include <arch/byteorder.h>
#include <nuttx/audio/audio.h>
#include <nuttx/config.h>
#include <nuttx/device.h>
#include <nuttx/device_audio.h>
#include <nuttx/device_i2s.h>
#include <nuttx/greybus/types.h>
#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/utils.h>

#include "audio-gb.h"
#include "i2s-gb.h"


#define AUD_DEV_DUMMY_SUPPORTED_PLAYBACK_USE_CASES (GB_AUDIO_PLAYBACK_MUSIC_USE_CASE | \
                           GB_AUDIO_PLAYBACK_VOICE_CALL_SPKR_USE_CASE | \
                           GB_AUDIO_PLAYBACK_RINGTONE_USE_CASE | \
                           GB_AUDIO_PLAYBACK_SONIFICATION_USE_CASE)

#define AUD_DEV_DUMMY_SUPPORTED_CAPTURE_USE_CASES  GB_AUDIO_CAPTURE_DEFAULT_USE_CASE


#define AUD_DEV_DUMMY_SUPPORTED_OUT_DEVICES    GB_AUDIO_DEVICE_OUT_LOUDSPEAKER
#define AUD_DEV_DUMMY_SUPPORTED_IN_DEVICES    (GB_AUDIO_DEVICE_IN_EC_REF | \
                                               GB_AUDIO_DEVICE_IN_MIC)

#define AUD_DEV_DUMMY_I2S_RATE_MASK    (DEVICE_I2S_PCM_RATE_48000 | \
                                    DEVICE_I2S_PCM_RATE_16000 | \
                                    DEVICE_I2S_PCM_RATE_96000)
#define AUD_DEV_DUMMY_I2S_PCM_FMT_MASK    (DEVICE_I2S_PCM_FMT_16 | DEVICE_I2S_PCM_FMT_24)
#define AUD_DEV_DUMMY_I2S_PROTOCOL_MASK    DEVICE_I2S_PROTOCOL_I2S
#define AUD_DEV_DUMMY_I2S_WCLK_PALARITY_MASK    (DEVICE_I2S_POLARITY_NORMAL | \
                                                       DEVICE_I2S_POLARITY_REVERSED)
#define AUD_DEV_DUMMY_I2S_WCLK_EDGE_MASK    (DEVICE_I2S_EDGE_FALLING | \
                                                        DEVICE_I2S_EDGE_RISING)
#define AUD_DEV_DUMMY_I2S_RXCLK_EDGE_MASK    (DEVICE_I2S_EDGE_FALLING | \
                                                        DEVICE_I2S_EDGE_RISING)
#define AUD_DEV_DUMMY_I2S_TXCLK_EDGE_MASK    (DEVICE_I2S_EDGE_FALLING | \
                                                        DEVICE_I2S_EDGE_RISING)


static const struct device_aud_pcm_config aud_dummy_pcm = {
    .rate       = AUD_DEV_DUMMY_I2S_RATE_MASK,
    .channels   = 2,
    .format     = AUD_DEV_DUMMY_I2S_PCM_FMT_MASK,
};

static const struct device_aud_i2s_config aud_dummy_i2s_dai = {
    .protocol            = AUD_DEV_DUMMY_I2S_PROTOCOL_MASK,
    .wclk_polarity       = AUD_DEV_DUMMY_I2S_WCLK_PALARITY_MASK,
    .wclk_change_edge    = AUD_DEV_DUMMY_I2S_WCLK_EDGE_MASK,
    .data_tx_edge        = AUD_DEV_DUMMY_I2S_TXCLK_EDGE_MASK,
    .data_rx_edge        = AUD_DEV_DUMMY_I2S_RXCLK_EDGE_MASK,
};

int aud_dev_dummy_open(struct device *dev)
{

    gb_debug("%s()\n", __func__);
    return 0;
}

static int aud_dev_dummy_get_supported_use_cases(struct device *dev,
                           struct device_aud_usecases *use_cases)
{
    gb_debug("%s()\n", __func__);

    use_cases->playback_usecases = AUD_DEV_DUMMY_SUPPORTED_PLAYBACK_USE_CASES;
    use_cases->capture_usecases = AUD_DEV_DUMMY_SUPPORTED_CAPTURE_USE_CASES;

    return 0;
}

static int aud_dev_dummy_get_vol_db_range(struct device *dev,
                           struct device_aud_vol_range *vol_range)
{
    gb_debug("%s()\n", __func__);

    /* 127.5 db */
    vol_range->min = -12750;
    /* 0.5 db steps */
    vol_range->step = 50;

    return 0;
}

static int aud_dev_dummy_set_current_use_case(struct device *dev, uint32_t use_case)
{
    return 0;
}

static int aud_dev_dummy_set_volume(struct device *dev, uint32_t vol_step)
{
    gb_debug("%s()\n", __func__);

    return 0;
}

static int aud_dev_dummy_set_sys_volume(struct device *dev, int32_t vol_db)
{

    gb_debug("%s()\n", __func__);

    return 0;
}

static int aud_dev_dummy_get_supp_devices(struct device *dev,
                           struct device_aud_devices *devices)
{
    gb_debug("%s()\n", __func__);

    devices->in_devices = AUD_DEV_DUMMY_SUPPORTED_IN_DEVICES;
    devices->out_devices = AUD_DEV_DUMMY_SUPPORTED_OUT_DEVICES;

    return 0;
}

static int aud_dev_dummy_enable_devices(struct device *dev,
                           struct device_aud_devices *devices)
{
    gb_debug("%s() in_devices: 0x%x out_devices 0x%x\n", __func__,
                         devices->in_devices, devices->out_devices);

    return 0;
}

static int aud_dev_dummy_get_config(struct device *dev,
                           struct device_aud_pcm_config *pcm,
                           struct device_aud_dai_config *dai)
{
    gb_debug("%s()\n", __func__);

    memcpy(pcm, &aud_dummy_pcm, sizeof(struct device_aud_pcm_config));
    if (dai->dai_type == DEVICE_AUDIO_DAI_I2S_TYPE) {
        memcpy(&dai->config.i2s_dai, &aud_dummy_i2s_dai,
                                   sizeof(struct device_aud_i2s_config));
        return 0;
    }

    return -EINVAL;
}

static int aud_dev_dummy_set_config(struct device *dev,
                           struct device_aud_pcm_config *pcm,
                           struct device_aud_dai_config *dai)
{
    gb_debug("%s()\n", __func__);

    return 0;
}

static int aud_dev_dummy_rx_dai_start(struct device *dev)
{
    gb_debug("%s()\n", __func__);

    return 0;
}

static int aud_dev_dummy_tx_dai_start(struct device *dev)
{
    gb_debug("%s()\n", __func__);

    return 0;
}

static int aud_dev_dummy_tx_dai_stop(struct device *dev)
{
    gb_debug("%s()\n", __func__);

    return 0;
}

static int aud_dev_dummy_rx_dai_stop(struct device *dev)
{
    gb_debug("%s()\n", __func__);

    return 0;
}

/* audio device protocol driver and ops */
static struct device_aud_dev_type_ops aud_dev_dummy_type_ops = {
    .get_volume_db_range         = aud_dev_dummy_get_vol_db_range,
    .get_supported_use_cases     = aud_dev_dummy_get_supported_use_cases,
    .set_current_playback_use_case = aud_dev_dummy_set_current_use_case,
    .set_volume                  = aud_dev_dummy_set_volume,
    .set_sys_volume              = aud_dev_dummy_set_sys_volume,
    .get_supp_devices            = aud_dev_dummy_get_supp_devices,
    .enable_devices              = aud_dev_dummy_enable_devices,
    .get_supp_config             = aud_dev_dummy_get_config,
    .set_config                  = aud_dev_dummy_set_config,
    .rx_dai_start                = aud_dev_dummy_rx_dai_start,
    .tx_dai_start                = aud_dev_dummy_tx_dai_start,
    .rx_dai_stop                 = aud_dev_dummy_rx_dai_stop,
    .tx_dai_stop                 = aud_dev_dummy_tx_dai_stop,
};

static struct device_driver_ops aud_dev_dummy_driver_ops = {
    .open           = aud_dev_dummy_open,
    .type_ops = &aud_dev_dummy_type_ops,
};

struct device_driver audio_dev_dummy_driver = {
    .type   = DEVICE_TYPE_MUC_AUD_HW,
    .name   = "Audio_Device_Null_Driver",
    .desc   = "Audio Device NUll Driver",
    .ops    = &aud_dev_dummy_driver_ops,
};
