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
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/util.h>
#include <nuttx/tfa9890.h>

/* Supported playback usecases */
#define TFA9890_SUPPORTED_PLAYBACK_USE_CASES (DEV_AUDIO_PLAYBACK_MUSIC_USE_CASE | \
                           DEV_AUDIO_PLAYBACK_VOICE_CALL_SPKR_USE_CASE | \
                           DEV_AUDIO_PLAYBACK_RINGTONE_USE_CASE | \
                           DEV_AUDIO_PLAYBACK_SONIFICATION_USE_CASE)
/* Supported capture usecases */
#define TFA9890_SUPPORTED_CAPTURE_USE_CASES  DEV_AUDIO_CAPTURE_DEFAULT_USE_CASE
/* Supported input and ouput devices */
#define TFA9890_SUPPORTED_OUT_DEVICES   DEV_AUDIO_DEVICE_OUT_LOUDSPEAKER
#define TFA9890_SUPPORTED_IN_DEVICES    DEV_AUDIO_DEVICE_IN_EC_REF
/* Supported i2s dai and pcm configurations */
#define TFA9890_I2S_RATE_MASK    DEVICE_I2S_PCM_RATE_48000
#define TFA9890_I2S_PCM_FMT_MASK    DEVICE_I2S_PCM_FMT_16
#define TFA9890_I2S_PROTOCOL_MASK    DEVICE_I2S_PROTOCOL_I2S
#define TFA9890_I2S_WCLK_PALARITY_MASK    DEVICE_I2S_POLARITY_NORMAL
#define TFA9890_I2S_WCLK_EDGE_MASK    DEVICE_I2S_EDGE_RISING
#define TFA9890_I2S_RXCLK_EDGE_MASK    DEVICE_I2S_EDGE_RISING
#define TFA9890_I2S_TXCLK_EDGE_MASK    DEVICE_I2S_EDGE_FALLING


extern struct tfa9890_lowerhalf_s * tfa9890_driver_init(FAR struct i2c_dev_s *i2c,
                                                   uint32_t i2c_addr,
                                                   uint32_t frequency,
                                                   int type);
extern void tfa9890_deinit(FAR struct tfa9890_lowerhalf_s *dev);

struct audio_tfa9890_devices {
     int i2c_addr;
     int type;
};

static const struct audio_tfa9890_devices tfa9890_devices[] =
{
#if defined (CONFIG_MODS_AUDIO_TFA9890_STEREO)
     {
         .i2c_addr = CONFIG_MODS_TFA9890_LEFT_I2C_ADDR,
         .type = TFA9890_LEFT,
     },
     {
         .i2c_addr = CONFIG_MODS_TFA9890_RIGHT_I2C_ADDR,
         .type = TFA9890_RIGHT,
     }
#endif
#if defined (CONFIG_MODS_AUDIO_TFA9890_MONO)
     {
         .i2c_addr = CONFIG_MODS_TFA9890_MONO_I2C_ADDR,
         .type = TFA9890_MONO,
     },
#endif
};

struct gb_audio_s {
    struct tfa9890_lowerhalf_s *aud_dev_s[ARRAY_SIZE(tfa9890_devices)];
    bool i2s_port_active;
    uint8_t use_case;
    uint32_t vol_step;
    uint32_t sys_vol_db;
    uint32_t out_enabled_devices;
    int rst_ls_gpio;
    int en_gpio;
    bool is_initialized;
    struct i2c_dev_s  *i2c_dev;
};

static struct gb_audio_s gb_aud;

/* default I2S configuration for tfa9890, IC will operate in this config for all
 * use cases
 */
static const struct device_aud_pcm_config tfa9890_i2s_pcm = {
    .rate       = TFA9890_I2S_RATE_MASK,
    .channels   = 2,
    .format     = TFA9890_I2S_PCM_FMT_MASK,
};

static const struct device_aud_i2s_config tfa9890_i2s_dai = {
    .protocol            = TFA9890_I2S_PROTOCOL_MASK,
    .wclk_polarity       = TFA9890_I2S_WCLK_PALARITY_MASK,
    .wclk_change_edge    = TFA9890_I2S_WCLK_EDGE_MASK,
    .data_tx_edge        = TFA9890_I2S_TXCLK_EDGE_MASK,
    .data_rx_edge        = TFA9890_I2S_RXCLK_EDGE_MASK,
};

/* Mods Audio Methods */
static int tfa9890_aud_dev_probe(struct device *dev)
{
    struct device_resource *r;
    struct gb_audio_s *priv = &gb_aud;

    device_set_private(dev, &gb_aud);

    r = device_resource_get_by_name(dev,
                            DEVICE_RESOURCE_TYPE_GPIO, "rst_ls");
    if (!r) {
        llvdbg("no reset gpio found\n");
        priv->rst_ls_gpio = -1;
    } else {
        /* pull reset gpio low */
        priv->rst_ls_gpio = r->start;
        gpio_direction_out(priv->rst_ls_gpio, 0);
    }

    r = device_resource_get_by_name(dev,
                             DEVICE_RESOURCE_TYPE_GPIO, "audio_en");
    if (!r) {
        llvdbg("no audio enable gpio found\n");
        priv->en_gpio = -1;
    } else {
        /* enable audio */
        priv->en_gpio = r->start;
        gpio_direction_out(priv->en_gpio, 1);
    }

    return 0;
}

static int tfa9890_aud_dev_open(struct device *dev)
{
    int i;
    FAR struct i2c_dev_s  *i2c_dev;
    struct gb_audio_s *priv = device_get_private(dev);

    if (!priv) {
        lldbg("ERROR: device private data is null\n");
        return -ENODEV;
    }
    if (priv->is_initialized)
        return 0;

    i2c_dev = up_i2cinitialize(CONFIG_MODS_TFA9890_I2C_BUS);
    if (!i2c_dev)
    {
        lldbg("ERROR: failed to init i2c \n");
        return -ENODEV;
    }
    for (i = 0; i < ARRAY_SIZE(tfa9890_devices); i++)
    {
        priv->aud_dev_s[i] = tfa9890_driver_init(i2c_dev,
                                         tfa9890_devices[i].i2c_addr,
                                         400000,
                                         tfa9890_devices[i].type);
        if(!priv->aud_dev_s[i])
        {
            lldbg("ERROR: failed to initialize device 0X%x\n",
                                         tfa9890_devices[i].i2c_addr);
            up_i2cuninitialize(i2c_dev);
            return -ENODEV;
        }
    }
    priv->is_initialized = true;
    return 0;
}

static void tfa9890_aud_dev_close(struct device *dev)
{
    struct gb_audio_s *priv = device_get_private(dev);
    int i;

    if (!priv) {
        lldbg("ERROR: device private data is null\n");
        return;
    }

    if (priv->en_gpio >= 0) {
        /* disable audio */
        gpio_set_value(priv->en_gpio, 0);
    }

    for (i = 0; i < ARRAY_SIZE(tfa9890_devices); i++)
    {
         if(priv->aud_dev_s[i]) {
             tfa9890_deinit(priv->aud_dev_s[i]);
         }
    }
    if (priv->i2c_dev) {
        up_i2cuninitialize(priv->i2c_dev);
    }
    priv->is_initialized = false;
}

static int tfa9890_aud_dev_get_supported_use_cases(struct device *dev,
                           struct device_aud_usecases *use_cases)
{
    use_cases->playback_usecases = TFA9890_SUPPORTED_PLAYBACK_USE_CASES;
    use_cases->capture_usecases = TFA9890_SUPPORTED_CAPTURE_USE_CASES;

    return 0;
}

static int tfa9890_aud_dev_get_vol_db_range(struct device *dev,
                           struct device_aud_vol_range *vol_range)
{
    /* 127.5 db */
    vol_range->min = -12750;
    /* 0.5 db steps */
    vol_range->step = 50;

    return 0;
}

static int gb_to_tfa9890_use_case(uint32_t use_case)
{
    int ret;

    switch (use_case) {
    case DEV_AUDIO_PLAYBACK_MUSIC_USE_CASE:
        ret = TFA9890_MUSIC;
        break;
    case DEV_AUDIO_PLAYBACK_VOICE_CALL_SPKR_USE_CASE:
        ret = TFA9890_VOICE;
        break;
    case DEV_AUDIO_PLAYBACK_NONE_USE_CASE:
    case DEV_AUDIO_PLAYBACK_RINGTONE_USE_CASE:
    case DEV_AUDIO_PLAYBACK_SONIFICATION_USE_CASE:
        ret = TFA9890_RINGTONE;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static int tfa9890_aud_dev_set_current_use_case(struct device *dev, uint32_t use_case)
{
    struct tfa9890_lowerhalf_s *aud_dev;
    struct tfa9890_caps_s caps;
    int ret;
    int i;
    int tfa9890_use_case;
    struct gb_audio_s *priv = device_get_private(dev);

    if (!priv) {
        lldbg("ERROR: device private data is null\n");
        return -ENODEV;
    }

    tfa9890_use_case = gb_to_tfa9890_use_case(use_case);
    /* if unsupported playback use case, fallback to music don't return error*/
    if (tfa9890_use_case < 0)
        priv->use_case = TFA9890_MUSIC;
    else
        priv->use_case = tfa9890_use_case;

    caps.type = TFA9890_USECASE;
    caps.param = tfa9890_use_case;

    for (i = 0; i < ARRAY_SIZE(tfa9890_devices); i++)
    {
        aud_dev = priv->aud_dev_s[i];
        if (aud_dev && aud_dev->ops->configure)
        {
            ret = aud_dev->ops->configure(aud_dev, &caps);
            if (ret)
                return -EINVAL;
        }
    }

    return 0;
}

static int tfa9890_aud_dev_set_volume(struct device *dev, uint32_t vol_step)
{
    struct tfa9890_lowerhalf_s *aud_dev;
    struct tfa9890_caps_s caps;
    int ret;
    int i;
    struct gb_audio_s *priv = device_get_private(dev);

    if (!priv) {
        lldbg("ERROR: device private data is null\n");
        return -ENODEV;
    }

    priv->vol_step = vol_step;
    caps.type = TFA9890_VOLUME;
    caps.param = vol_step;

    for (i = 0; i < ARRAY_SIZE(tfa9890_devices); i++)
    {
        aud_dev = priv->aud_dev_s[i];
        if (aud_dev && aud_dev->ops->configure)
        {
            ret = aud_dev->ops->configure(aud_dev, &caps);
            if (ret)
                return -EINVAL;
        }
    }

    return 0;
}

static int tfa9890_aud_dev_set_sys_volume(struct device *dev, int32_t vol_db)
{
    struct tfa9890_lowerhalf_s *aud_dev;
    struct tfa9890_caps_s caps;
    int ret;
    int i;
    struct gb_audio_s *priv = device_get_private(dev);

    if (!priv) {
        lldbg("ERROR: device private data is null\n");
        return -ENODEV;
    }

    priv->sys_vol_db = vol_db;
    caps.type = TFA9890_SYS_VOL;
    caps.param = vol_db;

    for (i = 0; i < ARRAY_SIZE(tfa9890_devices); i++)
    {
        aud_dev = priv->aud_dev_s[i];
        if (aud_dev && aud_dev->ops->configure)
        {
            ret = aud_dev->ops->configure(aud_dev, &caps);
            if (ret)
                return -EINVAL;
        }
    }

    return 0;
}

static int tfa9890_aud_dev_get_supp_devices(struct device *dev,
                           struct device_aud_devices *devices)
{
    devices->in_devices = TFA9890_SUPPORTED_IN_DEVICES;
    devices->out_devices = TFA9890_SUPPORTED_OUT_DEVICES;

    return 0;
}

static int tfa9890_aud_dev_enable_devices(struct device *dev,
                           struct device_aud_devices *devices)
{
    struct gb_audio_s *priv = device_get_private(dev);

    if (!priv) {
        lldbg("ERROR: device private data is null\n");
        return -ENODEV;
    }
    priv->out_enabled_devices = devices->out_devices;

    return 0;
}

static int tfa9890_aud_dev_op_get_caps(struct device *dev,
                           struct device_aud_pcm_config *pcm,
                           struct device_aud_dai_config *dai)
{

    memcpy(pcm, &tfa9890_i2s_pcm, sizeof(struct device_aud_pcm_config));
    if (dai->dai_type == DEVICE_AUDIO_DAI_I2S_TYPE) {
        memcpy(&dai->config.i2s_dai, &tfa9890_i2s_dai,
                                   sizeof(struct device_aud_i2s_config));
        return 0;
    }

    return -EINVAL;
}

static int tfa9890_aud_dev_op_set_config(struct device *dev,
                           struct device_aud_pcm_config *i2s_pcm,
                           struct device_aud_dai_config *dai)
{
    struct device_aud_i2s_config *i2s_dai;

     i2s_dai = &dai->config.i2s_dai;
     if (!i2s_dai) {
        lldbg("ERROR:Invalid Args passed\n");
        return -EINVAL;
     }

    /*
     * check if cfg is valid for now, as only one default
     * configuration is supported.
     */

    if (!ONE_BIT_IS_SET(i2s_dai->protocol) ||
          !ONE_BIT_IS_SET(i2s_dai->wclk_polarity) ||
          !ONE_BIT_IS_SET(i2s_dai->wclk_change_edge) ||
          !ONE_BIT_IS_SET(i2s_dai->data_rx_edge) ||
          !ONE_BIT_IS_SET(i2s_dai->data_tx_edge) ||
          !ONE_BIT_IS_SET(i2s_pcm->rate) ||
          !ONE_BIT_IS_SET(i2s_pcm->format)) {
         lldbg("ERROR: i2s configuration more than 1 bit set\n");
         return -EINVAL;
    }

    if (i2s_pcm->channels < 0 || i2s_pcm->channels > 2) {
         lldbg("ERROR:i2s channels configuration invalid\n");
         return -ERANGE;
    }

     /* Check that bit field settings match our capabilities */
    if (((i2s_dai->protocol & TFA9890_I2S_PROTOCOL_MASK) !=
                                              TFA9890_I2S_PROTOCOL_MASK)||
         ((i2s_dai->wclk_polarity & TFA9890_I2S_WCLK_PALARITY_MASK) !=
                                              TFA9890_I2S_WCLK_PALARITY_MASK) ||
         ((i2s_dai->wclk_change_edge & TFA9890_I2S_WCLK_EDGE_MASK) !=
                                              TFA9890_I2S_WCLK_EDGE_MASK) ||
         ((i2s_dai->data_rx_edge & TFA9890_I2S_RXCLK_EDGE_MASK) !=
                                              TFA9890_I2S_RXCLK_EDGE_MASK) ||
         ((i2s_dai->data_tx_edge & TFA9890_I2S_TXCLK_EDGE_MASK) !=
                                              TFA9890_I2S_TXCLK_EDGE_MASK) ||
         ((i2s_pcm->rate & TFA9890_I2S_RATE_MASK) !=
                                              TFA9890_I2S_RATE_MASK) ||
         ((i2s_pcm->format & TFA9890_I2S_PCM_FMT_MASK) !=
                                              TFA9890_I2S_PCM_FMT_MASK)) {
        lldbg("ERROR:failed matching tfa9890 capabilities\n");
        return -EINVAL;
    }

    return 0;
}

static int tfa9890_aud_dev_op_rx_dai_start(struct device *dev)
{
    int ret;
    struct tfa9890_lowerhalf_s *aud_dev;
    int i;
    struct gb_audio_s *priv = device_get_private(dev);

    if (!priv) {
        lldbg("ERROR: device private data is null\n");
        return -ENODEV;
    }

    /* TFA9890 supports only loud speaker device if its disabled
     * on I2S start return error
     */
    if (!(priv->out_enabled_devices & TFA9890_SUPPORTED_OUT_DEVICES))
    {
        lldbg("ERROR: loud speaker device not enabled\n");
        return -EINVAL;
    }

    if(!priv->i2s_port_active)
    {
        for (i = 0; i < ARRAY_SIZE(tfa9890_devices); i++)
        {
            aud_dev = priv->aud_dev_s[i];
            if (aud_dev && aud_dev->ops->start)
            {
                ret = aud_dev->ops->start(aud_dev);
                if (ret)
                    return -EINVAL;
            }
        }
    }
    priv->i2s_port_active = 1;

    return 0;
}

static int tfa9890_aud_dev_op_rx_dai_stop(struct device *dev)
{
    struct tfa9890_lowerhalf_s *aud_dev;
    int i;
    struct gb_audio_s *priv = device_get_private(dev);

    if (!priv) {
        lldbg("ERROR: device private data is null\n");
        return -ENODEV;
    }
    if(priv->i2s_port_active)
    {
        for (i = 0; i < ARRAY_SIZE(tfa9890_devices); i++)
        {
           aud_dev = priv->aud_dev_s[i];
           if (aud_dev && aud_dev->ops->stop)
               aud_dev->ops->stop(aud_dev);
        }
    }

    priv->i2s_port_active = 0;

    return 0;
}

/* audio device protocol driver and ops */
static struct device_aud_dev_type_ops tfa9890_aud_dev_type_ops = {
    .get_volume_db_range           = tfa9890_aud_dev_get_vol_db_range,
    .get_supported_use_cases       = tfa9890_aud_dev_get_supported_use_cases,
    .set_current_playback_use_case = tfa9890_aud_dev_set_current_use_case,
    .set_volume                    = tfa9890_aud_dev_set_volume,
    .set_sys_volume                = tfa9890_aud_dev_set_sys_volume,
    .get_supp_devices              = tfa9890_aud_dev_get_supp_devices,
    .enable_devices                = tfa9890_aud_dev_enable_devices,
    .get_supp_config               = tfa9890_aud_dev_op_get_caps,
    .set_config                    = tfa9890_aud_dev_op_set_config,
    .rx_dai_start                  = tfa9890_aud_dev_op_rx_dai_start,
    .rx_dai_stop                   = tfa9890_aud_dev_op_rx_dai_stop,
};

static struct device_driver_ops tfa9890_aud_dev_driver_ops = {
    .open     = tfa9890_aud_dev_open,
    .probe    = tfa9890_aud_dev_probe,
    .close    = tfa9890_aud_dev_close,
    .type_ops = &tfa9890_aud_dev_type_ops,
};

struct device_driver tfa9890_audio_dev_driver = {
    .type   = DEVICE_TYPE_MUC_AUD_HW,
    .name   = "audio_tfa9890_device_driver",
    .desc   = "Audio Tfa9890 Device Driver",
    .ops    = &tfa9890_aud_dev_driver_ops,
};

