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
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/util.h>

#include "audio-gb.h"
#include "i2s-gb.h"

#define NUM_INSTANCES_TFA9890    2
#define TFA9890_SUPPORTED_USE_CASES (GB_AUDIO_MUSIC_USE_CASE | \
                           GB_AUDIO_VOICE_CALL_SPKR_USE_CASE | \
                           GB_AUDIO_LOW_LATENCY_USE_CASE)
#define TFA9890_SUPPORTED_OUT_DEVICES    GB_AUDIO_DEVICE_OUT_LOUDSPEAKER
#define TFA9890_SUPPORTED_IN_DEVICES    GB_AUDIO_DEVICE_IN_EC_REF

extern struct audio_lowerhalf_s * tfa9890_driver_init(FAR struct i2c_dev_s *i2c,
                                                   uint32_t i2c_addr,
                                                   uint32_t frequency);

struct gb_audio_s {
    struct audio_lowerhalf_s *aud_dev_s[NUM_INSTANCES_TFA9890];
    bool i2s_port_active;
    uint8_t use_case;
    uint32_t vol_step;
    uint32_t sys_vol_db;
    uint32_t out_enabled_devices;
};

static struct gb_audio_s gb_aud;
static const int tfa9890_i2c_addr[NUM_INSTANCES_TFA9890]= {0x34, 0x35};

static const struct device_i2s_configuration tfa9890_i2s_config_table[] = {
    /* default I2S configuration for tfa9890, IC will operate in this config for all
     * use cases
     */
    {
        .sample_frequency       = 48000,
        .num_channels           = 2,
        .bytes_per_channel      = 2,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FL |
                                  DEVICE_I2S_SPATIAL_LOCATION_FR,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_I2S,
        .ll_mclk_role           = DEVICE_I2S_ROLE_MASTER,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_FALLING,
        .ll_data_tx_edge        = DEVICE_I2S_EDGE_RISING,
        .ll_data_rx_edge        = DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 1,
    },
};

int muc_i2s_tfa9890_direct_dev_open(struct device *dev)
{
   int i;
   FAR struct i2c_dev_s  *i2c_dev;

   gb_debug("%s()\n", __func__);

   i2c_dev = up_i2cinitialize(CONFIG_GREYBUS_TFA9890_I2C_BUS);
   if (!i2c_dev)
   {
       gb_debug("%s() failed to init i2c \n", __func__);
       return -ENODEV;
   }

   for (i = 0; i < NUM_INSTANCES_TFA9890; i++)
   {
       gb_aud.aud_dev_s[i] = tfa9890_driver_init(i2c_dev,
                                            tfa9890_i2c_addr[i], 400000);
       if(!gb_aud.aud_dev_s[i])
       {
           gb_error("%s() failed init\n", __func__);
           return 0;
       }
    }

    return 0;
}

int muc_aud_dev_open(struct device *dev)
{

    gb_debug("%s()\n", __func__);
    return 0;
}

static int muc_aud_dev_get_supported_use_cases(struct device *dev,
                           uint32_t *use_cases)
{
    gb_debug("%s()\n", __func__);

    *use_cases = TFA9890_SUPPORTED_USE_CASES;

    return 0;
}

static int muc_aud_dev_get_vol_db_range(struct device *dev,
                           struct device_aud_vol_range *vol_range)
{
    gb_debug("%s()\n", __func__);

    /* 127.5 db */
    vol_range->min = -12750;
    /* 0.5 db steps */
    vol_range->step = 50;

    return 0;
}

static int muc_aud_dev_set_current_use_case(struct device *dev, uint32_t use_case)
{
    struct audio_lowerhalf_s *aud_dev;
    FAR struct audio_caps_s caps;
    int ret;
    int i;

    if (!(TFA9890_SUPPORTED_USE_CASES & use_case))
        return -EINVAL;
    gb_debug("%s()\n", __func__);

    gb_aud.use_case = use_case;
    caps.ac_type = AUDIO_TYPE_FEATURE;
    caps.ac_format.hw = AUDIO_FU_EQUALIZER;
    caps.ac_controls.hw[0] = use_case;

    for (i = 0; i < NUM_INSTANCES_TFA9890; i++)
    {
        aud_dev = gb_aud.aud_dev_s[i];
        if (aud_dev && aud_dev->ops->configure)
        {
            ret = aud_dev->ops->configure(aud_dev, &caps);
            if (ret)
                return -EINVAL;
        }
    }

    return 0;
}

static int muc_aud_dev_set_volume(struct device *dev, uint32_t vol_step)
{
    struct audio_lowerhalf_s *aud_dev;
    struct audio_caps_s caps;
    int ret;
    int i;

    gb_debug("%s()\n", __func__);

    gb_aud.vol_step = vol_step;
    caps.ac_type = AUDIO_TYPE_FEATURE;
    caps.ac_format.hw = AUDIO_FU_VOLUME;
    caps.ac_controls.hw[0] = vol_step;

    for (i = 0; i < NUM_INSTANCES_TFA9890; i++)
    {
        aud_dev = gb_aud.aud_dev_s[i];
        if (aud_dev && aud_dev->ops->configure)
        {
           ret = aud_dev->ops->configure(aud_dev, &caps);
           if (ret)
             return -EINVAL;
        }
    }

    return 0;
}

static int muc_aud_dev_set_sys_volume(struct device *dev, int vol_db)
{
    struct audio_lowerhalf_s *aud_dev;
    struct audio_caps_s caps;
    int ret;
    int i;

    gb_debug("%s()\n", __func__);

    gb_aud.sys_vol_db = vol_db;
    caps.ac_type = AUDIO_TYPE_FEATURE;
    caps.ac_format.hw = AUDIO_FU_LOUDNESS;
    caps.ac_controls.hw[0] = vol_db;

    for (i = 0; i < NUM_INSTANCES_TFA9890; i++)
    {
        aud_dev = gb_aud.aud_dev_s[i];
        if (aud_dev && aud_dev->ops->configure)
        {
           ret = aud_dev->ops->configure(aud_dev, &caps);
           if (ret)
             return -EINVAL;
        }
    }

    return 0;
}

static int muc_aud_dev_get_supp_devices(struct device *dev,
                           struct device_aud_devices *devices)
{
    gb_debug("%s()\n", __func__);

    devices->in_devices = TFA9890_SUPPORTED_IN_DEVICES;
    devices->out_devices = TFA9890_SUPPORTED_OUT_DEVICES;

    return 0;
}

static int muc_aud_dev_enable_devices(struct device *dev,
                           struct device_aud_devices *devices)
{
    gb_debug("%s() in_devices: 0x%x out_devices 0x%x\n", __func__,
                         devices->in_devices, devices->out_devices);

    gb_aud.out_enabled_devices = devices->out_devices;

    return 0;
}

static int muc_i2s_tfa9890_direct_op_get_supported_configurations(struct device *dev,
                           uint16_t *configuration_count,
                           const struct device_i2s_configuration
                                                             *configurations[])
{
     gb_debug("%s()\n", __func__);

    *configuration_count = ARRAY_SIZE(tfa9890_i2s_config_table);
    *configurations = tfa9890_i2s_config_table;

    return 0;

}

static int muc_i2s_tfa9890_direct_op_set_configuration(struct device *dev,
                           struct device_i2s_configuration *configuration)
{
    int i;
    const struct device_i2s_configuration *config;
    gb_debug("%s()\n", __func__);

    /* TODO: only check if cfg is valid for now, as there is only one default
     * configuration in the table. will call configure to set the lower level
     * driver op when more supported configurations are added to the table.
     */
    for (i = 0; i < ARRAY_SIZE(tfa9890_i2s_config_table); i++)
    {
        config = &tfa9890_i2s_config_table[i];
        if ((config->sample_frequency == configuration->sample_frequency) &&
            (config->num_channels == configuration->num_channels) &&
            (config->bytes_per_channel == configuration->bytes_per_channel) &&
            (config->byte_order & configuration->byte_order) &&
            (config->spatial_locations & configuration->spatial_locations) &&
            (config->ll_protocol & configuration->ll_protocol) &&
            (config->ll_mclk_role & configuration->ll_mclk_role) &&
            (config->ll_bclk_role & configuration->ll_bclk_role) &&
            (config->ll_wclk_role & configuration->ll_wclk_role) &&
            (configuration->ll_bclk_role ==
                             configuration->ll_wclk_role) &&
            (config->ll_wclk_polarity & configuration->ll_wclk_polarity) &&
            (config->ll_wclk_change_edge &
                                     configuration->ll_wclk_change_edge) &&
            (config->ll_data_tx_edge & configuration->ll_data_tx_edge) &&
            (config->ll_data_rx_edge & configuration->ll_data_rx_edge) &&
            (config->ll_data_offset == configuration->ll_data_offset)) {

                return 0;
        }
    }

    return -EINVAL;
}

static int muc_i2s_tfa9890_direct_op_get_processing_delay(struct device *dev,
                           uint32_t *processing_delay)
{
    gb_debug("%s()\n", __func__);

    *processing_delay = 30000;  //30 ms will need to check for accuracy

    return 0;
}

static int muc_i2s_tfa9890_direct_op_start_receiver(struct device *dev)
{
    int ret;
    struct audio_lowerhalf_s *aud_dev;
    int i;

    gb_debug("%s()\n", __func__);
    /* 9890 supports only loud speaker device if its disabled
     * on I2S start return error
     */
    if (!(gb_aud.out_enabled_devices & GB_AUDIO_DEVICE_OUT_LOUDSPEAKER))
    {
        gb_debug("%s()loud speaker dev not enabled\n", __func__);
        return -EINVAL;
    }

    if(!gb_aud.i2s_port_active)
    {
        for (i = 0; i < NUM_INSTANCES_TFA9890; i++)
        {
           aud_dev = gb_aud.aud_dev_s[i];
           if (aud_dev && aud_dev->ops->start)
           {
               ret = aud_dev->ops->start(aud_dev);
               if (ret)
                   return -EINVAL;
           }
       }
   }
   gb_aud.i2s_port_active = 1;

   return 0;
}

static int muc_i2s_tfa9890_direct_op_stop_receiver(struct device *dev)
{
    struct audio_lowerhalf_s *aud_dev;
    int i;

    gb_debug("%s()\n", __func__);

    if(gb_aud.i2s_port_active)
    {
        for (i = 0; i < NUM_INSTANCES_TFA9890; i++)
        {
           aud_dev = gb_aud.aud_dev_s[i];
           if (aud_dev && aud_dev->ops->stop)
               aud_dev->ops->stop(aud_dev);
        }
    }

   gb_aud.i2s_port_active = 0;

   return 0;
}

/* audio device protocol driver and ops */
static struct device_aud_dev_type_ops muc_aud_dev_type_ops = {
    .get_volume_db_range         = muc_aud_dev_get_vol_db_range,
    .get_supported_use_cases     = muc_aud_dev_get_supported_use_cases,
    .set_current_use_case        = muc_aud_dev_set_current_use_case,
    .set_volume                  = muc_aud_dev_set_volume,
    .set_sys_volume              = muc_aud_dev_set_sys_volume,
    .get_supp_devices            = muc_aud_dev_get_supp_devices,
    .enable_devices              = muc_aud_dev_enable_devices,
};

static struct device_driver_ops muc_aud_dev_driver_ops = {
    .open           = muc_aud_dev_open,
    .type_ops = &muc_aud_dev_type_ops,
};

struct device_driver muc_audio_dev_driver = {
    .type   = DEVICE_TYPE_MUC_AUD_HW,
    .name   = "MUC_Audio_Device_Driver",
    .desc   = "MUC Audio Device Driver",
    .ops    = &muc_aud_dev_driver_ops,
};

/* I2S PHY protocol driver and ops */
static struct device_i2s_type_ops muc_i2s_tfa9890_direct_type_ops = {
    .get_processing_delay         = muc_i2s_tfa9890_direct_op_get_processing_delay,
    .get_supported_configurations = muc_i2s_tfa9890_direct_op_get_supported_configurations,
    .set_configuration            = muc_i2s_tfa9890_direct_op_set_configuration,
    .start_receiver               = muc_i2s_tfa9890_direct_op_start_receiver,
    .stop_receiver                = muc_i2s_tfa9890_direct_op_stop_receiver,
};

static struct device_driver_ops muc_i2s_tfa9890_direct_driver_ops = {
    .open           = muc_i2s_tfa9890_direct_dev_open,
    .type_ops   = &muc_i2s_tfa9890_direct_type_ops,
};

struct device_driver muc_i2s_tfa9890_direct_driver = {
    .type   = DEVICE_TYPE_I2S_HW,
    .name   = "muc_i2s_direct_tfa9890",
    .desc   = "MUC I2S TFA9890 Direct Driver",
    .ops    = &muc_i2s_tfa9890_direct_driver_ops,
};
