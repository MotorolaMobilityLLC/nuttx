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

#include <nuttx/greybus/greybus.h>
#include <arch/chip/unipro.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>

#include "i2s-gb.h"

struct gb_audio_s {
    struct audio_lowerhalf_s *rx_aud_dev_s;
    struct audio_lowerhalf_s *tx_aud_dev_s;
    int i2s_tx_cport;
    int i2s_rx_cport;
    int i2s_mgmnt_cport;
};

static struct gb_audio_s gb_aud;


uint8_t gb_audio_i2s_get_supported_cfgs(struct gb_i2s_mgmt_get_supported_configurations_response *response)
{
    struct gb_i2s_mgmt_configuration *conf;

    //TODO: call driver ops to get capabilities send hardcoded cfg for now
    response->config_count = 1;

    conf = &response->config[0];
    conf->sample_frequency = 48000;
    conf->ll_mclk_role = GB_I2S_MGMT_ROLE_MASTER;
    conf->num_channels = 2;
    conf->bytes_per_channel = 2;
    conf->byte_order = GB_I2S_MGMT_BYTE_ORDER_LE;
    conf->spatial_locations = (GB_I2S_MGMT_SPATIAL_LOCATION_FL |
                    GB_I2S_MGMT_SPATIAL_LOCATION_FR);
    conf->ll_protocol = (GB_I2S_MGMT_PROTOCOL_I2S);
    conf->ll_bclk_role = GB_I2S_MGMT_ROLE_MASTER;
    conf->ll_wclk_role = GB_I2S_MGMT_ROLE_MASTER;
    conf->ll_wclk_polarity = GB_I2S_MGMT_POLARITY_NORMAL;
    conf->ll_wclk_change_edge = GB_I2S_MGMT_EDGE_FALLING;
    conf->ll_wclk_tx_edge = GB_I2S_MGMT_EDGE_RISING;
    conf->ll_wclk_rx_edge = GB_I2S_MGMT_EDGE_FALLING;
    conf->ll_data_offset = 1;

    return GB_OP_SUCCESS;
}

uint8_t gb_audio_i2s_set_supported_cfgs(struct gb_i2s_mgmt_configuration *config)
{

    //call driver ops to get capabilities channels, sample rate, bit rate

    return GB_OP_SUCCESS;
}

uint8_t gb_audio_i2s_activate_cport(int cport)
{
    //call tx/rx aud dev start() here
    return GB_OP_SUCCESS;
}

uint8_t gb_audio_i2s_deactivate_cport(int cport)
{
    //call tx/rx aud dev stop() here
    return GB_OP_SUCCESS;
}


int gb_audio_i2s_rx_init(int cport)
{
     gb_aud.i2s_rx_cport = cport;
    //call driver specific output set up here
    return 0;
}

int gb_audio_i2s_tx_init(int cport)
{
     gb_aud.i2s_tx_cport = cport;
    //call driver specific input setup here
    return 0;
}

int gb_audio_i2s_mgmt_init(int cport)
{
    gb_aud.i2s_mgmnt_cport = cport;
    // call driver init here
    return 0;
}
