/*
 * Copyright (c) 2015 Motorola Mobility LLC.
 * Copyright (c) 2015 Google Inc.
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

#include <nuttx/config.h>
#include <nuttx/list.h>
#include <nuttx/device.h>
#include <nuttx/device_i2s.h>
#include <nuttx/ring_buf.h>
#include <nuttx/util.h>
#include <nuttx/wdog.h>
#include <nuttx/greybus/types.h>
#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/utils.h>
#include <arch/byteorder.h>
#include "i2s-gb.h"

#define GB_I2S_MGMT_VERSION_MAJOR        0x00
#define GB_I2S_MGMT_VERSION_MINOR        0x01

#define CONFIG_COUNT_MAX   5

#define GB_I2S_BUNDLE_0_ID              0
#define GB_I2S_BUNDLE_0_DEV_ID          1

struct gb_i2s_direct_dev {
    uint16_t            bundle_id;
    char                *dev_type;
    unsigned int        dev_id;
};

/* only one instance for I2S Direct */
static struct gb_i2s_direct_dev gb_i2s_direct_dev_id = {
        .bundle_id  = GB_I2S_BUNDLE_0_ID,
        .dev_type   = DEVICE_TYPE_I2S_HW,
        .dev_id     = GB_I2S_BUNDLE_0_DEV_ID,
};

struct gb_i2s_direct_info {
    uint16_t            mgmt_cport;
    uint16_t            tx_cport;
    uint16_t            rx_cport;
    struct device       *dev;
};

static struct gb_i2s_direct_info i2s_dev_info;

static uint8_t gb_i2s_mgmt_protocol_version(struct gb_operation *operation)
{
    struct gb_i2s_proto_version_response *response;

    gb_debug("%s()\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_I2S_MGMT_VERSION_MAJOR;
    response->minor = GB_I2S_MGMT_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static int gb_i2s_mgmt_init(unsigned int cport)
{
    gb_debug("%s()\n", __func__);

    i2s_dev_info.mgmt_cport = cport;

    i2s_dev_info.dev = device_open(gb_i2s_direct_dev_id.dev_type, gb_i2s_direct_dev_id.dev_id);

    if (!i2s_dev_info.dev) {
        gb_debug("%s() failed init \n", __func__);
        return -EIO;
    }

    return 0;
}

void gb_i2s_set_rx_cport(int cport)
{
    i2s_dev_info.rx_cport = cport;
}

void gb_i2s_set_tx_cport(int cport)
{
    i2s_dev_info.tx_cport = cport;
}

static uint8_t gb_i2s_mgmt_protocol_get_supported_cfgs(struct gb_operation *operation)
{
    struct gb_i2s_get_supported_configurations_response *response;
    const struct device_i2s_configuration *dev_cfg;
    struct gb_i2s_configuration *gb_cfg;
    uint16_t dev_cfg_cnt;
    int ret;
    int size;
    int i;

    gb_debug("%s()\n", __func__);

    ret = device_i2s_get_supported_configurations(i2s_dev_info.dev, &dev_cfg_cnt,
                                                  &dev_cfg);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    dev_cfg_cnt = MIN(dev_cfg_cnt, CONFIG_COUNT_MAX);
    size = CONFIG_COUNT_MAX * sizeof(*gb_cfg);

    response = gb_operation_alloc_response(operation, sizeof(*response) + size);
    if (!response)
        return GB_OP_NO_MEMORY;

    response->config_count = dev_cfg_cnt;
    gb_cfg = response->config;
    gb_debug("%s() cnt %d\n", __func__, dev_cfg_cnt);

    for (i = 0; i < dev_cfg_cnt; i++)
    {

       gb_cfg->sample_frequency = cpu_to_le32(dev_cfg->sample_frequency);
       gb_cfg->num_channels = dev_cfg->num_channels;
       gb_cfg->bytes_per_channel = dev_cfg->bytes_per_channel;
       gb_cfg->byte_order = dev_cfg->byte_order;
       gb_cfg->spatial_locations = cpu_to_le32(dev_cfg->spatial_locations);
       gb_cfg->ll_protocol = cpu_to_le32(dev_cfg->ll_protocol);
       gb_cfg->ll_mclk_role = dev_cfg->ll_mclk_role;
       gb_cfg->ll_bclk_role = dev_cfg->ll_bclk_role;
       gb_cfg->ll_wclk_role = dev_cfg->ll_wclk_role;
       gb_cfg->ll_wclk_polarity = dev_cfg->ll_wclk_polarity;
       gb_cfg->ll_wclk_change_edge = dev_cfg->ll_wclk_change_edge;
       gb_cfg->ll_data_tx_edge = dev_cfg->ll_data_tx_edge;
       gb_cfg->ll_data_rx_edge = dev_cfg->ll_data_rx_edge;
       gb_cfg->ll_data_offset = dev_cfg->ll_data_offset;
       dev_cfg++;
       gb_cfg++;
   }

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_mgmt_protocol_set_supported_cfgs(struct gb_operation *operation)
{
    struct gb_i2s_set_configuration_request *request =
                gb_operation_get_request_payload(operation);
    int ret;
    struct device_i2s_configuration dev_cfg;
    struct gb_i2s_configuration *gb_cfg = &request->config;


    gb_debug("%s()\n", __func__);

    dev_cfg.sample_frequency = le32_to_cpu(gb_cfg->sample_frequency);
    dev_cfg.num_channels = gb_cfg->num_channels;
    dev_cfg.bytes_per_channel = gb_cfg->bytes_per_channel;
    dev_cfg.byte_order = gb_cfg->byte_order;
    dev_cfg.spatial_locations = le32_to_cpu(gb_cfg->spatial_locations);
    dev_cfg.ll_protocol = le32_to_cpu(gb_cfg->ll_protocol);
    dev_cfg.ll_mclk_role = gb_cfg->ll_mclk_role;
    dev_cfg.ll_bclk_role = gb_cfg->ll_bclk_role;
    dev_cfg.ll_wclk_role = gb_cfg->ll_wclk_role;
    dev_cfg.ll_wclk_polarity = gb_cfg->ll_wclk_polarity;
    dev_cfg.ll_wclk_change_edge = gb_cfg->ll_wclk_change_edge;
    dev_cfg.ll_data_tx_edge = gb_cfg->ll_data_tx_edge;
    dev_cfg.ll_data_rx_edge = gb_cfg->ll_data_rx_edge;
    dev_cfg.ll_data_offset = gb_cfg->ll_data_offset;

    ret = device_i2s_set_configuration(i2s_dev_info.dev, &dev_cfg);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_mgmt_protocol_samples_per_msg(struct gb_operation *operation)
{
    gb_debug("%s()\n", __func__);

    return GB_OP_SUCCESS;

}

static uint8_t gb_i2s_mgmt_protocol_processing_delay(struct gb_operation *operation)
{
    gb_debug("%s()\n", __func__);

    return GB_OP_SUCCESS;

}

static uint8_t gb_i2s_mgmt_protocol_start_delay(struct gb_operation *operation)
{
    gb_debug("%s()\n", __func__);

    return GB_OP_SUCCESS;

}

static uint8_t  gb_i2s_mgmt_protocol_activate_cport(struct gb_operation *operation)
{
    struct gb_i2s_activate_cport_request *request =
        gb_operation_get_request_payload(operation);
        int ret;

    gb_debug("%s() activate cport: %d\n", __func__, request->cport);

    if (request->cport == i2s_dev_info.rx_cport)
        ret = device_i2s_start_receiver(i2s_dev_info.dev);
    else if (request->cport == i2s_dev_info.tx_cport)
        ret = device_i2s_start_transmitter(i2s_dev_info.dev);
    else
        ret = -EINVAL;

    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return 0;
}


static uint8_t  gb_i2s_mgmt_protocol_deactivate_cport(struct gb_operation *operation)
{
    struct gb_i2s_deactivate_cport_request *request =
        gb_operation_get_request_payload(operation);
    int ret;

    gb_debug("%s() deactivate cport: %d\n", __func__, request->cport);

   if (request->cport == i2s_dev_info.rx_cport)
        ret = device_i2s_stop_receiver(i2s_dev_info.dev);
    else if (request->cport == i2s_dev_info.tx_cport)
        ret = device_i2s_stop_receiver(i2s_dev_info.dev);
    else
        ret = -EINVAL;

    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return 0;
}

static struct gb_operation_handler gb_i2s_mgmt_handlers[] = {
    GB_HANDLER(GB_I2S_MGMT_TYPE_PROTOCOL_VERSION, gb_i2s_mgmt_protocol_version),
    GB_HANDLER(GB_I2S_MGMT_TYPE_GET_SUPPORTED_CONFIGURATIONS, gb_i2s_mgmt_protocol_get_supported_cfgs),
    GB_HANDLER(GB_I2S_MGMT_TYPE_SET_CONFIGURATION, gb_i2s_mgmt_protocol_set_supported_cfgs),
    GB_HANDLER(GB_I2S_MGMT_TYPE_SET_SAMPLES_PER_MESSAGE, gb_i2s_mgmt_protocol_samples_per_msg),
    GB_HANDLER(GB_I2S_MGMT_TYPE_GET_PROCESSING_DELAY, gb_i2s_mgmt_protocol_processing_delay),
    GB_HANDLER(GB_I2S_MGMT_TYPE_SET_START_DELAY, gb_i2s_mgmt_protocol_start_delay),
    GB_HANDLER(GB_I2S_MGMT_TYPE_ACTIVATE_CPORT, gb_i2s_mgmt_protocol_activate_cport),
    GB_HANDLER(GB_I2S_MGMT_TYPE_DEACTIVATE_CPORT, gb_i2s_mgmt_protocol_deactivate_cport),
};

static struct gb_driver gb_i2s_mgmt_driver = {
    .init = gb_i2s_mgmt_init,
    .op_handlers = (struct gb_operation_handler*)gb_i2s_mgmt_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_i2s_mgmt_handlers),
};

void gb_i2s_direct_mgmt_register(int cport)
{
    gb_debug("%s()\n", __func__);

    gb_register_driver(cport, &gb_i2s_mgmt_driver);
}
