/*
 * Copyright (c) 2014-2015 Motorola Mobility LLC.
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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/utils.h>

#include "i2s-gb.h"

#define GB_I2S_MGMT_VERSION_MAJOR        0x00
#define GB_I2S_MGMT_VERSION_MINOR        0x01

#define GB_I2S_DATA_VERSION_MAJOR        0x00
#define GB_I2S_DATA_VERSION_MINOR        0x01

extern uint8_t gb_audio_i2s_get_supported_cfgs(struct gb_i2s_mgmt_get_supported_configurations_response *response);
extern uint8_t gb_audio_i2s_set_supported_cfgs(struct gb_i2s_mgmt_configuration *config);
extern uint8_t gb_audio_i2s_activate_cport(int cport);
extern uint8_t gb_audio_i2s_deactivate_cport(int cport);
extern int gb_audio_i2s_rx_init(int cport);
extern int gb_audio_i2s_tx_init(int cport);
extern int gb_audio_i2s_mgmt_init(int cport);


static uint8_t gb_i2s_mgmt_protocol_version(struct gb_operation *operation)
{
    struct gb_i2s_mgmt_version_response *response;

    gb_info("%s()\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_I2S_MGMT_VERSION_MAJOR;
    response->minor = GB_I2S_MGMT_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static int gb_i2s_mgmt_init(unsigned int cport)
{
    gb_info("%s()\n", __func__);
    return gb_audio_i2s_mgmt_init(cport);
}


static uint8_t gb_i2s_mgmt_protocol_get_supported_cfgs(struct gb_operation *operation)
{
    struct gb_i2s_mgmt_get_supported_configurations_response *response;
    size_t payload_size;

    gb_info("%s()\n", __func__);

    payload_size = sizeof(struct gb_i2s_mgmt_get_supported_configurations_response)
       +            sizeof(struct gb_i2s_mgmt_configuration) * CONFIG_COUNT_MAX;
    response = gb_operation_alloc_response(operation, (payload_size));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_audio_i2s_get_supported_cfgs(response);
}

static uint8_t gb_i2s_mgmt_protocol_set_supported_cfgs(struct gb_operation *operation)
{
    struct gb_i2s_mgmt_set_configuration_request *request;

    gb_info("%s()\n", __func__);

    request = (struct gb_i2s_mgmt_set_configuration_request *)
                          gb_operation_get_request_payload(operation);

    return gb_audio_i2s_set_supported_cfgs(&request->config);
}

static uint8_t gb_i2s_mgmt_protocol_samples_per_msg(struct gb_operation *operation)
{
    gb_info("%s()\n", __func__);

    return GB_OP_SUCCESS;

}

static uint8_t gb_i2s_mgmt_protocol_processing_delay(struct gb_operation *operation)
{
    gb_info("%s()\n", __func__);

    return GB_OP_SUCCESS;

}

static uint8_t gb_i2s_mgmt_protocol_start_delay(struct gb_operation *operation)
{
    gb_info("%s()\n", __func__);

    return GB_OP_SUCCESS;

}

static uint8_t  gb_i2s_mgmt_protocol_activate_cport(struct gb_operation *operation)
{
    struct gb_i2s_mgmt_activate_cport_request *request =
        gb_operation_get_request_payload(operation);

    gb_info("%s() activate cport: %d\n", __func__, request->cport);

    return gb_audio_i2s_activate_cport(request->cport);

}


static uint8_t  gb_i2s_mgmt_protocol_deactivate_cport(struct gb_operation *operation)
{
    struct gb_i2s_mgmt_deactivate_cport_request *request =
        gb_operation_get_request_payload(operation);

    gb_info("%s() deactivate cport: %d\n", __func__, request->cport);

    return gb_audio_i2s_deactivate_cport(request->cport);

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

void gb_i2s_mgmt_register(int cport)
{
    gb_info("%s()\n", __func__);

    gb_register_driver(cport, &gb_i2s_mgmt_driver);
}


static uint8_t gb_i2s_data_protocol_version(struct gb_operation *operation)
{
    struct gb_i2s_data_version_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;


    response->major = GB_I2S_DATA_VERSION_MAJOR;
    response->minor = GB_I2S_DATA_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static int gb_i2s_tx_init(unsigned int cport)
{
    gb_info("%s()\n", __func__);

    return gb_audio_i2s_tx_init(cport);
}

static struct gb_operation_handler gb_i2s_tx_handlers[] = {
    GB_HANDLER(GB_I2S_DATA_TYPE_PROTOCOL_VERSION, gb_i2s_data_protocol_version),
};

static struct gb_driver gb_i2s_tx_driver = {
    .init = gb_i2s_tx_init,
    .op_handlers = (struct gb_operation_handler*)gb_i2s_tx_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_i2s_tx_handlers),
};

void gb_i2s_tx_register(int cport)
{
    gb_info("%s()\n", __func__);

    gb_register_driver(cport, &gb_i2s_tx_driver);
}

static struct gb_operation_handler gb_i2s_rx_handlers[] = {
    GB_HANDLER(GB_I2S_DATA_TYPE_PROTOCOL_VERSION, gb_i2s_data_protocol_version),
};

static int gb_i2s_rx_init(unsigned int cport)
{
    gb_info("%s()\n", __func__);

    return gb_audio_i2s_rx_init(cport);
}

static struct gb_driver gb_i2s_rx_driver = {
    .init = gb_i2s_rx_init,
    .op_handlers = (struct gb_operation_handler*)gb_i2s_rx_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_i2s_rx_handlers),
};

void gb_i2s_rx_register(int cport)
{
    gb_info("%s()\n", __func__);

    gb_register_driver(cport, &gb_i2s_rx_driver);
}
