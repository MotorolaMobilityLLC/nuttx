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
#include <errno.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/device_audio.h>
#include <apps/greybus-utils/utils.h>

#include "i2s-gb.h"
#include "audio-gb.h"


#define GB_AUD_VERSION_MAJOR        0x00
#define GB_AUD_VERSION_MINOR        0x01
#define GB_AUD_BUNDLE_0_DEV_ID          0x02
#define GB_AUD_BUNDLE_0_ID              0x00

struct gb_aud_info {
    uint16_t            bundle_id;
    char                *dev_type;
    unsigned int        dev_id;
    struct device       *dev;
};

static struct gb_aud_info dev_info = {
        .bundle_id  = GB_AUD_BUNDLE_0_ID,
        .dev_type   = DEVICE_TYPE_MUC_AUD_HW,
        .dev_id     = GB_AUD_BUNDLE_0_DEV_ID,
};

static uint8_t gb_aud_protocol_version(struct gb_operation *operation)
{
    struct gb_audio_proto_version_response *response;

    gb_debug("%s()\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_AUD_VERSION_MAJOR;
    response->minor = GB_AUD_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_get_supported_use_case(struct gb_operation *operation)
{
    struct gb_audio_get_supported_usecases_response *response;
    int ret;
    uint32_t gb_use_cases;

    gb_debug("%s()\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_audio_get_supported_use_cases(dev_info.dev, &gb_use_cases);
    if (ret)
        return GB_OP_SUCCESS;

     response->use_cases = gb_use_cases;

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_get_vol_range(struct gb_operation *operation)
{
    struct gb_audio_get_volume_db_range_response *response;
    int ret;
    struct device_aud_vol_range gb_vol_range;

    gb_debug("%s()\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;


    ret = device_audio_get_volume_db_range(dev_info.dev, &gb_vol_range);
    if (ret)
        return GB_OP_SUCCESS;

    response->vol_range.min = gb_vol_range.min;
    response->vol_range.step = gb_vol_range.step;

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_set_use_case(struct gb_operation *operation)
{
    struct gb_audio_set_use_case_request *request =
                            gb_operation_get_request_payload(operation);
    int ret;

    gb_debug("%s()\n", __func__);

    ret =  device_audio_set_use_case(dev_info.dev, request->use_case);
    if (ret)
        return -EIO;

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_set_volume(struct gb_operation *operation)
{
    struct gb_audio_set_volume_db_request *request =
                            gb_operation_get_request_payload(operation);
    int ret;

    gb_debug("%s()\n", __func__);

    ret =  device_audio_set_volume(dev_info.dev, request->vol_step);
    if (ret)
        return -EIO;

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_set_sys_volume(struct gb_operation *operation)
{
    struct gb_audio_set_system_volume_db_request *request =
                            gb_operation_get_request_payload(operation);
    int ret;

    gb_debug("%s()\n", __func__);

    ret =  device_audio_set_sys_volume(dev_info.dev, request->vol_db);
    if (ret)
        return -EIO;

    return GB_OP_SUCCESS;
}

static int gb_aud_init(unsigned int cport)
{

    gb_debug("%s()\n", __func__);
    dev_info.dev = device_open(dev_info.dev_type, dev_info.dev_id);
    if (!dev_info.dev)
        return -EIO;

    return 0;
}

static struct gb_operation_handler gb_aud_handlers[] = {
    GB_HANDLER(GB_AUDIO_PROTOCOL_VERSION, gb_aud_protocol_version),
    GB_HANDLER(GB_AUDIO_GET_VOLUME_DB_RANGE, gb_aud_get_vol_range),
    GB_HANDLER(GB_AUDIO_GET_SUPPORTED_USE_CASES, gb_aud_get_supported_use_case),
    GB_HANDLER(GB_AUDIO_SET_USE_CASE, gb_aud_protocol_set_use_case),
    GB_HANDLER(GB_AUDIO_SET_VOLUME, gb_aud_protocol_set_volume),
    GB_HANDLER(GB_AUDIO_SET_SYSTEM_VOLUME, gb_aud_protocol_set_sys_volume),
};

static struct gb_driver gb_aud_driver = {
    .init = gb_aud_init,
    .op_handlers = (struct gb_operation_handler*)gb_aud_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_aud_handlers),
};

void gb_aud_register(int cport)
{
    gb_debug("%s()\n", __func__);

    gb_register_driver(cport, &gb_aud_driver);
}
