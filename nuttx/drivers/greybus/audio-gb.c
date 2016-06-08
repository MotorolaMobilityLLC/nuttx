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
#include <arch/byteorder.h>

#include "i2s-gb.h"
#include "audio-gb.h"


#define GB_AUD_VERSION_MAJOR        0x00
#define GB_AUD_VERSION_MINOR        0x03

#define GB_AUD_BUNDLE_0_DEV_ID          0x02
#define GB_AUD_BUNDLE_0_ID              0x00

struct gb_aud_info {
    uint16_t            bundle_id;
    char                *dev_type;
    unsigned int        dev_id;
    struct device       *dev;
    unsigned int        cport;
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
    struct device_aud_usecases gb_use_cases;

    gb_debug("%s()\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_audio_get_supported_use_cases(dev_info.dev, &gb_use_cases);
    if (ret)
        return GB_OP_SUCCESS;

    response->aud_use_cases.capture_usecases =
                       cpu_to_le32(gb_use_cases.capture_usecases);
    response->aud_use_cases.playback_usecases =
                       cpu_to_le32(gb_use_cases.playback_usecases);

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

    response->vol_range.min = cpu_to_le32(gb_vol_range.min);
    response->vol_range.step = cpu_to_le32(gb_vol_range.step);

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_set_playback_use_case(struct gb_operation *operation)
{
    struct gb_audio_set_use_case_request *request =
                            gb_operation_get_request_payload(operation);
    int ret;
    uint32_t use_case;

    gb_debug("%s()\n", __func__);
    use_case =  le32_to_cpu(request->use_case);
    ret =  device_audio_set_playback_use_case(dev_info.dev,
                                 use_case);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_set_capture_use_case(struct gb_operation *operation)
{
    struct gb_audio_set_use_case_request *request =
                            gb_operation_get_request_payload(operation);
    int ret;
    uint32_t use_case;

    gb_debug("%s()\n", __func__);
    use_case =  le32_to_cpu(request->use_case);
    ret =  device_audio_set_capture_use_case(dev_info.dev,
                                 use_case);
    if (ret)
        return  GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_set_volume(struct gb_operation *operation)
{
    struct gb_audio_set_volume_db_request *request =
                            gb_operation_get_request_payload(operation);
    int ret;
    int32_t vol_step;

    gb_debug("%s()\n", __func__);
    vol_step = le32_to_cpu(request->vol_step);
    ret =  device_audio_set_volume(dev_info.dev, vol_step);
    if (ret)
        return -EIO;

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_set_sys_volume(struct gb_operation *operation)
{
    struct gb_audio_set_system_volume_db_request *request =
                            gb_operation_get_request_payload(operation);
    int ret;
    int32_t vol_db;

    gb_debug("%s()\n", __func__);

    vol_db = le32_to_cpu(request->vol_db);
    ret =  device_audio_set_sys_volume(dev_info.dev, vol_db);
    if (ret)
        return -EIO;

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_get_devices(struct gb_operation *operation)
{
    struct gb_audio_get_devices_response *response;
    int ret;
    struct device_aud_devices gb_devices;

    gb_debug("%s()\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_audio_get_supp_devices(dev_info.dev, &gb_devices);
    if (ret)
        return GB_OP_SUCCESS;

    response->devices.in_devices = cpu_to_le32(gb_devices.in_devices);
    response->devices.out_devices = cpu_to_le32(gb_devices.out_devices);

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_enable_devices(struct gb_operation *operation)
{
    struct gb_audio_enable_devices_request *request =
                            gb_operation_get_request_payload(operation);
    int ret;
    struct device_aud_devices gb_devices;

    gb_debug("%s()\n", __func__);
    gb_devices.in_devices = le32_to_cpu(request->devices.in_devices);
    gb_devices.out_devices = le32_to_cpu(request->devices.out_devices);
    ret =  device_audio_enable_devices(dev_info.dev, &gb_devices);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

static int gb_audio_report_devices_request_cb(struct device *dev,
                                     struct device_aud_devices *devices)
{
    struct gb_operation *operation;
    struct gb_operation_hdr *hdr;
    struct gb_audio_report_devices_request *request;
    int ret;

    operation = gb_operation_create(dev_info.cport,
                                    GB_AUDIO_DEVICES_REPORT_EVENT,
                                    sizeof(*hdr) + sizeof(*request));
    if (!operation)
        return -ENOMEM;

    request = gb_operation_get_request_payload(operation);
    request->devices.in_devices = cpu_to_le32(devices->in_devices);
    request->devices.out_devices = cpu_to_le32(devices->out_devices);

    ret = gb_operation_send_request(operation, NULL, false);
    if (ret)
        gb_error("%s: failed reporting audio devices %d\n", __func__, ret);
    gb_operation_destroy(operation);

    return ret;
}

static uint8_t gb_aud_protocol_get_spkr_preset_eq(struct gb_operation *operation)
{
    struct gb_audio_get_speaker_preset_eq_response *response =
                            gb_operation_get_request_payload(operation);
    int ret;
    int preset_eq;

    gb_debug("%s()\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_audio_get_spkr_preset_eq(dev_info.dev, &preset_eq);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response->preset_eq = cpu_to_le32(preset_eq);

    return GB_OP_SUCCESS;
}

static uint8_t gb_aud_protocol_get_mic_params(struct gb_operation *operation)
{
    struct gb_audio_get_mic_params_response *response =
                            gb_operation_get_request_payload(operation);
    int ret;

    gb_debug("%s()\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_audio_get_mic_params(dev_info.dev, response->params,
                    GB_AUDIO_MIC_PARAMS_SIZE);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

static int gb_aud_init(unsigned int cport)
{
    gb_debug("%s()\n", __func__);
    dev_info.dev = device_open(dev_info.dev_type, dev_info.dev_id);
    if (!dev_info.dev)
        return -EIO;

    dev_info.cport = cport;
    device_audio_register_callback(dev_info.dev,
                               gb_audio_report_devices_request_cb);

    return 0;
}

static void gb_aud_exit(unsigned int cport)
{
    if(dev_info.dev) {
        device_audio_unregister_callback(dev_info.dev);
        device_close(dev_info.dev);
        dev_info.dev = NULL;
    }
}

static struct gb_operation_handler gb_aud_handlers[] = {
    GB_HANDLER(GB_AUDIO_PROTOCOL_VERSION, gb_aud_protocol_version),
    GB_HANDLER(GB_AUDIO_GET_VOLUME_DB_RANGE, gb_aud_get_vol_range),
    GB_HANDLER(GB_AUDIO_GET_SUPPORTED_USE_CASES, gb_aud_get_supported_use_case),
    GB_HANDLER(GB_AUDIO_SET_CAPTURE_USE_CASE, gb_aud_protocol_set_capture_use_case),
    GB_HANDLER(GB_AUDIO_SET_PLAYBACK_USE_CASE, gb_aud_protocol_set_playback_use_case),
    GB_HANDLER(GB_AUDIO_SET_VOLUME, gb_aud_protocol_set_volume),
    GB_HANDLER(GB_AUDIO_SET_SYSTEM_VOLUME, gb_aud_protocol_set_sys_volume),
    GB_HANDLER(GB_AUDIO_GET_SUPPORTED_DEVICES, gb_aud_protocol_get_devices),
    GB_HANDLER(GB_AUDIO_ENABLE_DEVICES, gb_aud_protocol_enable_devices),
    GB_HANDLER(GB_AUDIO_GET_SPEAKER_PRESET_EQ, gb_aud_protocol_get_spkr_preset_eq),
    GB_HANDLER(GB_AUDIO_GET_MIC_PARAMS, gb_aud_protocol_get_mic_params),
};

static struct gb_driver gb_aud_driver = {
    .init = gb_aud_init,
    .exit = gb_aud_exit,
    .op_handlers = (struct gb_operation_handler*)gb_aud_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_aud_handlers),
};

void gb_aud_register(int cport)
{
    gb_debug("%s()\n", __func__);

    gb_register_driver(cport, &gb_aud_driver);
}
