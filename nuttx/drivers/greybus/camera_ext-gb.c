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
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <arch/byteorder.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/gpio.h>

//#define DEBUG
#include <nuttx/camera/camera_ext_dbg.h>

//support only one camera_ext device for now
#define CAMERA_EXT_DEVICE_ID 0

/* Version of the Greybus camera protocol we support */
#define GB_CAMERA_EXT_VERSION_MAJOR 0x01
#define GB_CAMERA_EXT_VERSION_MINOR 0x01

/* Used to indicate enum index is not found */
#define GB_CAMERA_EXT_INVALID_INDEX cpu_to_le32(0xFFFFFFFF)

/* Greybus camera request types */
#define GB_CAMERA_TYPE_INVALID          0x00
#define GB_CAMERA_TYPE_PROTOCOL_VERSION 0x01

#define GB_CAMERA_TYPE_POWER_ON         0x02
#define GB_CAMERA_TYPE_POWER_OFF        0x03

#define GB_CAMERA_EXT_TYPE_INPUT_ENUM   0x04
#define GB_CAMERA_EXT_TYPE_INPUT_GET    0x05
#define GB_CAMERA_EXT_TYPE_INPUT_SET    0x06

#define GB_CAMERA_EXT_TYPE_FMT_ENUM     0x07
#define GB_CAMERA_EXT_TYPE_FMT_GET      0x08
#define GB_CAMERA_EXT_TYPE_FMT_SET      0x09

#define GB_CAMERA_EXT_TYPE_FMSIZE_ENUM  0x0A
#define GB_CAMERA_EXT_TYPE_FRMIVAL_ENUM 0x0B

#define GB_CAMERA_EXT_TYPE_STREAM_ON    0x0C
#define GB_CAMERA_EXT_TYPE_STREAM_OFF   0x0D

#define GB_CAMERA_EXT_TYPE_STREAM_PARM_SET 0x0E
#define GB_CAMERA_EXT_TYPE_STREAM_PARM_GET 0x0F

#define GB_CAMERA_EXT_TYPE_CTRL_GET_CFG 0x10

#define GB_CAMERA_EXT_TYPE_CTRL_GET     0x11
#define GB_CAMERA_EXT_TYPE_CTRL_SET     0x12
#define GB_CAMERA_EXT_TYPE_CTRL_TRY     0x13

#define GB_CAMERA_EXT_EVENT             0x14

struct gb_proto_version_request {
    __u8    major;
    __u8    minor;
} __packed;

struct gb_proto_version_response {
    uint8_t major;
    uint8_t minor;
} __packed;

struct gb_camera_ext_info {
    unsigned int cport;
    struct device *dev;
    char          *dev_type;
};

static struct gb_camera_ext_info dev_info = {
    .dev_type  = DEVICE_TYPE_CAMERA_EXT_HW,
};

static uint8_t gb_operation_errno_map(int code)
{
    switch (code) {
    case 0:
        return GB_OP_SUCCESS;
    case -EINTR:
        return GB_OP_INTERRUPTED;
    case -ETIMEDOUT:
        return GB_OP_TIMEOUT;
    case -ENOMEM:
        return GB_OP_NO_MEMORY;
    case -EPROTONOSUPPORT:
        return GB_OP_PROTOCOL_BAD;
    case -EMSGSIZE:
        return GB_OP_OVERFLOW;  /* Could be underflow too */
    case -EINVAL:
        return GB_OP_INVALID;
    case -EAGAIN:
        return GB_OP_RETRY;
    case -ENODEV:
        return GB_OP_NONEXISTENT;
    case -EIO:
    default:
        return GB_OP_UNKNOWN_ERROR;
    }
}

/* Callback by camera_ext device driver to send event to AP */
static int gb_camera_ext_event_send(struct device *dev, uint32_t ev_type,
            uint8_t *data, size_t data_size)
{
    struct camera_ext_event_hdr *event;
    struct gb_operation *operation;
    int ret;

    operation = gb_operation_create(dev_info.cport, GB_CAMERA_EXT_EVENT,
                    sizeof(*event) + data_size);
    if (!operation)
        return -ENOMEM;

    event = gb_operation_get_request_payload(operation);
    if (!event) {
        CAM_ERR("failed to get payload for event\n");
        gb_operation_destroy(operation);
        return -EIO;
    }

    event->type = cpu_to_le32(ev_type);
    memcpy(event->data, data, data_size);

    ret = gb_operation_send_request(operation, NULL, false);
    gb_operation_destroy(operation);

    if (ret != 0) {
        CAM_ERR("failed to send request, result %d\n", ret);
        return -EIO;
    }

    return 0;
}

static int gb_camera_ext_init(unsigned int cport)
{
    int ret;

    CAM_DBG("init camera device\n");
    if (dev_info.dev != NULL) {
        CAM_DBG("device not closed in last session\n");
        device_close(dev_info.dev);
    }

    dev_info.dev = device_open(dev_info.dev_type, CAMERA_EXT_DEVICE_ID);
    if (!dev_info.dev) {
        CAM_ERR("failed to open camera device\n");
        return -EIO;
    }

    dev_info.cport = cport;
    ret = CALL_CAM_DEV_OP(dev_info.dev, register_event_cb,
            gb_camera_ext_event_send);

    if (ret != 0) {
        CAM_ERR("failed to register event callback\n");
        device_close(dev_info.dev);
        dev_info.dev = NULL;
    }

    return ret;
}

static void gb_camera_ext_exit(unsigned int cport)
{
    if (dev_info.dev != NULL) {
        device_close(dev_info.dev);
    }
    dev_info.dev = NULL;
}

static uint8_t gb_camera_protocol_version(struct gb_operation *operation)
{
    struct gb_proto_version_request *request;
    struct gb_proto_version_response *response;

    if (gb_operation_get_request_payload_size(operation) == sizeof(*request)) {
        request = (struct gb_proto_version_request *)
                        gb_operation_get_request_payload(operation);
        int ret = CALL_CAM_DEV_OP(dev_info.dev, set_phone_ver, request->major,
                        request->minor);
        if (ret != 0)
            return gb_operation_errno_map(ret);
    } else {
        CAM_ERR("invalid version request");
        return -EINVAL;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_CAMERA_EXT_VERSION_MAJOR;
    response->minor = GB_CAMERA_EXT_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_camera_power_on(struct gb_operation *operation)
{
    int retval;
    uint8_t *mode;

    if (gb_operation_get_request_payload_size(operation) == sizeof(uint8_t)) {
        mode = (uint8_t*)gb_operation_get_request_payload(operation);
        retval = CALL_CAM_DEV_OP(dev_info.dev, power_on, *mode);
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }

    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_power_off(struct gb_operation *operation)
{
    int retval = CALL_CAM_DEV_OP(dev_info.dev, power_off);

    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_stream_on(struct gb_operation *operation)
{
    int retval = CALL_CAM_DEV_OP(dev_info.dev, stream_on);
    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_stream_off(struct gb_operation *operation)
{
    int retval = CALL_CAM_DEV_OP(dev_info.dev, stream_off);
    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_input_enum(struct gb_operation *operation)
{
    int retval;
    __le32 *index;
    struct camera_ext_input* response;

    if (gb_operation_get_request_payload_size(operation) == sizeof(__le32)) {
        index = (__le32*)gb_operation_get_request_payload(operation);
        response = gb_operation_alloc_response(operation,
            sizeof(struct camera_ext_input));
        if (response != NULL) {
            response->index = *index;
            if (CALL_CAM_DEV_OP(dev_info.dev, input_enum, response) != 0)
                response->index = GB_CAMERA_EXT_INVALID_INDEX;

            retval = 0;
        } else {
            retval = -ENOMEM;
        }
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }

    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_input_get(struct gb_operation *operation)
{
    int retval;
    __le32 *response;
    int index = -1;

    retval = CALL_CAM_DEV_OP(dev_info.dev, input_get, &index);
    if (retval == 0) {
        response = gb_operation_alloc_response(operation, sizeof(__le32));
        if (response != NULL) {
            *response = cpu_to_le32(index);
        } else {
            retval = -ENOMEM;
        }
    }

    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_input_set(struct gb_operation *operation)
{
    int retval;
    __le32 *input;

    if (gb_operation_get_request_payload_size(operation) == sizeof(__le32)) {
        input = (__le32*)gb_operation_get_request_payload(operation);
        retval = CALL_CAM_DEV_OP(dev_info.dev,
                        input_set, le32_to_cpu(*input));
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }
    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_fmt_enum(struct gb_operation *operation)
{
    int retval;
    __le32* index;
    struct camera_ext_fmtdesc *response;

    if (gb_operation_get_request_payload_size(operation) == sizeof(__le32)) {
        index = (__le32*)gb_operation_get_request_payload(operation);
        response = gb_operation_alloc_response(operation,
            sizeof(struct camera_ext_fmtdesc));
        if (response != NULL) {
            response->index = *index;
            if (CALL_CAM_DEV_OP(dev_info.dev, format_enum, response) != 0)
                response->index = GB_CAMERA_EXT_INVALID_INDEX;

            retval = 0;
        } else {
            retval = -ENOMEM;
        }
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }

    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_fmt_get(struct gb_operation *operation)
{
    int retval;
    struct camera_ext_format *response;

    response = gb_operation_alloc_response(operation,
        sizeof(struct camera_ext_format));
    if (response != NULL) {
        retval = CALL_CAM_DEV_OP(dev_info.dev, format_get, response);
    } else {
        retval = -ENOMEM;
    }
    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_fmt_set(struct gb_operation *operation)
{
    int retval;
    struct camera_ext_format *request;

    if (gb_operation_get_request_payload_size(operation) == sizeof(*request)) {
        request = (struct camera_ext_format*)
            gb_operation_get_request_payload(operation);
        retval = CALL_CAM_DEV_OP(dev_info.dev, format_set, request);
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }
    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_frmsize_enum(struct gb_operation *operation)
{
    int retval;
    struct camera_ext_frmsize *response;
    struct camera_ext_frmsize *request;

    if (gb_operation_get_request_payload_size(operation) == sizeof(*request)) {
        request = (struct camera_ext_frmsize*)
            gb_operation_get_request_payload(operation);

        response = gb_operation_alloc_response(operation,
            sizeof(struct camera_ext_frmsize));
        if (response != NULL) {
            response->index = request->index;
            response->pixelformat = request->pixelformat;
            if (CALL_CAM_DEV_OP(dev_info.dev, frmsize_enum, response) != 0)
                response->index = GB_CAMERA_EXT_INVALID_INDEX;

            retval = 0;
        } else {
            retval = -ENOMEM;
        }
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }
    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_frmival_enum(struct gb_operation *operation)
{
    int retval;
    struct camera_ext_frmival *response;
    struct camera_ext_frmival *request;

    if (gb_operation_get_request_payload_size(operation) == sizeof(*request)) {
        response = gb_operation_alloc_response(operation,
            sizeof(struct camera_ext_frmival));

        if (response != NULL) {
            request = (struct camera_ext_frmival *)
                gb_operation_get_request_payload(operation);

            response->index = request->index;
            response->pixelformat = request->pixelformat;
            response->width = request->width;
            response->height = request->height;

            if (CALL_CAM_DEV_OP(dev_info.dev, frmival_enum, response) != 0)
                response->index = GB_CAMERA_EXT_INVALID_INDEX;

            retval = 0;
        } else {
            retval = -ENOMEM;
        }
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }
    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_stream_set_parm(struct gb_operation *operation)
{
    int retval;
    struct camera_ext_streamparm *parm;

    if (gb_operation_get_request_payload_size(operation) == sizeof(*parm)) {
        parm = (struct camera_ext_streamparm*)
            gb_operation_get_request_payload(operation);
        retval = CALL_CAM_DEV_OP(dev_info.dev, stream_set_parm, parm);
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }
    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_stream_get_parm(struct gb_operation *operation)
{
    int retval;
    struct camera_ext_streamparm *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (response != NULL) {
        retval = CALL_CAM_DEV_OP(dev_info.dev, stream_get_parm, response);
    } else {
        retval = -ENOMEM;
    }
    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_ctrl_get_cfg(struct gb_operation *operation)
{
    int retval;
    struct camera_ext_predefined_ctrl_mod_req *req;
    struct camera_ext_predefined_ctrl_mod_cfg *mod_ctrl_cfg;
    uint32_t cfg_size;

    if (gb_operation_get_request_payload_size(operation) == sizeof(*req)) {
        req = (struct camera_ext_predefined_ctrl_mod_req *)
                    gb_operation_get_request_payload(operation);
        cfg_size = le32_to_cpu(req->data_size);
        mod_ctrl_cfg = gb_operation_alloc_response(operation, cfg_size);
        if (mod_ctrl_cfg != NULL) {
            retval = CALL_CAM_DEV_OP(dev_info.dev, ctrl_get_cfg,
                        le32_to_cpu(req->idx), mod_ctrl_cfg,
                        cfg_size - sizeof(*mod_ctrl_cfg));
        } else {
            retval = -ENOMEM;
        }
        CAM_DBG("idx %d, result %d\n", le32_to_cpu(req->idx), retval);
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }

    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_ctrl_get(struct gb_operation *operation)
{
    int retval;
    struct camera_ext_predefined_ctrl_mod_req *req;
    uint8_t *ctrl_val;
    uint32_t ctrl_val_size;

    if (gb_operation_get_request_payload_size(operation) == sizeof(*req)) {
        req = (struct camera_ext_predefined_ctrl_mod_req *)
                gb_operation_get_request_payload(operation);
        ctrl_val_size = le32_to_cpu(req->data_size);
        ctrl_val = gb_operation_alloc_response(operation, ctrl_val_size);
        if (ctrl_val != NULL) {
            retval = CALL_CAM_DEV_OP(dev_info.dev, ctrl_get,
                        le32_to_cpu(req->idx), ctrl_val, ctrl_val_size);
        } else {
            retval = -ENOMEM;
        }
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }

    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_ctrl_set_try(struct gb_operation *operation, int is_set)
{
    int retval;
    struct camera_ext_predefined_ctrl_mod_req *req;
    size_t req_size;
    size_t ctrl_val_size;

    req_size = gb_operation_get_request_payload_size(operation);
    if (req_size >= sizeof(*req)) {
        req = (struct camera_ext_predefined_ctrl_mod_req *)
                gb_operation_get_request_payload(operation);
        ctrl_val_size = le32_to_cpu(req->data_size);
        if (ctrl_val_size + sizeof(*req) == req_size) {
            if (is_set)
                retval = CALL_CAM_DEV_OP(dev_info.dev, ctrl_set,
                    le32_to_cpu(req->idx), req->data, ctrl_val_size);
            else
                retval = CALL_CAM_DEV_OP(dev_info.dev, ctrl_try,
                    le32_to_cpu(req->idx), req->data, ctrl_val_size);
        } else {
            CAM_ERR("invalid payload size\n");
            retval = -EINVAL;
        }
    } else {
        CAM_ERR("invalid request size\n");
        retval = -EINVAL;
    }

    CAM_DBG("result %d\n", retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_ext_ctrl_set(struct gb_operation *operation)
{
    return gb_camera_ext_ctrl_set_try(operation, 1);
}

static uint8_t gb_camera_ext_ctrl_try(struct gb_operation *operation)
{
    return gb_camera_ext_ctrl_set_try(operation, 0);
}

static struct gb_operation_handler gb_camera_handlers[] = {
    GB_HANDLER(GB_CAMERA_TYPE_PROTOCOL_VERSION, gb_camera_protocol_version),
    GB_HANDLER(GB_CAMERA_TYPE_POWER_ON, gb_camera_power_on),
    GB_HANDLER(GB_CAMERA_TYPE_POWER_OFF, gb_camera_power_off),

    GB_HANDLER(GB_CAMERA_EXT_TYPE_INPUT_ENUM, gb_camera_ext_input_enum),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_INPUT_GET, gb_camera_ext_input_get),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_INPUT_SET, gb_camera_ext_input_set),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_FMT_ENUM, gb_camera_ext_fmt_enum),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_FMT_GET, gb_camera_ext_fmt_get),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_FMT_SET, gb_camera_ext_fmt_set),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_FMSIZE_ENUM, gb_camera_ext_frmsize_enum),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_FRMIVAL_ENUM, gb_camera_ext_frmival_enum),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_STREAM_ON, gb_camera_ext_stream_on),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_STREAM_OFF, gb_camera_ext_stream_off),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_STREAM_PARM_SET,
        gb_camera_ext_stream_set_parm),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_STREAM_PARM_GET,
        gb_camera_ext_stream_get_parm),

    GB_HANDLER(GB_CAMERA_EXT_TYPE_CTRL_GET_CFG,
        gb_camera_ext_ctrl_get_cfg),

    GB_HANDLER(GB_CAMERA_EXT_TYPE_CTRL_GET, gb_camera_ext_ctrl_get),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_CTRL_SET, gb_camera_ext_ctrl_set),
    GB_HANDLER(GB_CAMERA_EXT_TYPE_CTRL_TRY, gb_camera_ext_ctrl_try),
};

static struct gb_driver gb_camera_driver = {
    .init = gb_camera_ext_init,
    .exit = gb_camera_ext_exit,
    .op_handlers = gb_camera_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_camera_handlers),
};

void gb_camera_ext_register(int cport)
{
    CAM_DBG("register camera greybus driver at %d\n", cport);
    gb_register_driver(cport, &gb_camera_driver);
}
