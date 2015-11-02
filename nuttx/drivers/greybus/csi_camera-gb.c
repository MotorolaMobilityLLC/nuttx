/*
 * Copyright (c) 2015 Motorola Mobility
 * All rights reserved.
 */

#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/greybus/greybus.h>
#include <nuttx/gpio.h>

#include "csi_camera.h"

/* Version of the Greybus camera protocol we support */
#define GB_CAMERA_VERSION_MAJOR 0x00
#define GB_CAMERA_VERSION_MINOR 0x01

/* Greybus camera request types */
#define GB_CAMERA_TYPE_INVALID          0x00
#define GB_CAMERA_TYPE_PROTOCOL_VERSION 0x01
#define GB_CAMERA_TYPE_QUERY_INFO       0x02
#define GB_CAMERA_TYPE_POWER_ON         0x03
#define GB_CAMERA_TYPE_POWER_OFF        0x04
#define GB_CAMERA_TYPE_STREAM_ON        0x05
#define GB_CAMERA_TYPE_STREAM_OFF       0x06
#define GB_CAMERA_TYPE_SET_RESOLUTION   0x07
#define GB_CAMERA_TYPE_SET_TEST_PATTERN 0x08

#define GPIO_APBE_CAM_PWR_EN (3) /* output to camera (CAM_DVDD_EN) */
#define GPIO_APBE_CAM_RST_N  (5) /* output to camera (CAM_RST_N) */

#define CAMERA_POWER_DELAY_US (100000)

struct gb_proto_version_response {
    uint8_t major;
    uint8_t minor;
};

static struct camera_dev_s cam_dev;

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
    case -EILSEQ:
        return GB_OP_MALFUNCTION;
    case -ENODEV:
        return GB_OP_NONEXISTENT;
    case -EIO:
    default:
        return GB_OP_UNKNOWN_ERROR;
    }
}

static uint8_t gb_camera_protocol_version(struct gb_operation *operation)
{
    struct gb_proto_version_response *response;

    lldbg("%s: Enter\n", __func__);
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_CAMERA_VERSION_MAJOR;
    response->minor = GB_CAMERA_VERSION_MINOR;

    lldbg("%s: Leave\n", __func__);
    return GB_OP_SUCCESS;
}

static uint8_t gb_camera_query_info(struct gb_operation *operation)
{
    int retval;
    struct gb_csi_camera_info *cam_info;
    lldbg("%s: Enter\n", __func__);
    cam_info = gb_operation_alloc_response(operation, sizeof(*cam_info));
    if (!cam_info)
        return GB_OP_NO_MEMORY;
    retval = cam_query_info(&cam_dev, cam_info);
    lldbg("%s: result %d\n", __func__, retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_power_on(struct gb_operation *operation)
{
    int retval = cam_power_on(&cam_dev);
    //TODO:stop cdsi cam tx when power off camera
    //TODO: when detached, ensure camera is stop and off
    cdsi_camrx_start();
    lldbg("%s: result %d\n", __func__, retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_power_off(struct gb_operation *operation)
{
    int retval = cam_power_off(&cam_dev);
    lldbg("%s: result %d\n", __func__, retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_stream_on(struct gb_operation *operation)
{
    int retval = cam_stream_on(&cam_dev);
    lldbg("%s: result %d\n", __func__, retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_stream_off(struct gb_operation *operation)
{
    int retval = cam_stream_off(&cam_dev);
    lldbg("%s: result %d\n", __func__, retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_set_resolution(struct gb_operation *operation)
{
    int retval;
    uint8_t *res;
    res = (uint8_t*)gb_operation_get_request_payload(operation);
    retval = cam_set_resolution(&cam_dev, *res);
    lldbg("%s: result %d\n", __func__, retval);
    return gb_operation_errno_map(retval);
}

static uint8_t gb_camera_set_test_pattern(struct gb_operation *operation)
{
#ifdef TEST_PATTERN
    int retval;
    uint8_t *test;
    test = (uint8_t*)gb_operation_get_request_payload(operation);
    retval = cam_set_test_pattern(&cam_dev, *test);
    return gb_operation_errno_map(retval);
#else
    return GB_OP_NONEXISTENT;
#endif
}

static struct gb_operation_handler gb_camera_handlers[] = {
    GB_HANDLER(GB_CAMERA_TYPE_PROTOCOL_VERSION, gb_camera_protocol_version),
    GB_HANDLER(GB_CAMERA_TYPE_QUERY_INFO, gb_camera_query_info),
    GB_HANDLER(GB_CAMERA_TYPE_POWER_ON, gb_camera_power_on),
    GB_HANDLER(GB_CAMERA_TYPE_POWER_OFF, gb_camera_power_off),
    GB_HANDLER(GB_CAMERA_TYPE_STREAM_ON, gb_camera_stream_on),
    GB_HANDLER(GB_CAMERA_TYPE_STREAM_OFF, gb_camera_stream_off),
    GB_HANDLER(GB_CAMERA_TYPE_SET_RESOLUTION, gb_camera_set_resolution),
    GB_HANDLER(GB_CAMERA_TYPE_SET_TEST_PATTERN, gb_camera_set_test_pattern),
};

static struct gb_driver gb_camera_driver = {
    .op_handlers = gb_camera_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_camera_handlers),
};

void gb_moto_camera_register(int cport)
{
    lldbg("%s: Enter %d\n", __func__, cport);
    int retval;
    retval = cam_setup(&cam_dev);
    if (retval == 0) {
        gb_register_driver(cport, &gb_camera_driver);
    } else {
        lldbg("%s: failed to setup camera\n", __func__);
    }
    lldbg("%s: Leave\n", __func__);
}
