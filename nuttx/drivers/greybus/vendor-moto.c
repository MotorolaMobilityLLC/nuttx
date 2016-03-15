/*
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>

#include <arch/byteorder.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/greybus/greybus.h>

#ifdef CONFIG_RAMLOG_SYSLOG
#  include <nuttx/syslog/ramlog.h>
#endif

#include "vendor-moto-gb.h"

#define GB_VENDOR_MOTO_VERSION_MAJOR     0
#define GB_VENDOR_MOTO_VERSION_MINOR     3

static uint8_t gb_vendor_moto_protocol_version(struct gb_operation *operation)
{
    struct gb_vendor_moto_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_VENDOR_MOTO_VERSION_MAJOR;
    response->minor = GB_VENDOR_MOTO_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

static uint8_t gb_vendor_moto_get_dmesg(struct gb_operation *operation)
{
#ifdef CONFIG_RAMLOG_SYSLOG
    struct gb_vendor_moto_get_dmesg_response *response;
    int fd;
    int ret;

    fd = open(CONFIG_SYSLOG_DEVPATH, O_RDONLY);
    if (fd < 0)
        return GB_OP_UNKNOWN_ERROR;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = read(fd, response->buf, GB_VENDOR_MOTO_DMESG_SIZE);
    if (ret < 0)
        return GB_OP_UNKNOWN_ERROR;

    close(fd);
    return GB_OP_SUCCESS;
#else
    return GB_OP_NONEXISTENT;
#endif
}

static uint8_t gb_vendor_moto_get_last_dmesg(struct gb_operation *operation)
{
#ifdef CONFIG_RAMLOG_LAST_DMESG
    struct gb_vendor_moto_get_dmesg_response *response;
    int fd;
    int ret;

    fd = open(CONFIG_SYSLOG_LAST_DEVPATH, O_RDONLY);
    if (fd < 0)
        return GB_OP_INVALID;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = read(fd, response->buf, GB_VENDOR_MOTO_DMESG_SIZE);
    if (ret < 0)
        return GB_OP_UNKNOWN_ERROR;

    close(fd);
    return GB_OP_SUCCESS;
#else
    return GB_OP_NONEXISTENT;
#endif
}

static uint8_t gb_vendor_moto_pwr_up_reason(struct gb_operation *operation)
{
#ifdef CONFIG_ARCH_RESET_FLAGS
    struct gb_vendor_moto_pwr_up_reason_response *response;
    int ret;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = up_resetflags(&response->reason);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;
    response->reason = cpu_to_le32(response->reason);

    return GB_OP_SUCCESS;
#else
    return GB_OP_NONEXISTENT;
#endif
}

static uint8_t gb_vendor_moto_get_dmesg_size(struct gb_operation *operation)
{
#ifdef CONFIG_RAMLOG_SYSLOG
    struct gb_vendor_moto_get_dmesg_size_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->size = cpu_to_le16(GB_VENDOR_MOTO_DMESG_SIZE);

    return GB_OP_SUCCESS;
#else
    return GB_OP_NONEXISTENT;
#endif
}

static uint8_t gb_vendor_moto_get_uptime(struct gb_operation *operation)
{
    struct gb_vendor_moto_get_uptime_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->secs = cpu_to_le32(TICK2SEC(clock_systimer()));

    return GB_OP_SUCCESS;
}

static struct gb_operation_handler gb_vendor_moto_handlers[] = {
    GB_HANDLER(GB_VENDOR_MOTO_PROTOCOL_VERSION, gb_vendor_moto_protocol_version),
    GB_HANDLER(GB_VENDOR_MOTO_GET_DMESG, gb_vendor_moto_get_dmesg),
    GB_HANDLER(GB_VENDOR_MOTO_GET_LAST_DMESG, gb_vendor_moto_get_last_dmesg),
    GB_HANDLER(GB_VENDOR_MOTO_GET_PWR_UP_REASON, gb_vendor_moto_pwr_up_reason),
    GB_HANDLER(GB_VENDOR_MOTO_GET_DMESG_SIZE, gb_vendor_moto_get_dmesg_size),
    GB_HANDLER(GB_VENDOR_MOTO_GET_UPTIME, gb_vendor_moto_get_uptime),
};

static struct gb_driver gb_vendor_moto_driver = {
    .op_handlers = gb_vendor_moto_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_vendor_moto_handlers),
};

void gb_vendor_register(int cport)
{
    gb_register_driver(cport, &gb_vendor_moto_driver);
}
