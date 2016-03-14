/*
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
 *
 * Author: Viresh Kumar <viresh.kumar@linaro.org>
 */

#include <string.h>
#include <arch/byteorder.h>
#include <nuttx/arch.h>
#include <nuttx/bootmode.h>
#include <nuttx/version.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/unipro/unipro.h>
#include <apps/greybus-utils/manifest.h>

#include "control-gb.h"

static uint8_t gb_control_protocol_version(struct gb_operation *operation)
{
    struct gb_control_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_CONTROL_VERSION_MAJOR;
    response->minor = GB_CONTROL_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

static uint8_t gb_control_get_manifest_size(struct gb_operation *operation)
{
    struct gb_control_get_manifest_size_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->size = cpu_to_le16(get_signed_manifest_size());

    return GB_OP_SUCCESS;
}

static uint8_t gb_control_get_manifest(struct gb_operation *operation)
{
    struct gb_control_get_manifest_response *response;
    const struct greybus_manifest_header *mh;
    int size = get_signed_manifest_size();

    response = gb_operation_alloc_response(operation, size);
    if (!response)
        return GB_OP_NO_MEMORY;

    mh = get_manifest_blob();
    if (!mh) {
        gb_error("Failed to get a valid manifest\n");
        return GB_OP_INVALID;
    }

    memcpy(response->data, mh, size);

    return GB_OP_SUCCESS;
}

static uint8_t gb_control_connected(struct gb_operation *operation)
{
    int retval;
    struct gb_control_connected_request *request =
        gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    retval = gb_listen(le16_to_cpu(request->cport_id));
    if (retval) {
        gb_error("Can not connect cport %d: error %d\n",
                 le16_to_cpu(request->cport_id), retval);
        return GB_OP_INVALID;
    }

    retval = gb_notify(le16_to_cpu(request->cport_id), GB_EVT_CONNECTED);
    if (retval)
        goto error_notify;

    return GB_OP_SUCCESS;

error_notify:
    gb_stop_listening(le16_to_cpu(request->cport_id));

    return gb_errno_to_op_result(retval);
}

static uint8_t gb_control_disconnected(struct gb_operation *operation)
{
    int retval;
    struct gb_control_connected_request *request =
        gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    retval = gb_notify(le16_to_cpu(request->cport_id), GB_EVT_DISCONNECTED);
    if (retval) {
        gb_error("Cannot notify GB driver of disconnect event.\n");
        /*
         * don't return, we still want to reset the cport and stop listening
         * on the CPort.
         */
    }

#ifdef CONFIG_ARCH_CHIP_TSB
    unipro_reset_cport(le16_to_cpu(request->cport_id), NULL, NULL);
#endif

    retval = gb_stop_listening(le16_to_cpu(request->cport_id));
    if (retval) {
        gb_error("Can not disconnect cport %d: error %d\n",
                 le16_to_cpu(request->cport_id), retval);
        return GB_OP_INVALID;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief sets the flag to tell the bootloader to stay in flash mode
 */
static uint8_t gb_control_reboot_flash(struct gb_operation *operation)
{
    if (gb_bootmode_set(BOOTMODE_REQUEST_FLASH))
        gb_error("error setting boot state to flash\n");

#ifdef CONFIG_ARCH_HAVE_SYSRESET
    up_systemreset(); /* will not return */
#endif

    return GB_OP_SUCCESS;
}

/**
 * @brief performs a simple reset of the system
 */
static uint8_t gb_control_reboot_reset(struct gb_operation *operation)
{
#ifdef CONFIG_ARCH_HAVE_SYSRESET
    up_systemreset(); /* will not return */
#endif

    return GB_OP_INVALID;
}

/**
 * @brief performs the desired reboot type specified in the mode field
 * of the request.
 */
static uint8_t gb_control_reboot(struct gb_operation *operation)
{
    struct gb_control_reboot_request *request =
        gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    switch (request->mode) {
    case GB_CONTROL_REBOOT_MODE_BOOTLOADER:
        return gb_control_reboot_flash(operation);
    case GB_CONTROL_REBOOT_MODE_RESET:
        return gb_control_reboot_reset(operation);
    }

    gb_error("unsupported reboot mode\n");

    return GB_OP_INVALID;
}

static uint8_t gb_control_get_ids(struct gb_operation *operation)
{
    struct gb_control_get_ids_response *response;
    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->fw_version =
            cpu_to_le32(CONFIG_VERSION_MAJOR << 16 | CONFIG_VERSION_MINOR);

#ifdef CONFIG_ARCH_UID
    /* Populate the UID from the microprocessor */
    up_getuid(&response->uid_high, &response->uid_low);
#endif

#ifdef CONFIG_ARCH_CHIPID
    up_getchipid(&response->unipro_mfg_id, &response->unipro_prod_id);
#endif

#ifdef CONFIG_ARCH_BOARDID
    up_getboardid(&response->ara_vend_id, &response->ara_prod_id);
#endif

    return GB_OP_SUCCESS;
}

static struct gb_operation_handler gb_control_handlers[] = {
    GB_HANDLER(GB_CONTROL_TYPE_PROTOCOL_VERSION, gb_control_protocol_version),
    GB_HANDLER(GB_CONTROL_TYPE_GET_MANIFEST_SIZE, gb_control_get_manifest_size),
    GB_HANDLER(GB_CONTROL_TYPE_GET_MANIFEST, gb_control_get_manifest),
    GB_HANDLER(GB_CONTROL_TYPE_CONNECTED, gb_control_connected),
    GB_HANDLER(GB_CONTROL_TYPE_DISCONNECTED, gb_control_disconnected),
    GB_HANDLER(GB_CONTROL_TYPE_GET_IDS, gb_control_get_ids),
    GB_HANDLER(GB_CONTROL_TYPE_REBOOT, gb_control_reboot),
};

struct gb_driver control_driver = {
    .op_handlers = (struct gb_operation_handler*) gb_control_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_control_handlers),
};

void gb_control_register(int cport)
{
    gb_register_driver(cport, &control_driver);
    gb_listen(cport);
}
