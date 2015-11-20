/*
 * Copyright (c) 2015 Google Inc.
 * Copyright (c) 2015 Motorola LLC.
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
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <arch/byteorder.h>
#include <nuttx/arch.h>
#include <nuttx/bootmode.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/progmem.h>
#include <nuttx/version.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/mods-ctrl.h>
#include <nuttx/greybus/types.h>

#ifdef CONFIG_UNIPRO_P2P
#include <nuttx/unipro/unipro.h>
#include <arch/chip/unipro_p2p.h>
#endif

/* Version of the Greybus control protocol we support */
#define MB_CONTROL_VERSION_MAJOR              0x00
#define MB_CONTROL_VERSION_MINOR              0x01

/* Greybus control request types */
#define MB_CONTROL_TYPE_INVALID               0x00
#define MB_CONTROL_TYPE_PROTOCOL_VERSION      0x01
#define MB_CONTROL_TYPE_GET_IDS               0x02
#define MB_CONTROL_TYPE_REBOOT                0x03
#define MB_CONTROL_TYPE_PORT_CONNECTED        0x04
#define MB_CONTROL_TYPE_PORT_DISCONNECTED     0x05
#define MB_CONTROL_TYPE_SLAVE_POWER           0x06

/* Valid modes for the reboot request */
#define MB_CONTROL_REBOOT_MODE_RESET          0x01
#define MB_CONTROL_REBOOT_MODE_BOOTLOADER     0x02
#define MB_CONTROL_REBOOT_MODE_SELF_DESTRUCT  0x03

/* Valid modes for the slave power request */
#define MB_CONTROL_SLAVE_POWER_ON             0x01
#define MB_CONTROL_SLAVE_POWER_OFF            0x02
#define MB_CONTROL_SLAVE_POWER_FLASH_MODE     0x03

/* version request has no payload */
struct gb_control_proto_version_response {
    __u8      major;
    __u8      minor;
} __packed;

/* Control protocol reboot request */
struct gb_control_reboot_request {
    __u8      mode;
} __packed;
/* Control protocol reboot has no response */

/* Control protocol get_ids request has no payload */
struct gb_control_get_ids_response {
    __le32    unipro_mfg_id;
    __le32    unipro_prod_id;
    __le32    ara_vend_id;
    __le32    ara_prod_id;
    __le64    uid_low;
    __le64    uid_high;
    __le32    fw_version;
    __le32    slave_mask;
} __packed;

/* Control protocol [dis]connected request */
struct gb_control_connected_request {
    __le16    cport_id;
};
/* Control protocol [dis]connected response has no payload */

/* Control protocol slave power request */
struct mb_control_power_ctrl_request {
    __le32    slave_id;
    __u8      mode;
} __packed;
/* Control protocol slave power response has no payload */

struct mb_control_info {
    uint16_t cport;
    struct device *dev;
};

static struct mb_control_info *ctrl_info;

static uint8_t gb_control_protocol_version(struct gb_operation *operation)
{
    struct gb_control_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = MB_CONTROL_VERSION_MAJOR;
    response->minor = MB_CONTROL_VERSION_MINOR;
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

#ifdef GREYBUS_SELF_DESTRUCT
/**
 * @brief Self destruct so we go into blank flash
 */
static uint8_t mb_control_self_destruct(void)
{
    ssize_t page_start;
    ssize_t page_end;
    size_t page;
    size_t page_size;
    ssize_t size_erased;

    page_start = up_progmem_getpage(GREYBUS_SELF_DESTRUCT_START);
    if (page_start < 0)
        return GB_OP_INVALID;

    page_end = up_progmem_getpage(GREYBUS_SELF_DESTRUCT_END);
    if (page_end < 0)
        return GB_OP_INVALID;

    /* now lets erase the pages we need */
    for (page = page_start; page <= page_end; page++) {
        page_size = up_progmem_pagesize(page);

        size_erased = up_progmem_erasepage(page);
        if (size_erased != page_size) {
            break;
        }
    }

#ifdef CONFIG_ARCH_HAVE_SYSRESET
    up_systemreset(); /* will not return */
#endif

    return GB_OP_SUCCESS;
}
#endif

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
    case MB_CONTROL_REBOOT_MODE_BOOTLOADER:
        return gb_control_reboot_flash(operation);
    case MB_CONTROL_REBOOT_MODE_RESET:
        return gb_control_reboot_reset(operation);
#ifdef GREYBUS_SELF_DESTRUCT
    case MB_CONTROL_REBOOT_MODE_SELF_DESTRUCT:
        return mb_control_self_destruct();
#endif
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

    response->slave_mask = 0;
    if (ctrl_info->dev) {
        int ret = device_slave_pwrctrl_get_mask(ctrl_info->dev,
                &response->slave_mask);
        if (ret)
            gb_error("Failed to get pwrctrl mask\n");
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_control_connected(struct gb_operation *operation)
{
    struct gb_control_connected_request *request =
        gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

#ifdef CONFIG_UNIPRO_P2P
    unipro_p2p_setup_connection(le16_to_cpu(request->cport_id));
#endif

    return GB_OP_SUCCESS;
}

static uint8_t gb_control_disconnected(struct gb_operation *operation)
{
    struct gb_control_connected_request *request =
        gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

#ifdef CONFIG_UNIPRO_P2P
    unipro_reset_cport(le16_to_cpu(request->cport_id), NULL, NULL);
#endif

    return GB_OP_SUCCESS;
}

static uint8_t mb_control_power_ctrl(struct gb_operation *operation)
{
    struct mb_control_power_ctrl_request *request =
        gb_operation_get_request_payload(operation);
    int ret = 0;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    if (ctrl_info->dev) {
        ret = device_slave_pwrctrl_set_mode(ctrl_info->dev, request->mode);
    }

    return ret ? GB_OP_UNKNOWN_ERROR : GB_OP_SUCCESS;
}

static int mb_control_init(unsigned int cport)
{
    ctrl_info = zalloc(sizeof(*ctrl_info));
    if (!ctrl_info) {
        return -ENOMEM;
    }

    ctrl_info->cport = cport;

    /* see if this device functions as the power control for additional */
    /* devices.  Not having one is not an error.                        */
    ctrl_info->dev = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW, 0);
    if (!ctrl_info->dev) {
        gb_info("Failed to open SLAVE Power Control\n");
    }

    return 0;
}

static void mb_control_exit(unsigned int cport)
{
    if (!ctrl_info)
        return;

    if (ctrl_info->dev) {
        device_close(ctrl_info->dev);
    }

    free(ctrl_info);
    ctrl_info = NULL;
}

static struct gb_operation_handler mb_control_handlers[] = {
    GB_HANDLER(MB_CONTROL_TYPE_PROTOCOL_VERSION, gb_control_protocol_version),
    GB_HANDLER(MB_CONTROL_TYPE_REBOOT, gb_control_reboot),
    GB_HANDLER(MB_CONTROL_TYPE_GET_IDS, gb_control_get_ids),
    GB_HANDLER(MB_CONTROL_TYPE_PORT_CONNECTED, gb_control_connected),
    GB_HANDLER(MB_CONTROL_TYPE_PORT_DISCONNECTED, gb_control_disconnected),
    GB_HANDLER(MB_CONTROL_TYPE_SLAVE_POWER, mb_control_power_ctrl),
};

struct gb_driver mb_control_driver = {
    .init = mb_control_init,
    .exit = mb_control_exit,
    .op_handlers = (struct gb_operation_handler*) mb_control_handlers,
    .op_handlers_count = ARRAY_SIZE(mb_control_handlers),
};

int mods_cport_valid(int c)
{
    return (c == MODS_VENDOR_CTRL_CPORT);
}

void mb_control_register(int cport)
{
    gb_register_driver(cport, &mb_control_driver);
    gb_listen(cport);
}
