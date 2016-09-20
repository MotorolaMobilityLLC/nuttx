/*
 * Copyright (c) 2015 Google Inc.
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
 *
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <arch/byteorder.h>
#include <nuttx/arch.h>
#include <nuttx/bootmode.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/device_pwr_limit.h>
#include <nuttx/progmem.h>
#include <nuttx/rtc.h>
#include <nuttx/time.h>
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
#define MB_CONTROL_VERSION_MINOR              0x09

/* Version of the Greybus control protocol to support
 * send slave state
 */
#define MB_CONTROL_VERSION_SLAVE_STATE_MAJOR  0x00
#define MB_CONTROL_VERSION_SLAVE_STATE_MINOR  0x05

/* Version of the Greybus control protocol to support
 * send powerup reason
 */
#define MB_CONTROL_VERSION_PWRUP_REASON_MAJOR 0x00
#define MB_CONTROL_VERSION_PWRUP_REASON_MINOR 0x07

#define MB_CONTROL_SUPPORT_CURRENT_RSV_MAJOR  0x00
#define MB_CONTROL_SUPPORT_CURRENT_RSV_MINOR  0x08

#define MB_CONTROL_VERSION_TEST_MODE_MAJOR    0x00
#define MB_CONTROL_VERSION_TEST_MODE_MINOR    0x09

/* Greybus control request types */
#define MB_CONTROL_TYPE_INVALID               0x00
#define MB_CONTROL_TYPE_PROTOCOL_VERSION      0x01
#define MB_CONTROL_TYPE_GET_IDS               0x02
#define MB_CONTROL_TYPE_REBOOT                0x03
#define MB_CONTROL_TYPE_PORT_CONNECTED        0x04
#define MB_CONTROL_TYPE_PORT_DISCONNECTED     0x05
#define MB_CONTROL_TYPE_SLAVE_POWER           0x06
#define MB_CONTROL_TYPE_GET_ROOT_VER          0x07
#define MB_CONTROL_TYPE_RTC_SYNC              0x08
#define MB_CONTROL_TYPE_SLAVE_STATE           0x09

#define MB_CONTROL_TYPE_SET_CURRENT_LIMIT     0x0a
#define MB_CONTROL_TYPE_CAPABLITY_CHANGED     0x0b
#define MB_CONTROL_TYPE_GET_PWRUP_REASON      0x0c
#define MB_CONTROL_TYPE_CURRENT_RSV           0x0d
#define MB_CONTROL_TYPE_CURRENT_RSV_ACK       0x0e
#define MB_CONTROL_TYPE_TEST_MODE             0x0f

/* Valid modes for the reboot request */
#define MB_CONTROL_REBOOT_MODE_RESET          0x01
#define MB_CONTROL_REBOOT_MODE_BOOTLOADER     0x02
#define MB_CONTROL_REBOOT_MODE_SELF_DESTRUCT  0x03

/* Valid modes for the slave power request */
#define MB_CONTROL_SLAVE_POWER_ON             0x01
#define MB_CONTROL_SLAVE_POWER_OFF            0x02
#define MB_CONTROL_SLAVE_POWER_FLASH_MODE     0x03

/* Reserved values for get core version */
#define MB_CONTROL_ROOT_VER_INVALID           0x00
#define MB_CONTROL_ROOT_VER_NOT_APPLICABLE    0xff

#ifndef CONFIG_GREYBUS_MODS_HW_ROOT_VERSION
#define CONFIG_GREYBUS_MODS_HW_ROOT_VERSION MB_CONTROL_ROOT_VER_NOT_APPLICABLE
#endif

/* Only the bootloader has something interesting to
 * say at this point. */
#define PWRUP_REASON_NORMAL                      0

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
#define MB_CONTROL_FW_VER_STR_SZ              32
struct gb_control_get_ids_response {
    __le32    unipro_mfg_id;
    __le32    unipro_prod_id;
    __le32    ara_vend_id;
    __le32    ara_prod_id;
    __le64    uid_low;
    __le64    uid_high;
    __le32    fw_version;
    __le32    slave_mask;
    char      fw_version_str[MB_CONTROL_FW_VER_STR_SZ];
    __u8      fw_vendor_updates;
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

/* Control protocol get root version response */
struct mb_control_root_ver_response {
    __u8      version;
} __packed;

/* Control protocol RTC sync request */
struct mb_control_rtc_sync_request {
    __le64    nsec;
} __packed;
/* Control protocol RTC sync has no response */

/* Control protocol current limit request */
struct mb_control_current_limit_request {
    __u8 limit;
} __packed;
/* Control protocol current limit response has no payload */

struct mb_control_capability_change_request {
    __u8 level;
    __u8 reason;
    __le16 vendor;
} __packed;
/* Control protocol capability change has no response */

struct mb_control_current_rsv_request {
    __u8 rsv;
} __packed;

struct mb_control_current_rsv_ack_request {
    __u8 rsv;
} __packed;

struct mb_control_info {
    uint16_t cport;
    struct device *slv_pwr_dev;
    struct device *pwr_limit_dev;
    uint8_t host_major;
    uint8_t host_minor;
};

static struct mb_control_info *ctrl_info;

struct mb_control_slave_state_request {
    __le32  slave_mask;
    __le32  slave_state;
} __packed;

struct mb_control_get_pwrup_response {
    __le32 reason;
} __packed;

#define MODS_TEST_MODE_STATUS_ENABLED_BIT 0x00000001
struct mb_control_test_mode_request {
    __le32  value;
} __packed;

/* Keep track of changes in the clock, for potential */
/* detection of 'jumps' in time.                     */
static uint32_t mods_rtc_sync_cntr;

uint32_t mods_control_get_rtc_clock_counter(void)
{
    return mods_rtc_sync_cntr;
}

static inline void mods_control_inc_rtc_clock_counter(void)
{
    mods_rtc_sync_cntr++;
}

#ifdef CONFIG_DEVICE_CORE
/**
 * @brief send slave state
 *
 * @param slave_state: slave state is sent as part of gb message
 * @return 0 on success, error code on failure.
 */
static int mb_control_send_slave_state(uint32_t slave_state)
{
    uint32_t slave_mask = 0;

    gb_debug("%s()\n", __func__);
    /* check protocol version to make sure AP supports this message
     * before sending it.
     */
    if (ctrl_info->host_major < MB_CONTROL_VERSION_SLAVE_STATE_MAJOR) {
        gb_error("%s() failed protocol version check!\n", __func__);
        return -EINVAL;
    }
    if (ctrl_info->host_minor < MB_CONTROL_VERSION_SLAVE_STATE_MINOR &&
          ctrl_info->host_major == MB_CONTROL_VERSION_SLAVE_STATE_MAJOR)
    {
        gb_error("%s() failed protocol minor version check!\n", __func__);
        return -EINVAL;
    }

    if (ctrl_info->slv_pwr_dev) {
        int ret = device_slave_pwrctrl_get_mask(ctrl_info->slv_pwr_dev,
                &slave_mask);
        if (ret) {
            gb_error("Failed to get pwrctrl mask\n");
            return -EINVAL;
        }
    }

    struct gb_operation *ss_operation =
            gb_operation_create(MODS_VENDOR_CTRL_CPORT,
                                MB_CONTROL_TYPE_SLAVE_STATE,
                                sizeof(struct mb_control_slave_state_request));
    if (!ss_operation) {
        return -ENOMEM;
    }

    struct mb_control_slave_state_request *request =
        gb_operation_get_request_payload(ss_operation);
    request->slave_mask = cpu_to_le32(slave_mask);
    request->slave_state = cpu_to_le32(slave_state);

    int ret = gb_operation_send_request(ss_operation, NULL, false);
    if (ret) {
        gb_error("failed to send slave state\n");
    }

    gb_operation_destroy(ss_operation);

    return ret;
}

/**
 * @brief Callback for sending slave state
 *
 * @param dev Pointer to structure of device data.
 * @param slave_state: slave state is sent as part of gb message
 * @return 0 on success, error code on failure.
 */
static int mb_control_send_slave_state_cb(struct device *dev,
                                uint32_t slave_state)
{
    gb_debug("%s\n", __func__);

    /* return error if the device type is not
     * DEVICE_TYPE_SLAVE_PWRCTRL_HW.
     */
    if (strncmp(dev->type, DEVICE_TYPE_SLAVE_PWRCTRL_HW,
                           sizeof(DEVICE_TYPE_SLAVE_PWRCTRL_HW))) {
        gb_error("%s invalid device type \n", __func__);
        return GB_OP_INVALID;
    }

    return mb_control_send_slave_state(slave_state);
}
#endif

static uint8_t gb_control_protocol_version(struct gb_operation *operation)
{
    struct gb_control_proto_version_response *response =
               gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*response)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }
    ctrl_info->host_major = response->major;
    ctrl_info->host_minor = response->minor;

    gb_debug("%s() host protocol major version %d\n", __func__, response->major);
    gb_debug("%s() host protocol minor version %d\n", __func__, response->major);

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

#ifdef CONFIG_GREYBUS_SELF_DESTRUCT
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

    page_start = up_progmem_getpage(CONFIG_GREYBUS_SELF_DESTRUCT_START);
    if (page_start < 0)
        return GB_OP_INVALID;

    page_end = up_progmem_getpage(CONFIG_GREYBUS_SELF_DESTRUCT_END);
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
#ifdef CONFIG_GREYBUS_SELF_DESTRUCT
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
    snprintf(response->fw_version_str, MB_CONTROL_FW_VER_STR_SZ,
             "%s %s", CONFIG_VERSION_STRING, CONFIG_VERSION_BUILD);

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
    if (ctrl_info->slv_pwr_dev) {
        int ret = device_slave_pwrctrl_get_mask(ctrl_info->slv_pwr_dev,
                &response->slave_mask);
        if (ret)
            gb_error("Failed to get pwrctrl mask\n");
    }

#ifdef CONFIG_GREYBUS_MODS_SUPPORT_VENDOR_UPDATES
    response->fw_vendor_updates = true;
#else
    response->fw_vendor_updates = false;
#endif

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

    if (ctrl_info->slv_pwr_dev) {
        ret = device_slave_pwrctrl_set_mode(ctrl_info->slv_pwr_dev, request->mode);
    }

    return ret ? GB_OP_UNKNOWN_ERROR : GB_OP_SUCCESS;
}

static uint8_t mb_control_get_root_vers(struct gb_operation *operation)
{
    struct mb_control_root_ver_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->version = CONFIG_GREYBUS_MODS_HW_ROOT_VERSION;

    return GB_OP_SUCCESS;
}

static uint8_t mb_control_rtc_sync(struct gb_operation *operation)
{
#ifdef CONFIG_RTC
    struct mb_control_rtc_sync_request *request =
        gb_operation_get_request_payload(operation);
    struct timespec ts;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    mods_control_inc_rtc_clock_counter();

    nsec_to_timespec(request->nsec, &ts);
    ret = clock_settime(CLOCK_REALTIME, &ts);

    return ret ? GB_OP_UNKNOWN_ERROR : GB_OP_SUCCESS;
#else
    return GB_OP_UNKNOWN_ERROR;
#endif
}

static uint8_t mb_control_get_pwrup_reason(struct gb_operation *operation)
{
    struct mb_control_get_pwrup_response *response;

    if (ctrl_info->host_major < MB_CONTROL_VERSION_PWRUP_REASON_MAJOR) {
        gb_error("%s() failed protocol version check!\n", __func__);
        return GB_OP_PROTOCOL_BAD;
    }
    if (ctrl_info->host_minor < MB_CONTROL_VERSION_PWRUP_REASON_MINOR &&
          ctrl_info->host_major == MB_CONTROL_VERSION_PWRUP_REASON_MAJOR)
    {
        gb_error("%s() failed protocol minor version check!\n", __func__);
        return GB_OP_PROTOCOL_BAD;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->reason = cpu_to_le32(PWRUP_REASON_NORMAL);

    return GB_OP_SUCCESS;
}

static uint8_t mb_control_set_current_limit(struct gb_operation *operation)
{
    /* if driver exists, call it to notify of current limit */
    if (ctrl_info->pwr_limit_dev) {
        struct mb_control_current_limit_request *request =
                gb_operation_get_request_payload(operation);

        return device_pwr_limit_set_pwr_limit(ctrl_info->pwr_limit_dev, request->limit);
    }

    return GB_OP_SUCCESS;
}

#if defined(CONFIG_DEVICE_CORE) && defined(CONFIG_PWR_LIMIT)
static int mb_control_capability_change_cb(struct device *dev,
        uint8_t level,
        uint8_t reason,
        uint16_t vendor)
{
    struct mb_control_capability_change_request *request;
    struct gb_operation *operation;
    int ret;

    operation = gb_operation_create(ctrl_info->cport,
            MB_CONTROL_TYPE_CAPABLITY_CHANGED, sizeof(*request));
    if (!operation)
        return -ENOMEM;

    request = gb_operation_get_request_payload(operation);
    if (!request) {
        gb_operation_destroy(operation);
        return -EINVAL;
    }
    request->level = level;
    request->reason = reason;
    request->vendor = cpu_to_le16(vendor);

    ret = gb_operation_send_request(operation, NULL, false);
    if (ret)
        ret = -EIO;

    gb_operation_destroy(operation);

    return ret;
}
#endif

# ifdef CONFIG_PWR_LIMIT
/*
 * Notify the Core that we will be using high power.  Currently (no
 * pun intended) this means greater than 1A.
 */
static int mb_control_current_rsv_cb(struct device *dev, uint8_t rsv)
{
    struct mb_control_current_rsv_request *request;
    struct gb_operation *operation;
    int ret;

    operation = gb_operation_create(ctrl_info->cport,
            MB_CONTROL_TYPE_CURRENT_RSV, sizeof(*request));
    if (!operation)
        return -ENOMEM;

    request = gb_operation_get_request_payload(operation);
    if (!request) {
        gb_operation_destroy(operation);
        return -EINVAL;
    }

    request->rsv = rsv;

    ret = gb_operation_send_request(operation, NULL, false);
    if (ret)
        ret = -EIO;

    gb_operation_destroy(operation);

    return ret;

}
#endif

static uint8_t mb_control_current_rsv_ack(struct gb_operation *operation)
{
# ifdef CONFIG_PWR_LIMIT
    /* if driver exists, call it to notify of current limit */
    if (ctrl_info->pwr_limit_dev) {
        struct mb_control_current_rsv_request *request =
                gb_operation_get_request_payload(operation);

        device_pwr_limit_current_rsv_ack(ctrl_info->pwr_limit_dev,
                request->rsv);
    }
#endif
    return GB_OP_SUCCESS;
}

static uint8_t mb_control_test_mode(struct gb_operation *operation)
{
    struct mb_control_test_mode_request *request =
        gb_operation_get_request_payload(operation);


    if (le32_to_cpu(request->value) & MODS_TEST_MODE_STATUS_ENABLED_BIT) {
        /* TODO: No current implementation */
    } else {
        /* TODO: No current implementation */
    }

    return GB_OP_SUCCESS;
}

static int mb_control_init(unsigned int cport)
{
    ctrl_info = zalloc(sizeof(*ctrl_info));
    if (!ctrl_info) {
        return -ENOMEM;
    }

    ctrl_info->cport = cport;

#ifdef CONFIG_DEVICE_CORE
    /* see if this device functions as the power control for additional */
    /* devices.  Not having one is not an error.                        */
    ctrl_info->slv_pwr_dev = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW, 0);
    if (ctrl_info->slv_pwr_dev) {
        if (device_slave_pwrctrl_register_callback(ctrl_info->slv_pwr_dev,
                                           mb_control_send_slave_state_cb)) {
            gb_debug("failed to register callback for %s device!\n",
                    DEVICE_TYPE_SLAVE_PWRCTRL_HW);
        }
    } else {
        gb_debug("Failed to open SLAVE Power Control\n");
    }

# ifdef CONFIG_PWR_LIMIT
    ctrl_info->pwr_limit_dev = device_open(DEVICE_TYPE_PWR_LIMIT_HW, 0);
    if (ctrl_info->pwr_limit_dev) {
        int ret = device_pwr_limit_register_capability_change_cb(
                ctrl_info->pwr_limit_dev, mb_control_capability_change_cb);
        if (ret) {
            gb_info("Failed to register callback for %s device!\n",
                ctrl_info->pwr_limit_dev->name);
        }

        ret = device_pwr_limit_register_current_rsv(
                ctrl_info->pwr_limit_dev, mb_control_current_rsv_cb);
        if (ret) {
            gb_info("Failed to register callback for %s device!\n",
                ctrl_info->pwr_limit_dev->name);
        }
    } else {
        gb_info("Failed to open Power Limit Device\n");
    }
# endif
#endif

    return 0;
}

static void mb_control_exit(unsigned int cport)
{
    if (!ctrl_info)
        return;

#ifdef CONFIG_DEVICE_CORE
    if (ctrl_info->slv_pwr_dev) {
        device_close(ctrl_info->slv_pwr_dev);
        device_slave_pwrctrl_unregister_callback(ctrl_info->slv_pwr_dev);
    }
# ifdef CONFIG_PWR_LIMIT
    if (ctrl_info->pwr_limit_dev) {
        (void)device_pwr_limit_unregister_capability_change_cb(
                ctrl_info->pwr_limit_dev);
        device_close(ctrl_info->pwr_limit_dev);
    }
# endif
#endif

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
    GB_HANDLER(MB_CONTROL_TYPE_GET_ROOT_VER, mb_control_get_root_vers),
    GB_HANDLER(MB_CONTROL_TYPE_RTC_SYNC, mb_control_rtc_sync),
    GB_HANDLER(MB_CONTROL_TYPE_SET_CURRENT_LIMIT, mb_control_set_current_limit),
    GB_HANDLER(MB_CONTROL_TYPE_GET_PWRUP_REASON, mb_control_get_pwrup_reason),
    GB_HANDLER(MB_CONTROL_TYPE_CURRENT_RSV_ACK, mb_control_current_rsv_ack),
    GB_HANDLER(MB_CONTROL_TYPE_TEST_MODE, mb_control_test_mode),
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
