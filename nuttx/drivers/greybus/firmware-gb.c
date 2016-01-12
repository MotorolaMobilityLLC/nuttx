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

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <nuttx/arch.h>
#include <arch/byteorder.h>
#include <nuttx/bootmode.h>
#include <nuttx/clock.h>
#include <nuttx/progmem.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/manifest.h>
#include <nuttx/wqueue.h>

#define GB_FIRMWARE_TFTF_HDR_SIZE         512

/* Version of the Greybus firmware protocol we support */
#define GB_FIRMWARE_VERSION_MAJOR         0x00
#define GB_FIRMWARE_VERSION_MINOR         0x01

/* Greybus firmware request types */
#define GB_FIRMWARE_TYPE_INVALID          0x00
#define GB_FIRMWARE_TYPE_PROTOCOL_VERSION 0x01
#define GB_FIRMWARE_TYPE_FIRMWARE_SIZE    0x02
#define GB_FIRMWARE_TYPE_GET_FIRMWARE     0x03
#define GB_FIRMWARE_TYPE_READY_TO_BOOT    0x04
#define GB_FIRMWARE_TYPE_AP_READY         0x05

/* Greybus firmware boot stages */
#define GB_FIRMWARE_BOOT_STAGE_ONE        0x01 /* Reserved for the boot ROM */
#define GB_FIRMWARE_BOOT_STAGE_TWO        0x02 /* Firmware package to be loaded by the boot ROM */
#define GB_FIRMWARE_BOOT_STAGE_THREE      0x03 /* Module personality package loaded by Stage 2 firmware */

/* Greybus firmware ready to boot status */
#define GB_FIRMWARE_BOOT_STATUS_INVALID   0x00 /* Firmware blob could not be validated */
#define GB_FIRMWARE_BOOT_STATUS_INSECURE  0x01 /* Firmware blob is valid but insecure */
#define GB_FIRMWARE_BOOT_STATUS_SECURE    0x02 /* Firmware blob is valid and secure */

/* Max firmware data fetch size in bytes */
#define GB_FIRMWARE_FETCH_MAX             2000

/* version request has no payload */
struct gb_firmware_proto_version_response {
    __u8      major;
    __u8      minor;
};

/* Firmware protocol firmware size request/response */
struct gb_firmware_size_request {
    __u8      stage;
} __packed;

struct gb_firmware_size_response {
    __le32    size;
} __packed;

/* Firmware protocol get firmware request/response */
struct gb_firmware_get_firmware_request {
    __le32    offset;
    __le32    size;
} __packed;

struct gb_firmware_get_firmware_response {
    __u8      data[0];
} __packed;

/* Firmware protocol Ready to boot request */
struct gb_firmware_ready_to_boot_request {
    __u8      status;
} __packed;
/* Firmware protocol Ready to boot response has no payload */

#define TFTF_NUM_SECTIONS                     (20)
#define TFTF_SECTION_TYPE_RAW_CODE        (0x0001)
#define TFTF_SECTION_TYPE_RAW_DATA        (0x0002)
#define TFTF_SECTION_TYPE_COMPRESSED_CODE (0x0003)
#define TFTF_SECTION_TYPE_COMPRESSED_DATA (0x0004)
#define TFTF_SECTION_TYPE_MANIFEST        (0x0005)
#define TFTF_SECTION_TYPE_SIGNATURE       (0x0006)
#define TFTF_SECTION_TYPE_CERTIFICATE     (0x0007)
#define TFTF_SECTION_TYPE_END             (0x00fe)

struct section_descriptor {
    uint8_t section_type;
    uint8_t section_class[3];
    uint32_t section_id;
    uint32_t section_length;
    uint32_t section_load_address;
    uint32_t section_expanded_length;
} __packed;

/* There is some useful information in the tftf header that we may want to */
/* save off for the future.  This is just a stand in for that information  */
/* since header_size is probably the least useful field (it is a fixed     */
/* value)                                                                  */
struct gb_firmware_tftf_header {
    char sentinel_value[4];
    uint32_t header_size;
    uint8_t build_timestamp[16];
    char firmware_package_name[48];
    uint32_t package_type;
    uint32_t start_location;
    uint32_t unipro_vid;
    uint32_t unipro_pid;
    uint32_t ara_vid;
    uint32_t ara_pid;
    uint8_t reserved[16];
    struct section_descriptor desc[TFTF_NUM_SECTIONS];
} __packed;

#ifdef CONFIG_GREYBUS_DEBUG
static inline void dump_header(struct gb_firmware_tftf_header *hdr)
{
    int d;

    gb_debug("sentinel_value           = %s\n", hdr->sentinel_value);
    gb_debug("header_size              = %u\n", hdr->header_size);
    gb_debug("build_timestamp          = %c%c\n", hdr->build_timestamp[0], hdr->build_timestamp[1]);
    gb_debug("firmware_package_name    = %s\n", hdr->firmware_package_name);
    gb_debug("package_type             = 0x%08x\n", hdr->package_type);
    gb_debug("start_location           = 0x%08x\n", hdr->start_location);
    gb_debug("unipro_vid               = 0x%08x\n", hdr->unipro_vid);
    gb_debug("unipro_pid               = 0x%08x\n", hdr->unipro_pid);
    gb_debug("ara_vid                  = 0x%08x\n", hdr->ara_vid);
    gb_debug("ara_pid                  = 0x%08x\n", hdr->ara_pid);
    for (d = 0; d < TFTF_NUM_SECTIONS; d++) {
        if (hdr->desc[d].section_type == TFTF_SECTION_TYPE_END)
            break;

        gb_debug("  section_type           = 0x%02x\n", hdr->desc[d].section_type);
        gb_debug("  section_class          = 0x%02x%02x%02x\n",
                                                hdr->desc[d].section_class[0],
                                                hdr->desc[d].section_class[1],
                                                hdr->desc[d].section_class[2]);
        gb_debug("  section_id             = 0x%08x\n", hdr->desc[d].section_id);
        gb_debug("  section_length         = 0x%08x\n", hdr->desc[d].section_length);
        gb_debug("  section_load_address   = 0x%08x\n", hdr->desc[d].section_load_address);
        gb_debug("  section_expaned_length = 0x%08x\n", hdr->desc[d].section_expanded_length);
    };
}
#else
# define dump_header(hdr)
#endif

#define GB_FIRMWARE_FLASH_DELAY_MS 1000

#if !defined(CONFIG_SCHED_WORKQUEUE) || !defined(CONFIG_SCHED_LPWORK)
# error  "requires low priority workqueue"
#endif

struct gb_firmware_info {
    uint32_t cport;
    uint32_t base_address;
    uint32_t chunk_size;
    uint32_t firmware_size;
    struct work_s flash_work;
    struct work_s reset_work;
};

static struct gb_firmware_info *g_firmware_info = NULL;

static uint8_t gb_firmware_protocol_version(struct gb_operation *operation)
{
    struct gb_firmware_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_FIRMWARE_VERSION_MAJOR;
    response->minor = GB_FIRMWARE_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

static int gb_firmware_size(uint8_t stage, size_t *size)
{
    struct gb_firmware_size_request *request;
    struct gb_firmware_size_response *response;
    struct gb_operation *operation;
    struct gb_operation *response_op;
    uint8_t result;
    int ret = -EFAULT;

    *size = 0;

    operation = gb_operation_create(g_firmware_info->cport,
            GB_FIRMWARE_TYPE_FIRMWARE_SIZE, sizeof(*request));
    if (!operation) {
        gb_error("failed to create operation\n");
        return -ENOMEM;
    }

    request = gb_operation_get_request_payload(operation);
    if (!request) {
        gb_error("failed to get request payload\n");
        gb_operation_destroy(operation);
        return -EIO;
    }

    request->stage = stage;

    if (gb_operation_send_request_sync(operation) !=  GB_OP_SUCCESS) {
        gb_error("failed to send firmware size request\n");
        gb_operation_destroy(operation);
        return -EIO;
    }

    result = gb_operation_get_response_result(operation);
    if (result != GB_OP_SUCCESS) {
        gb_error("Invalid response got result %d\n", result);
        ret = -EINVAL;
        goto out;
    }

    response_op = gb_operation_get_response_op(operation);
    if (response_op) {
        response = gb_operation_get_request_payload(response_op);
        g_firmware_info->firmware_size = le32_to_cpu(response->size);
        *size = le32_to_cpu(response->size);
        ret = 0;
    }

out:
    gb_operation_destroy(operation);

    return ret;
}

static int gb_firmware_setup_flash(size_t load_address, size_t size)
{
    ssize_t page_start;
    ssize_t page_end;
    size_t page;
    size_t page_size;
    ssize_t size_erased;
    int ret = 0;

    /* determine the address range we will be flashing and
     * make sure that is valid.
     */
    page_start = up_progmem_getpage(load_address);
    if (page_start < 0) {
      gb_error("attempt to flash invalid address 0x%08x\n",
          g_firmware_info->base_address);
      return page_start;
    }

    page_end = up_progmem_getpage(load_address + size);
    if (page_end < 0) {
      gb_error("attempt to flash invalid address 0x%08x\n",
            load_address + size);
      return page_end;
    }

    /* now lets erase the pages we need */
    for (page = page_start; page <= page_end; page++) {
        page_size = up_progmem_pagesize(page);

        size_erased = up_progmem_erasepage(page);
        if (size_erased != page_size) {
            gb_error("failed to erase page %u err = %d\n", page, size_erased);
            ret = (int)size_erased;
            break;
        }
    }

    return ret;
}

/* write a single chunk to flash */
static int gb_firmware_flash_chunk(size_t flash_address, uint8_t *data, size_t size)
{
    ssize_t err = 0;

    gb_debug("flash to 0x%08x %u bytes\n", flash_address, size);

    gb_dump(data, MIN(size, 32));

    err = up_progmem_write(flash_address, (const void *)data, size);
    err = (err < 0) ? err : 0;     /* convert from count to error flag */
    if (err) {
        gb_error("failed to write address 0x%08x err = %d\n", flash_address, err);
    }

    return (int)err;
}

static int gb_firmware_get_header(
        size_t firmware_size,
        struct gb_firmware_tftf_header *hdr)
{
    struct gb_operation *operation;
    struct gb_firmware_get_firmware_request *request;
    struct gb_firmware_get_firmware_response *response;
    int ret = -ENOENT;

    if (firmware_size < GB_FIRMWARE_TFTF_HDR_SIZE)
        goto out;

    operation = gb_operation_create(g_firmware_info->cport,
            GB_FIRMWARE_TYPE_GET_FIRMWARE, sizeof(*request));
    if (!operation) {
        ret = -ENOMEM;
        goto out;
    }

    request = (struct gb_firmware_get_firmware_request *)
    gb_operation_get_request_payload(operation);
    if (!request) {
        ret = -EIO;
        goto cleanup;
    }

    request->offset = 0;
    request->size = GB_FIRMWARE_TFTF_HDR_SIZE;

    ret = gb_operation_send_request_sync(operation);
    if (ret != GB_OP_SUCCESS) {
        gb_error("failed to send firmware request\n");
        ret = -EIO;
        goto cleanup;
    }

    if (!operation->response) {
        gb_error("No firmware received\n");
        gb_operation_destroy(operation);
        ret = -EIO;
        goto cleanup;
    }

    response = gb_operation_get_request_payload(operation->response);

    if (!memcmp(response->data, "TFTF", 4)) {
        memcpy(hdr, response->data, sizeof(*hdr));
        dump_header(hdr);
        ret = 0;
    } else
        ret = -ENOENT;

cleanup:
    gb_operation_destroy(operation);
out:
    return ret;
}

static int gb_firmware_get_firmware(size_t size)
{
    struct gb_operation *operation;
    struct gb_firmware_get_firmware_request *request;
    struct gb_firmware_get_firmware_response *response;
    ssize_t remaining;
    uint32_t fetch_offset;
    uint32_t write_offset;
    struct gb_firmware_tftf_header *hdr;
    int ret;

    hdr = zalloc(sizeof(*hdr));
    if (!hdr) {
        gb_error("Failed to allocate memory for header\n");
        return -ENOMEM;
    }

    ret = gb_firmware_get_header(size, hdr);
    if (!ret) {
        /* Header found */
        int section;
        bool found_code_section = false;

        fetch_offset = hdr->header_size;
        remaining = size - fetch_offset;

        /* Use the load address from the header if it is available          */
        for (section = 0; section < TFTF_NUM_SECTIONS; section++) {
            uint8_t section_type = hdr->desc[section].section_type;

            if (section_type == TFTF_SECTION_TYPE_END) {
                break;
            } else if (section_type == TFTF_SECTION_TYPE_RAW_CODE) {
                write_offset = hdr->desc[section].section_load_address;
                remaining = hdr->desc[section].section_length;
                found_code_section = true;
                break;
            } else {
                fetch_offset += hdr->desc[section].section_length;
            }
        }

        if (!found_code_section) {
            ret = -EINVAL;
            goto out;
        }
    } else if (ret == -ENOENT) {
        /* No Header found, currently this is okay */
        fetch_offset = 0;
        remaining = size;
        write_offset = g_firmware_info->base_address;
    } else
        goto out;

    ret = gb_bootmode_set(BOOTMODE_FLASHING);
    if (ret) {
        gb_error("failed to write FLASHING barker\n");
        goto out;
    }

    /* erase program memory */
    ret = gb_firmware_setup_flash(write_offset, remaining);
    if (ret)
        goto out;

    while (remaining > 0) {
        operation = gb_operation_create(g_firmware_info->cport,
                GB_FIRMWARE_TYPE_GET_FIRMWARE, sizeof(*request));
        if (!operation) {
            ret = -ENOMEM;
            break;
        }

        request = gb_operation_get_request_payload(operation);
        if (!request) {
            gb_operation_destroy(operation);
            ret = -EIO;
            break;
        }

        request->offset = fetch_offset;
        request->size = MIN(g_firmware_info->chunk_size, remaining);

        ret = gb_operation_send_request_sync(operation);
        if (ret != GB_OP_SUCCESS) {
            gb_error("failed to send firmware request\n");
            gb_operation_destroy(operation);
            ret = -EIO;
            break;
        }

        if (!operation->response) {
            gb_error("No firmware received\n");
            gb_operation_destroy(operation);
            ret = -EIO;
            break;
        }

        response = gb_operation_get_request_payload(operation->response);

        ret = gb_firmware_flash_chunk(write_offset, response->data, request->size);
        if (ret) {
            /* well this isn't good!  what can we really do */
            gb_error("FLASHING FAILED!!!\n")
            gb_operation_destroy(operation);
            break;
        }

        fetch_offset += request->size;
        write_offset += request->size;
        remaining -= request->size;
        gb_debug("remaining bytes = %d\n", remaining);

        gb_operation_destroy(operation);
    }

    if (!ret) {
        ret = gb_bootmode_set(BOOTMODE_NORMAL);
    }

out:
    free(hdr);
    return ret;
}

static void gb_firmware_reset_worker(FAR void *arg)
{
#ifdef CONFIG_ARCH_HAVE_SYSRESET
    up_systemreset(); /* will not return */
#endif
}

static int gb_firmware_ready_to_boot(uint8_t stage, uint8_t status)
{
    struct gb_firmware_ready_to_boot_request *request;
    struct gb_operation *operation;
    int ret;

    operation = gb_operation_create(g_firmware_info->cport,
            GB_FIRMWARE_TYPE_READY_TO_BOOT, sizeof(*request));
    if (!operation)
        return -ENOMEM;

    request = (struct gb_firmware_ready_to_boot_request *)
            gb_operation_get_request_payload(operation);
    request->stage = stage;
    request->status = status;

    ret = gb_operation_send_request_sync(operation);
    if (ret) {
        gb_error("failed to send boot ready request\n");
        gb_operation_destroy(operation);
        return ret;
    }

    gb_operation_destroy(operation);

    /* cancel any work and reset ourselves */
    if (!work_available(&g_firmware_info->reset_work))
        work_cancel(LPWORK, &g_firmware_info->reset_work);

    work_queue(LPWORK, &g_firmware_info->reset_work,
            gb_firmware_reset_worker, NULL,
            MSEC2TICK(GB_FIRMWARE_FLASH_DELAY_MS));

    return 0;
}

/*
 * main flashing algorithm.  We will check the signatures on booting
 * so just send up that the everything is secure if flashing works.
 */

static void gb_firmware_worker(FAR void *arg)
{
    int err;
    size_t firmware_size = 0;  /* initialize to prevent spurious warning */
    uint8_t status = GB_FIRMWARE_BOOT_STATUS_SECURE;
    uint8_t stage;
    uint8_t last_stage_attempted = GB_FIRMWARE_BOOT_STAGE_ONE;

    for (stage = GB_FIRMWARE_BOOT_STAGE_ONE;
         stage <= GB_FIRMWARE_BOOT_STAGE_THREE; stage++) {

        last_stage_attempted = stage;
        err = gb_firmware_size(stage, &firmware_size);
        if (err) {
            gb_error("failed to get firmware size %d\n", err);
            continue;
        }

        if (firmware_size == 0) {
            gb_error("Refusing to flash firmware size of 0\n")
            continue;
        }

        err = gb_firmware_get_firmware(firmware_size);
        if (err) {
            gb_error("failed to get firmware\n");
            status = GB_FIRMWARE_BOOT_STATUS_INVALID;
            break;
        }
    }

    /* in all cases, send up the ready to boot.  At least then
     * we can try again.
     */
    err = gb_firmware_ready_to_boot(last_stage_attempted, status);
    if (err) {
        gb_error("failed to send ready to boot\n");
    }
}

/*
 * called from the AP to kick the whole process off.  This is
 * described as temporary on the AP side.
 */
static uint8_t gb_firmware_ap_ready(struct gb_operation *operation)
{
    if (!work_available(&g_firmware_info->flash_work))
        work_cancel(LPWORK, &g_firmware_info->flash_work);

    /* Kick off the flashing, but delay long enough to allow the
     * response to this message to be received or timeout.
     * (either one works).
     */
    work_queue(LPWORK, &g_firmware_info->flash_work,
            gb_firmware_worker, NULL,
            MSEC2TICK(GB_FIRMWARE_FLASH_DELAY_MS));
    return GB_OP_SUCCESS;
}

static int gb_firmware_init(unsigned int cport)
{
    g_firmware_info = zalloc(sizeof(*g_firmware_info));
    if (!g_firmware_info)
        return -ENOMEM;

    g_firmware_info->cport = cport;
    g_firmware_info->base_address = CONFIG_FIRMWARE_FLASH_ADDRESS;
    g_firmware_info->chunk_size =
        MIN(CONFIG_FIRMWARE_CHUNK_SIZE, GB_FIRMWARE_FETCH_MAX);
    g_firmware_info->firmware_size = 0;

    return 0;
}

static void gb_firmware_exit(unsigned int cport)
{
    if (g_firmware_info) {
        work_cancel(LPWORK, &g_firmware_info->flash_work);
        work_cancel(LPWORK, &g_firmware_info->reset_work);
        free(g_firmware_info);
    }
    g_firmware_info = NULL;
}

static struct gb_operation_handler gb_firmware_handlers[] = {
    GB_HANDLER(GB_FIRMWARE_TYPE_PROTOCOL_VERSION, gb_firmware_protocol_version),
    GB_HANDLER(GB_FIRMWARE_TYPE_AP_READY, gb_firmware_ap_ready),
};

struct gb_driver firmware_driver = {
    .init = gb_firmware_init,
    .exit = gb_firmware_exit,
    .op_handlers = (struct gb_operation_handler*) gb_firmware_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_firmware_handlers),
};

void gb_firmware_register(int cport)
{
    gb_register_driver(cport, &firmware_driver);
}
