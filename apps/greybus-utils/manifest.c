/*
 * Copyright (c) 2014-2015 Google Inc.
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>

#include <arch/byteorder.h>
#include <apps/greybus-utils/utils.h>
#include <nuttx/util.h>

#include "svc_msg.h"
#include "greybus_manifest.h"

extern void gb_control_register(int cport);
extern void gb_gpio_register(int cport);
extern void gb_i2c_register(int cport);
extern void gb_battery_register(int cport);
extern void gb_loopback_register(int cport);
extern void gb_vibrator_register(int cport);
extern void gb_usb_register(int cport);
extern void gb_pwm_register(int cport);
extern void gb_i2s_mgmt_register(int cport);
extern void gb_i2s_receiver_register(int cport);
extern void gb_i2s_transmitter_register(int cport);
extern void gb_spi_register(int cport);
extern void gb_uart_register(int cport);
extern void gb_hid_register(int cport);
extern void gb_lights_register(int cport);

struct greybus {
    struct list_head cports;
    struct greybus_driver *drv;
};

struct greybus g_greybus = {
    .cports = LIST_INIT(g_greybus.cports),
};

struct manifest_file manifest_files[] = { MANIFEST };
static int g_device_id;

void foreach_manifest(manifest_handler handler)
{
    int i = 0;
    for (i = 0; i < ARRAY_SIZE(manifest_files); i++)
        handler(manifest_files[i].bin, manifest_files[i].id, i);
}

static void *alloc_cport(void)
{
    struct gb_cport *gb_cport;

    gb_cport = malloc(sizeof(struct gb_cport));
    if (!gb_cport)
        return NULL;

    list_add(&g_greybus.cports, &gb_cport->list);
    return gb_cport;
}

static void free_cport(int cportid)
{
    struct gb_cport *gb_cport;
    struct list_head *iter, *next;
    list_foreach_safe(&g_greybus.cports, iter, next) {
        gb_cport = list_entry(iter, struct gb_cport, list);
        if (gb_cport->id == cportid) {
            list_del(iter);
            free(gb_cport);
        }
    }
}

#ifdef CONFIG_GREYBUS
void enable_cports(void)
{
    struct list_head *iter;
    struct gb_cport *gb_cport;
    __attribute__((unused)) int id;
    __attribute__((unused)) int protocol;
    list_foreach(&g_greybus.cports, iter) {
        gb_cport = list_entry(iter, struct gb_cport, list);
        id = gb_cport->id;
        protocol = gb_cport->protocol;

#ifdef CONFIG_GREYBUS_CONTROL_PROTOCOL
        if (protocol == GREYBUS_PROTOCOL_CONTROL) {
            gb_info("Registering CONTROL greybus driver.\n");
            gb_control_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_GPIO_PHY
        if (protocol == GREYBUS_PROTOCOL_GPIO) {
            gb_info("Registering GPIO greybus driver.\n");
            gb_gpio_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_I2C_PHY
        if (protocol == GREYBUS_PROTOCOL_I2C) {
            gb_info("Registering I2C greybus driver.\n");
            gb_i2c_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_BATTERY
        if (protocol == GREYBUS_PROTOCOL_BATTERY) {
            gb_info("Registering BATTERY greybus driver.\n");
            gb_battery_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_LOOPBACK
        if (protocol == GREYBUS_PROTOCOL_LOOPBACK) {
            gb_info("Registering Loopback greybus driver.\n");
            gb_loopback_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_VIBRATOR
        if (protocol == GREYBUS_PROTOCOL_VIBRATOR) {
            gb_info("Registering VIBRATOR greybus driver.\n");
            gb_vibrator_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_USB_HOST_PHY
        if (protocol == GREYBUS_PROTOCOL_USB) {
            gb_info("Registering USB greybus driver.\n");
            gb_usb_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_PWM_PHY
        if (protocol == GREYBUS_PROTOCOL_PWM) {
            gb_info("Registering PWM greybus driver.\n");
            gb_pwm_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_I2S_PHY
        if (protocol == GREYBUS_PROTOCOL_I2S_MGMT) {
            gb_info("Registering I2S MGMT greybus driver.\n");
            gb_i2s_mgmt_register(id);
        } else if (protocol == GREYBUS_PROTOCOL_I2S_RECEIVER) {
            gb_info("Registering I2S RECEIVER greybus driver.\n");
            gb_i2s_receiver_register(id);
        } else if (protocol == GREYBUS_PROTOCOL_I2S_TRANSMITTER) {
            gb_info("Registering I2S TRANSMITTER greybus driver.\n");
            gb_i2s_transmitter_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_SPI_PHY
        if (protocol == GREYBUS_PROTOCOL_SPI) {
            gb_info("Registering SPI greybus driver.\n");
            gb_spi_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_UART_PHY
        if (protocol == GREYBUS_PROTOCOL_UART) {
            gb_info("Registering Uart greybus driver. id= %d\n", id);
            gb_uart_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_HID
        if (protocol == GREYBUS_PROTOCOL_HID) {
            gb_info("Registering HID greybus driver. id= %d\n", id);
            gb_hid_register(id);
        }
#endif

#ifdef CONFIG_GREYBUS_LIGHTS
        if (protocol == GREYBUS_PROTOCOL_LIGHTS) {
            gb_info("Registering Lights greybus driver. id= %d\n", id);
            gb_lights_register(id);
        }
#endif
    }
}
#endif

/*
 * Validate the given descriptor.  Its reported size must fit within
 * the number of bytes reamining, and it must have a recognized
 * type.  Check that the reported size is at least as big as what
 * we expect to see.  (It could be bigger, perhaps for a new version
 * of the format.)
 *
 * Returns the number of bytes consumed by the descriptor, or a
 * negative errno.
 */
static int identify_descriptor(struct greybus_descriptor *desc, size_t size,
                               int release)
{
    struct greybus_descriptor_header *desc_header = &desc->header;
    size_t expected_size;
    size_t desc_size;
    struct gb_cport *cport;

    if (size < sizeof(*desc_header)) {
        gb_error("manifest too small\n");
        return -EINVAL;         /* Must at least have header */
    }

    desc_size = (int)le16toh(desc_header->size);
    if ((size_t) desc_size > size) {
        gb_error("descriptor too big\n");
        return -EINVAL;
    }

    /* Descriptor needs to at least have a header */
    expected_size = sizeof(*desc_header);

    switch (desc_header->type) {
    case GREYBUS_TYPE_STRING:
        expected_size += sizeof(struct greybus_descriptor_string);
        expected_size += desc->string.length;

        /* String descriptors are padded to 4 byte boundaries */
        expected_size = ALIGN(expected_size);
        break;
    case GREYBUS_TYPE_INTERFACE:
        expected_size += sizeof(struct greybus_descriptor_interface);
        break;
    case GREYBUS_TYPE_BUNDLE:
        expected_size += sizeof(struct greybus_descriptor_bundle);
        break;
    case GREYBUS_TYPE_CPORT:
        expected_size += sizeof(struct greybus_descriptor_cport);
        if (desc_size >= expected_size) {
            if (!release) {
                cport = alloc_cport();
                cport->id = desc->cport.id;
                cport->protocol = desc->cport.protocol_id;
                cport->device_id = g_device_id;
                gb_debug("cport_id = %d\n", cport->id);
            } else {
                free_cport(desc->cport.id);
            }
        }
        break;
    case GREYBUS_TYPE_INVALID:
    default:
        gb_error("invalid descriptor type (%hhu)\n", desc_header->type);
        return -EINVAL;
    }

    if (desc_size < expected_size) {
        gb_error("%d: descriptor too small (%zu < %zu)\n",
                 desc_header->type, desc_size, expected_size);
        return -EINVAL;
    }

    /* Descriptor bigger than what we expect */
    if (desc_size > expected_size) {
        gb_error("%d descriptor size mismatch (want %zu got %zu)\n",
                 desc_header->type, expected_size, desc_size);
    }

    return desc_size;
}

bool _manifest_parse(void *data, size_t size, int release)
{
    struct greybus_manifest *manifest = data;
    struct greybus_manifest_header *header = &manifest->header;
    struct greybus_descriptor *desc;
    uint16_t manifest_size;

    if (!release) {
        /* we have to have at _least_ the manifest header */
        if (size <= sizeof(manifest->header)) {
            gb_error("short manifest (%zu)\n", size);
            return false;
        }

        /* Make sure the size is right */
        manifest_size = le16toh(header->size);
        if (manifest_size != size) {
            gb_error("manifest size mismatch %zu != %hu\n", size,
                     manifest_size);
            return false;
        }

        /* Validate major/minor number */
        if (header->version_major > GREYBUS_VERSION_MAJOR) {
            gb_error("manifest version too new (%hhu.%hhu > %hhu.%hhu)\n",
                     header->version_major, header->version_minor,
                     GREYBUS_VERSION_MAJOR, GREYBUS_VERSION_MINOR);
            return false;
        }
    }

    /* OK, find all the descriptors */
    desc = (struct greybus_descriptor *)(header + 1);
    size -= sizeof(*header);
    while (size) {
        int desc_size;

        desc_size = identify_descriptor(desc, size, release);
        if (desc_size <= 0)
            return false;
        desc = (struct greybus_descriptor *)((char *)desc + desc_size);
        size -= desc_size;
    }

    return true;
}

/*
 * Parse a buffer containing a interface manifest.
 *
 * If we find anything wrong with the content/format of the buffer
 * we reject it.
 *
 * The first requirement is that the manifest's version is
 * one we can parse.
 *
 * We make an initial pass through the buffer and identify all of
 * the descriptors it contains, keeping track for each its type
 * and the location size of its data in the buffer.
 *
 * Returns true if parsing was successful, false otherwise.
 */
bool manifest_parse(void *data, size_t size)
{
    return _manifest_parse(data, size, 0);
}

bool manifest_release(void *data, size_t size)
{
    return _manifest_parse(data, size, 1);
}

static int get_interface_id(char *fname)
{
    char *iid_str;
    int iid = 0;
    char tmp[256];

    strcpy(tmp, fname);
    iid_str = strtok(tmp, "-");
    if (!strncmp(iid_str, "IID", 3))
        iid = strtol(fname + 4, NULL, 0);

    return iid;
}

void *get_manifest_blob(void)
{
    return manifest_files[0].bin;
}

void parse_manifest_blob(void *manifest)
{
    struct greybus_manifest_header *mh = manifest;

    manifest_parse(mh, le16toh(mh->size));
}

void release_manifest_blob(void *manifest)
{
    struct greybus_manifest_header *mh = manifest;

    manifest_release(mh, le16toh(mh->size));
}

void enable_manifest(char *name, void *manifest, int device_id)
{
    if (!manifest) {
        manifest = get_manifest_blob();
    }

    if (manifest) {
        g_device_id = device_id;
        parse_manifest_blob(manifest);
        int iid = get_interface_id(name);
        if (iid > 0) {
            gb_info("%s interface inserted\n", name);
        } else {
            gb_error("invalid interface ID, no hotplug plug event sent\n");
        }
    } else {
        gb_error("missing manifest blob, no hotplug event sent\n");
    }
}

void disable_manifest(char *name, void *priv, int device_id)
{
    void *manifest;

    manifest = get_manifest_blob();
    if (manifest) {
        g_device_id = device_id;
        release_manifest_blob(manifest);
    }
}

struct list_head *get_manifest_cports(void)
{
    return &g_greybus.cports;
}

int get_manifest_size(void)
{
    struct greybus_manifest_header *mh = get_manifest_blob();

    return mh ? le16_to_cpu(mh->size) : 0;
}
