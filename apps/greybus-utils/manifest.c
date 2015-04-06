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

#include <arch/tsb/unipro.h>
#include <apps/greybus-utils/utils.h>

#include "svc_msg.h"
#include "greybus_manifest.h"

struct gb_cport {
    int id;
    int protocol;
};

extern void gb_gpio_register(int cport);
extern void gb_i2c_register(int cport);
extern void gb_battery_register(int cport);
extern void gb_loopback_register(int cport);

struct greybus {
    /* TODO use a list instead */
    int cports_bmp;
    struct gb_cport cports[CPORT_MAX];
    struct greybus_driver *drv;
};

struct greybus g_greybus;

unsigned char *manifest_files[] = { MANIFEST };

void foreach_manifest(void (manifest_handler)
                       (unsigned char *manifest_file, int manifest_number))
{
    int i = 0;
    for (i = 0; i < ARRAY_SIZE(manifest_files); i++)
        manifest_handler(manifest_files[i], i);
}

static void *alloc_cport(void)
{
    int i = 0;
    while (i < CPORT_MAX) {
        if ((g_greybus.cports_bmp & (1 << i)) == 0) {
            g_greybus.cports_bmp |= 1 << i;
            return &g_greybus.cports[i];
        }
        i++;
    }
    return NULL;
}

static void free_cport(int cportid)
{
    g_greybus.cports_bmp &= ~(1 << cportid);
}

void enable_cports(void)
{
    int i = 0;
    __attribute__((unused)) int id;
    __attribute__((unused)) int protocol;
    while (i < CPORT_MAX) {
        if (g_greybus.cports_bmp & (1 << i)) {
            id = g_greybus.cports[i].id;
            protocol = g_greybus.cports[i].protocol;

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

#ifdef CONFIG_GREYBUS_BATTERY_PHY
            if (protocol == GREYBUS_PROTOCOL_BATTERY) {
                gb_info("Registering BATTERY greybus driver.\n");
                gb_battery_register(id);
            }
#endif

#ifdef CONFIG_GREYBUS_LOOPBACK_PHY
            if (protocol == GREYBUS_PROTOCOL_LOOPBACK) {
                gb_info("Registering Loopback greybus driver.\n");
                gb_loopback_register(id);
            }
#endif
        }
        i++;
    }
}

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
    int desc_size;
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

    switch (desc_header->type) {
    case GREYBUS_TYPE_MODULE:
        if (desc_size < sizeof(struct greybus_descriptor_module)) {
            gb_error("module descriptor too small (%u)\n", desc_size);
            return -EINVAL;
        }
        break;
    case GREYBUS_TYPE_STRING:
        expected_size = sizeof(struct greybus_descriptor_header);
        expected_size += sizeof(struct greybus_descriptor_string);
        expected_size += (size_t) desc->string.length;
        if (desc_size < expected_size) {
            gb_error("string descriptor too small (%u)\n", desc_size);
            return -EINVAL;
        }
        break;
    case GREYBUS_TYPE_INTERFACE:
        break;
    case GREYBUS_TYPE_CPORT:
        if (desc_size < sizeof(struct greybus_descriptor_cport)) {
            gb_error("cport descriptor too small (%u)\n", desc_size);
            return -EINVAL;
        }
        if (!release) {
            cport = alloc_cport();
            cport->id = desc->cport.id;
            cport->protocol = desc->cport.protocol;
            gb_debug("cport_id = %d\n", cport->id);
        } else {
            free_cport(desc->cport.id);
        }
        break;
    case GREYBUS_TYPE_CLASS:
        gb_debug("class descriptor found (ignoring)\n");
        break;
    case GREYBUS_TYPE_INVALID:
    default:
        gb_error("invalid descriptor type (%hhu)\n", desc_header->type);
        return -EINVAL;
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
        if (desc_size <= 0) {
            if (!desc_size)
                gb_error("zero-sized manifest descriptor\n");
            return false;
        }
        desc = (struct greybus_descriptor *)((char *)desc + desc_size);
        size -= desc_size;
    }

    return true;
}

/*
 * Parse a buffer containing a module manifest.
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

char *get_manifest_blob(void *data)
{
    uint16_t size;
    char *hpb;
    struct greybus_manifest_header *mh;

    if (!data)
        data = manifest_files[0];

    memcpy(&size, data, 2);
    if (!(hpb = malloc(HP_BASE_SIZE + size))) {
        gb_error("failed to allocate hotplug buffer\n");
        goto out;
    }

    mh = (struct greybus_manifest_header *)(hpb + HP_BASE_SIZE);
    memcpy(mh, data, size);

 out:
    return hpb;
}

void parse_manifest_blob(char *hpe)
{
    struct greybus_manifest_header *mh =
        (struct greybus_manifest_header *)(hpe + HP_BASE_SIZE);

    manifest_parse(mh, le16toh(mh->size));
}

void enable_manifest(char *name, void *priv)
{
    char *hpe;

    hpe = get_manifest_blob(priv);
    if (hpe) {
        parse_manifest_blob(hpe);
        int iid = get_interface_id(name);
        if (iid > 0) {
            gb_info("%s interface inserted\n", name);
            free(hpe);
        } else {
            gb_error("invalid interface ID, no hotplug plug event sent\n");
        }
    } else {
        gb_error("missing manifest blob, no hotplug event sent\n");
    }
}
