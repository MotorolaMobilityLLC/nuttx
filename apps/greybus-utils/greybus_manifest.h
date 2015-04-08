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

#ifndef __GREYBUS_MANIFEST_H
#define __GREYBUS_MANIFEST_H

#pragma pack(push, 1)

enum greybus_descriptor_type {
    GREYBUS_TYPE_INVALID = 0x00,
    GREYBUS_TYPE_MODULE = 0x01,
    GREYBUS_TYPE_STRING = 0x02,
    GREYBUS_TYPE_INTERFACE = 0x03,
    GREYBUS_TYPE_CPORT = 0x04,
    GREYBUS_TYPE_CLASS = 0x05,
};

enum greybus_protocol {
    GREYBUS_PROTOCOL_CONTROL = 0x00,
    GREYBUS_PROTOCOL_AP = 0x01,
    GREYBUS_PROTOCOL_GPIO = 0x02,
    GREYBUS_PROTOCOL_I2C = 0x03,
    GREYBUS_PROTOCOL_UART = 0x04,
    GREYBUS_PROTOCOL_HID = 0x05,
    GREYBUS_PROTOCOL_USB = 0x06,
    GREYBUS_PROTOCOL_SDIO = 0x07,
    GREYBUS_PROTOCOL_BATTERY = 0x08,
    GREYBUS_PROTOCOL_PWM = 0x09,
    GREYBUS_PROTOCOL_I2S_MGMT = 0x0a,
    GREYBUS_PROTOCOL_SPI = 0x0b,
    GREYBUS_PROTOCOL_DISPLAY = 0x0c,
    GREYBUS_PROTOCOL_CAMERA = 0x0d,
    GREYBUS_PROTOCOL_SENSOR = 0x0e,
    GREYBUS_PROTOCOL_LED = 0x0f,
    GREYBUS_PROTOCOL_VIBRATOR = 0x10,
    GREYBUS_PROTOCOL_LOOPBACK = 0x11,
    GREYBUS_PROTOCOL_I2S_RECEIVER = 0x12,
    GREYBUS_PROTOCOL_I2S_TRANSMITTER = 0x13,
	/* ... */
    GREYBUS_PROTOCOL_VENDOR = 0xff,
};

enum greybus_class_type {
    GREYBUS_CLASS_CONTROL = 0x00,
    GREYBUS_CLASS_AP = 0x01,
    GREYBUS_CLASS_GPIO = 0x02,
    GREYBUS_CLASS_I2C = 0x03,
    GREYBUS_CLASS_UART = 0x04,
    GREYBUS_CLASS_HID = 0x05,
    GREYBUS_CLASS_USB = 0x06,
    GREYBUS_CLASS_SDIO = 0x07,
    GREYBUS_CLASS_BATTERY = 0x08,
    GREYBUS_CLASS_PWM = 0x09,
    GREYBUS_CLASS_I2S = 0x0a,
    GREYBUS_CLASS_SPI = 0x0b,
    GREYBUS_CLASS_DISPLAY = 0x0c,
    GREYBUS_CLASS_CAMERA = 0x0d,
    GREYBUS_CLASS_SENSOR = 0x0e,
    GREYBUS_CLASS_VENDOR = 0xff,
};

/*
 * A module descriptor describes information about a module as a
 * whole, *not* the functions within it.
 */
struct greybus_descriptor_module {
    __le16 vendor;
    __le16 product;
    __le16 version;
    __u8 vendor_stringid;
    __u8 product_stringid;
    __le64 unique_id;
};

/*
 * The string in a string descriptor is not NUL-terminated.  The
 * size of the descriptor will be rounded up to a multiple of 4
 * bytes, by padding the string with 0x00 bytes if necessary.
 */
struct greybus_descriptor_string {
    __u8 length;
    __u8 id;
    __u8 string[0];
};

/*
 * An interface descriptor simply defines a module-unique id for
 * each interface present on a module.  Its sole purpose is to allow
 * CPort descriptors to specify which interface they are associated
 * with.  Normally there's only one interface, with id 0.  The
 * second one must have id 1, and so on consecutively.
 *
 * The largest CPort id associated with an interface (defined by a
 * CPort descriptor in the manifest) is used to determine how to
 * encode the device id and module number in UniPro packets
 * that use the interface.
 */
struct greybus_descriptor_interface {
    __u8 id;                    /* module-relative id (0..) */
};

/*
 * A CPort descriptor indicates the id of the interface within the
 * module it's associated with, along with the CPort id used to
 * address the CPort.  The protocol defines the format of messages
 * exchanged using the CPort.
 */
struct greybus_descriptor_cport {
    __u8 interface;
    __le16 id;
    __u8 protocol;              /* enum greybus_protocol */
};

/*
 * A class descriptor defines functionality supplied by a module.
 * Beyond that, not much else is defined yet...
 */
struct greybus_descriptor_class {
    __u8 class;                 /* enum greybus_class_type */
};

struct greybus_descriptor_header {
    __le16 size;
    __u8 type;                  /* enum greybus_descriptor_type */
};

struct greybus_descriptor {
    struct greybus_descriptor_header header;
    union {
        struct greybus_descriptor_module module;
        struct greybus_descriptor_string string;
        struct greybus_descriptor_interface interface;
        struct greybus_descriptor_cport cport;
        struct greybus_descriptor_class class;
    };
};

struct greybus_manifest_header {
    __le16 size;
    __u8 version_major;
    __u8 version_minor;
};

struct greybus_manifest {
    struct greybus_manifest_header header;
    struct greybus_descriptor descriptors[0];
};

char *get_manifest_blob(void *data);
void parse_manifest_blob(char *hpe);

#pragma pack(pop)

#endif                          /* __GREYBUS_MANIFEST_H */
