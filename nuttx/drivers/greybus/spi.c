/*
 * Copyright (c) 2015 Google, Inc.
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

#include <nuttx/device.h>
#include <nuttx/device_spi.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/debug.h>
#include <apps/greybus-utils/utils.h>

#include <arch/byteorder.h>

#include "spi-gb.h"

#define GB_SPI_VERSION_MAJOR 0
#define GB_SPI_VERSION_MINOR 1

static struct device *spi_dev = NULL;

/**
 * @brief Returns the major and minor Greybus SPI protocol version number
 *        supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_version(struct gb_operation *operation)
{
    struct gb_spi_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_SPI_VERSION_MAJOR;
    response->minor = GB_SPI_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Returns a bit mask indicating the modes supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_mode(struct gb_operation *operation)
{
    struct gb_spi_mode_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* get hardware capabilities */
    ret = device_spi_getcaps(spi_dev, &caps);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }
    response->mode = cpu_to_le16(caps.modes);

    return GB_OP_SUCCESS;
}

/**
 * @brief Returns a bit mask indicating the constraints of the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_flags(struct gb_operation *operation)
{
    struct gb_spi_flags_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* get hardware capabilities */
    ret = device_spi_getcaps(spi_dev, &caps);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }
    response->flags = cpu_to_le16(caps.flags);

    return GB_OP_SUCCESS;
}

/**
 * @brief Returns the number of bits per word supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_bpw(struct gb_operation *operation)
{
    struct gb_spi_bpw_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* get hardware capabilities */
    ret = device_spi_getcaps(spi_dev, &caps);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }

    response->bits_per_word_mask = cpu_to_le32(caps.bpw);

    return GB_OP_SUCCESS;
}

/**
 * @brief Returns the number of chip select pins supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_num_chipselect(struct gb_operation *operation)
{
    struct gb_spi_chipselect_response *response;
    struct device_spi_caps caps;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* get hardware capabilities */
    ret = device_spi_getcaps(spi_dev, &caps);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }
    response->num_chipselect = cpu_to_le16(caps.csnum);

    return GB_OP_SUCCESS;
}

/**
 * @brief Performs a SPI transaction as one or more SPI transfers, defined
 *        in the supplied array.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_transfer(struct gb_operation *operation)
{
    int i, op_count;
    uint32_t size = 0;
    int ret = 0, errcode = GB_OP_SUCCESS;
    uint8_t *write_data;
    uint8_t *read_buf;
    uint32_t freq = 0;
    bool selected = false;
    struct device_spi_transfer transfer;
    size_t request_size = gb_operation_get_request_payload_size(operation);
    size_t expected_size;

    struct gb_spi_transfer_desc *desc;
    struct gb_spi_transfer_request *request;
    struct gb_spi_transfer_response *response;

    if (request_size < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    op_count = le16_to_cpu(request->count);
    write_data = (uint8_t *)&request->transfers[op_count];

    expected_size = sizeof(*request) + op_count * sizeof(request->transfers[0]);
    if (request_size < expected_size) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    for (i = 0; i < op_count; i++) {
        desc = &request->transfers[i];
        size += le32_to_cpu(desc->len);
    }

    response = gb_operation_alloc_response(operation, size);
    if (!response) {
        return GB_OP_NO_MEMORY;
    }
    read_buf = response->data;

    /* lock SPI bus */
    ret = device_spi_lock(spi_dev);
    if (ret) {
        return (ret == -EINVAL)? GB_OP_INVALID : GB_OP_UNKNOWN_ERROR;
    }

    /* set SPI mode */
    ret = device_spi_setmode(spi_dev, request->mode);
    if (ret) {
        goto spi_err;
    }

    /* parse all transfer request from AP host side */
    for (i = 0; i < op_count; i++) {
        desc = &request->transfers[i];
        freq = le32_to_cpu(desc->speed_hz);

        /* set SPI bits-per-word */
        ret = device_spi_setbits(spi_dev, desc->bits_per_word);
        if (ret) {
            goto spi_err;
        }

        /* set SPI clock */
        ret = device_spi_setfrequency(spi_dev, &freq);
        if (ret) {
            goto spi_err;
        }

        /* assert chip-select pin */
        if (!selected) {
            ret = device_spi_select(spi_dev, request->chip_select);
            if (ret) {
                goto spi_err;
            }
            selected = true;
        }

        /* setup SPI transfer */
        memset(&transfer, 0, sizeof(struct device_spi_transfer));
        transfer.txbuffer = write_data;
        transfer.rxbuffer = read_buf;
        transfer.nwords = le32_to_cpu(desc->len);
        transfer.flags = SPI_FLAG_DMA_TRNSFER; // synchronous & DMA transfer

        /* start SPI transfer */
        ret = device_spi_exchange(spi_dev, &transfer);
        if (ret) {
            goto spi_err;
        }
        /* move to next gb_spi_transfer data buffer */
        write_data += le32_to_cpu(desc->len);
        read_buf += le32_to_cpu(desc->len);

        if (le16_to_cpu(desc->delay_usecs) > 0) {
            usleep(le16_to_cpu(desc->delay_usecs));
        }

        /* if cs_change enable, change the chip-select pin signal */
        if (desc->cs_change) {
            /* force deassert chip-select pin */
            ret = device_spi_deselect(spi_dev, request->chip_select);
            if (ret) {
                goto spi_err;
            }
            selected = false;
        }
    }

spi_err:
    errcode = ret;

    if (selected) {
        /* deassert chip-select pin */
        ret = device_spi_deselect(spi_dev, request->chip_select);
        if (ret) {
            errcode = ret;
        }
    }

    /* unlock SPI bus*/
    ret = device_spi_unlock(spi_dev);
    if (ret) {
        errcode = ret;
    }

    if (errcode) {
        /* get error code */
        errcode = (errcode == -EINVAL)? GB_OP_INVALID : GB_OP_UNKNOWN_ERROR;
    }
    return errcode;
}

/**
 * @brief Greybus SPI protocol initialize function
 *
 * @param cport CPort number
 * @return 0 on success, negative errno on error
 */
static int gb_spi_init(unsigned int cport)
{
    if (!spi_dev) {
        spi_dev = device_open(DEVICE_TYPE_SPI_HW, 0);
        if (!spi_dev) {
            return -EIO;
        }
    }
    return 0;
}

/**
 * @brief Greybus SPI protocol deinitialize function
 *
 * @param cport CPort number
 */
static void gb_spi_exit(unsigned int cport)
{
    if (spi_dev) {
        device_close(spi_dev);
        spi_dev = NULL;
    }
}

/**
 * @brief Greybus SPI protocol operation handler
 */
static struct gb_operation_handler gb_spi_handlers[] = {
    GB_HANDLER(GB_SPI_PROTOCOL_VERSION, gb_spi_protocol_version),
    GB_HANDLER(GB_SPI_PROTOCOL_MODE, gb_spi_protocol_mode),
    GB_HANDLER(GB_SPI_PROTOCOL_FLAGS, gb_spi_protocol_flags),
    GB_HANDLER(GB_SPI_PROTOCOL_BITS_PER_WORD_MASK, gb_spi_protocol_bpw),
    GB_HANDLER(GB_SPI_PROTOCOL_NUM_CHIPSELECT, gb_spi_protocol_num_chipselect),
    GB_HANDLER(GB_SPI_PROTOCOL_TRANSFER, gb_spi_protocol_transfer),
};

static struct gb_driver gb_spi_driver = {
    .init = gb_spi_init,
    .exit = gb_spi_exit,
    .op_handlers = gb_spi_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_spi_handlers),
};

/**
 * @brief Register Greybus SPI protocol
 *
 * @param cport CPort number
 */
void gb_spi_register(int cport)
{
    gb_register_driver(cport, &gb_spi_driver);
}

/**
 * @brief Set SPI device
 *
 * @param device pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
int gb_spi_set_dev(struct device *dev)
{
    if (!spi_dev)
        spi_dev = dev;
    else
        return -EBUSY;
    return 0;
}

/**
 * @brief Get SPI device
 *
 * @return a device pointer.
 */
struct device *gb_spi_get_dev(void)
{
    return spi_dev;
}
