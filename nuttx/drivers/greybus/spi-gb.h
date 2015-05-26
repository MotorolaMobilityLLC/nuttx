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

#ifndef _GREYBUS_SPI_H_
#define _GREYBUS_SPI_H_

#include <nuttx/greybus/types.h>

/* SPI Protocol Operation Types */
#define GB_SPI_PROTOCOL_VERSION             0x01    /* Protocol Version */
#define GB_SPI_PROTOCOL_MODE                0x02    /* Mode */
#define GB_SPI_PROTOCOL_FLAGS               0x03    /* Flags */
#define GB_SPI_PROTOCOL_BITS_PER_WORD_MASK  0x04    /* Bits per word mask */
#define GB_SPI_PROTOCOL_NUM_CHIPSELECT      0x05    /* Number of Chip-select */
#define GB_SPI_PROTOCOL_TRANSFER            0x06    /* Transfer */

/* SPI Protocol Mode Bit Masks */
#define GB_SPI_MODE_CPHA        0x01    /* clock phase */
#define GB_SPI_MODE_CPOL        0x02    /* clock polarity */
#define GB_SPI_MODE_CS_HIGH     0x04    /* chipselect active high */
#define GB_SPI_MODE_LSB_FIRST   0x08    /* per-word bits-on-wire */
#define GB_SPI_MODE_3WIRE       0x10    /* SI/SO signals shared */
#define GB_SPI_MODE_LOOP        0x20    /* loopback mode */
#define GB_SPI_MODE_NO_CS       0x40    /* one dev/bus, no chipselect */
#define GB_SPI_MODE_READY       0x80    /* slave pulls low to pause */

/* SPI Protocol Flags */
#define GB_SPI_FLAG_HALF_DUPLEX 0x01    /* can't do full duplex */
#define GB_SPI_FLAG_NO_RX       0x02    /* can't do buffer read */
#define GB_SPI_FLAG_NO_TX       0x04    /* can't do buffer write */

/**
 * SPI Protocol Version Response
 */
struct gb_spi_proto_version_response {
    __u8    major; /**< SPI Protocol major version */
    __u8    minor; /**< SPI Protocol minor version */
};

/**
 * SPI Protocol Mode Response
 */
struct gb_spi_mode_response {
    __le16  mode; /**< Greybus SPI Protocol Mode Bit Masks */
};

/**
 * SPI Protocol Flags Response
 */
struct gb_spi_flags_response {
    __le16  flags; /**< Greybus SPI Protocol Flags Bit Masks */
};

/**
 * SPI Protocol Bits Per Word Mask Response
 */
struct gb_spi_bpw_response {
    __le32  bits_per_word_mask; /**< Bits per word mask of the SPI master */
};

/**
 * Number of Chip Selects Response
 */
struct gb_spi_chipselect_response {
    __le16  num_chipselect; /**< Maximum number of chip select pins */
};

/**
 * SPI Protocol gb_spi_transfer descriptor
 */
struct gb_spi_transfer_desc {
    /** Transfer speed in Hz */
    __le32  speed_hz;
    /** Size of data to transfer */
    __le32  len;
    /** Wait period after completion of transfer */
    __le16  delay_usecs;
    /** Toggle chip select pin after this transfer completes */
    __u8    cs_change;
    /** Select bits per word for this trnasfer */
    __u8    bits_per_word;
};

/**
 * SPI Protocol Transfer Request
 */
struct gb_spi_transfer_request {
    /** chip-select pin for the slave device */
    __u8    chip_select;
    /** Greybus SPI Protocol Mode Bit Masks */
    __u8    mode;
    /** Number of gb_spi_transfer_desc */
    __le16  count;
    /** SPI gb_spi_transfer_desc array in the transfer */
    struct gb_spi_transfer_desc  transfers[0];
};

/**
 * SPI Protocol Transfer Response
 */
struct gb_spi_transfer_response {
    /** Data array for read gb_spi_transfer descriptor on the transfer */
    __u8    data[0];
};

#endif /* _GREYBUS_SPI_H_ */
