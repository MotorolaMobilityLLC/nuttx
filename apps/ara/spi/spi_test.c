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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/types.h>

#include <arch/tsb/chip.h>
#include <tsb_scm.h>

#include <up_arch.h>

#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/device_spi.h>
#include <nuttx/gpio.h>

#define BUFSIZE 32

/**
 * @brief SPI data transfer
 *
 * The function transfer one word (32-bits) to SPI hardware
 *
 * @param mode SPI transfer mode
 * @param nbits SPI bit per word setting
 * @param freq SPI transfer frequency
 * @param txbuf pointer to the spi write buffer
 * @param rxbuf pointer to the spi read buffer
 * @param len SPI data transfer length
 * @return 0 on success, negative errno on error
 */
static int spi_xfer(uint16_t mode, uint32_t nbits, uint32_t freq, void* txbuf,
             void* rxbuf, int len)
{
    struct device *spi_dev = NULL;
    struct device_spi_transfer transfer;
    int result = 0, i = 0;

    printf("SPI FREQUENCY       : %u Hz\n", freq);
    printf("SPI_MODE_CPHA       : %u\n", mode & SPI_MODE_CPHA);
    printf("SPI_MODE_CPOL       : %u\n", mode & SPI_MODE_CPOL);
    printf("SPI_MODE_CS_HIGH    : %u\n", mode & SPI_MODE_CS_HIGH);
    printf("SPI_MODE_LSB_FIRST  : %u\n", mode & SPI_MODE_LSB_FIRST);
    printf("SPI_MODE_3WIRE      : %u\n", mode & SPI_MODE_3WIRE);
    printf("SPI_MODE_LOOP       : %u\n", mode & SPI_MODE_LOOP);
    printf("SPI_MODE_NO_CS      : %u\n", mode & SPI_MODE_NO_CS);
    printf("SPI_MODE_READY      : %u\n", mode & SPI_MODE_READY);
    printf("SPI BIT-PER-WORD    : %u Bits\n", nbits);

    spi_dev = device_open(DEVICE_TYPE_SPI_HW, 0);
    if (!spi_dev) {
        result = -EIO;
        goto err_open;
    }

    /* fill out the testing data for spi transfer */
    transfer.txbuffer = txbuf;
    transfer.rxbuffer = rxbuf;
    transfer.nwords = len;
    transfer.flags = 0; /* blocking & non-dma */
    transfer.timeout = 0;
    transfer.status = 0;

    device_spi_lock(spi_dev);
    result = device_spi_setmode(spi_dev, mode);
    if (result != 0) {
        goto err_config;
    }
    result = device_spi_setbits(spi_dev, nbits);
    if (result != 0) {
        goto err_config;
    }
    result = device_spi_setfrequency(spi_dev, &freq);
    if (result != 0) {
        goto err_config;
    }
    device_spi_select(spi_dev, 0);
    device_spi_exchange(spi_dev, &transfer);
    device_spi_deselect(spi_dev, 0);

    /* dump the SPI buffer data */
    for (i = 0; i < transfer.nwords; i++) {
        if (nbits <= 8) {
            if (txbuf) {
                printf("txbuf[%d]= %02X\n", i, *((uint8_t*)txbuf + i));
            }
            if (rxbuf) {
                printf("rxbuf[%d]= %02X\n", i, *((uint8_t*)rxbuf + i));
            }
        } else if (nbits <= 16) {
            if (txbuf) {
                printf("txbuf[%d]= %04X\n", i, *((uint16_t*)txbuf + i));
            }
            if (rxbuf) {
                printf("rxbuf[%d]= %04X\n", i, *((uint16_t*)rxbuf + i));
            }
        } else if (nbits <= 32) {
            if (txbuf) {
                printf("txbuf[%d]= %08X\n", i, *((uint32_t*)txbuf + i));
            }
            if (rxbuf) {
                printf("rxbuf[%d]= %08X\n", i, *((uint32_t*)rxbuf + i));
            }
        }
    }
    result = 0;
err_config:
    device_spi_unlock(spi_dev);
    device_close(spi_dev);
err_open:
    if (result != 0) {
        printf("transfer err = %d\n",result);
    }
    return result;
}

static void print_usage(void) {
    printf("spi_test: Usage:\n");
    printf("          m <hex mode bitmask> SPI transfer mode test\n");
    printf("          f <frequency> SPI frequency test\n");
    printf("          b <nbits> SPI bit-per-word test\n");
    printf("          l <len> n bytes data transfer test\n");
    printf("          d <hex data ...> transfer 1 byte specific data\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[]) {
#else
int spi_test_main(int argc, char *argv[]) {
#endif
    char cmd = '\0';
    int ret = 0, i = 0;
    char *endptr = NULL;

    uint8_t rxbuf[BUFSIZE] = {0}, txbuf[BUFSIZE] = {0};
    uint16_t mode = 0;
    uint32_t nbits = 0, freq = 0;

    if (argc < 2) {
        print_usage();
        return EXIT_FAILURE;
    } else {
        cmd = argv[1][0];
    }

    /* fill test data */
    for (i = 0; i < BUFSIZE; i++) {
        txbuf[i] = 0x55;
    }

    mode = SPI_MODE_0; /* default SPI mode0 */
    freq = 10000; /* default frequency 10KHz */
    nbits = 8; /* default bpw = 8bits */

    switch (cmd) {
    case 'h':
    case '?':
        print_usage();
        break;
    case 'm':
        if (argc == 3) {
            mode = strtoul(argv[2], &endptr, 16);
            ret = spi_xfer(mode, nbits, freq, txbuf, rxbuf, 1);
        } else {
            print_usage();
            return EXIT_FAILURE;
        }
        break;
    case 'f':
        if (argc == 3) {
            freq = strtoul(argv[2], &endptr, 10);
            ret = spi_xfer(mode, nbits, freq, txbuf, rxbuf, 1);
        } else {
            print_usage();
            return EXIT_FAILURE;
        }
        break;
    case 'b':
        if (argc == 3) {
            nbits = strtoul(argv[2], &endptr, 10);
            ret = spi_xfer(mode, nbits, freq, txbuf, rxbuf, 1);
        } else {
            print_usage();
            return EXIT_FAILURE;
        }
        break;
    case 'l':
        if (argc == 3) {
            i = strtoul(argv[2], &endptr, 10);
            if (i > BUFSIZE) {
                i = BUFSIZE;
            }
            ret = spi_xfer(mode, nbits, freq, txbuf, rxbuf, i);
        } else {
            print_usage();
            return EXIT_FAILURE;
        }
        break;
    case 'd':
        if (argc >= 3) {
            for (i = 0; i < argc - 2; i++) {
                txbuf[i] = strtoul(argv[i + 2], &endptr, 16);
            }
            ret = spi_xfer(mode, nbits, freq, txbuf, rxbuf, 1);
        } else {
            print_usage();
            return EXIT_FAILURE;
        }
        break;
    default:
        print_usage();
        return EXIT_FAILURE;
    }
    return ret;
}
