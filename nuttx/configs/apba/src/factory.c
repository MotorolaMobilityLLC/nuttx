/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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

#include <debug.h>
#include <errno.h>
#include <queue.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arch/board/factory.h>

#include <arch/byteorder.h>

#include <arch/chip/unipro_p2p.h>

#include <nuttx/device.h>
#include <nuttx/device_uart.h>
#include <nuttx/gpio.h>
#include <nuttx/greybus/types.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pthread.h>
#include <nuttx/util.h>

#include <up_arch.h>
#include <tsb_scm.h>

#define MODE_MASK_MIRROR_BOOTRET   0x00000001 // If 1: Put BOOTRET val back on GPIO 8
#define MODE_MASK_GPIO_8_SET       0x00000002 // If 1: turn GPIO 8 on
#define MODE_MASK_GPIO_5_LOOP      0x00000004 // If 1: loopback GPIO 5
#define MODE_MASK_I2S_AS_GPIO      0x00000008 // If 1: Loopback I2S pins as GPIOs instead
#define MODE_MASK_UART_LOOPBACK    0x00000010 // If 1: Activate loopback of UART
/* reserved */
#define MODE_MASK_UNIPRO_COMMAND   0x80000000 // If 1: bits [30:0] are the command id.

#define BOOTRET_REG  0x4009002C
#define BOOTRET_MASK 0x00000001

struct gpio_info {
    int which;
    int direction; // !0 = in, 0 = out
    int mask;
};

#define UART_BAUD      115200
#define UART_RXBUFSIZE 1024

static uint8_t *rx_buf;
static struct device *uart_dev;

static int uart_loopback_rx_start(void);

// List of GPIOs that make up the loop.
// The first half are inputs and the second half are the corresponding outputs.
// The loopback will only be active if the "mode" parameter matches the mask
const static struct gpio_info loop_gpios[] = {
    // Inputs
    { 5,  1, MODE_MASK_GPIO_5_LOOP },    // 5 drives 6
    { 17, 1, MODE_MASK_I2S_AS_GPIO },   // 17 drives 16
    { 20, 1, MODE_MASK_I2S_AS_GPIO },   // 20 drives 19

    // Outputs
    { 6,  0, MODE_MASK_GPIO_5_LOOP },
    { 16, 0, MODE_MASK_I2S_AS_GPIO },
    { 19, 0, MODE_MASK_I2S_AS_GPIO },
};

// For a particular input GPIO, find the matching output pair and set it
// to the passed in value.
static void mirror(int input_gpio, int value)
{
    int pair;
    int output_gpio;

    for (pair = 0; pair < (ARRAY_SIZE(loop_gpios) / 2); pair++)
    {
        if (loop_gpios[pair].which == input_gpio) {
            output_gpio = loop_gpios[pair + (ARRAY_SIZE(loop_gpios) / 2)].which;
            lldbg("Mirroring %d value %d to %d\n", input_gpio, value, output_gpio);
            gpio_set_value(output_gpio, value);
        }
    }
}

static int gpio_irq_event(int irq, FAR void *context)
{
    int value;

    gpio_mask_irq(irq);

    // mirror the input to the corresponding output
    value = gpio_get_value(irq);
    mirror(irq, value);

    gpio_clear_interrupt(irq);
    gpio_unmask_irq(irq);

    return 0;
}

static void gpio_loopback_config(uint8_t mode)
{
    int i;

    if (mode & MODE_MASK_I2S_AS_GPIO) {
        // Setup PINSHARE for GPIO access of GPIO16,17,19,20 instead of I2S.
        // Note: GPIO18 is not connected to the AP.  We just set that for completeness.
        // Also PINSHARE register bit 10 is shared between two pins, selecting
        // GPIO16 & GPIO17 vs. I2S_BCLK & I2S_LRCLK
        tsb_clr_pinshare(TSB_PIN_ETM);
        tsb_set_pinshare(TSB_PIN_GPIO16 | TSB_PIN_GPIO18 | TSB_PIN_GPIO19 | TSB_PIN_GPIO20);
    }

    // Configure loopback input and output GPIOs.  The interrupts start as masked,
    // so we must unmask below.
    for (i = 0; i < ARRAY_SIZE(loop_gpios); i++)
    {
        if ((loop_gpios[i].mask & mode) == loop_gpios[i].mask) {
            gpio_activate(loop_gpios[i].which);
            if (loop_gpios[i].direction) {
                gpio_direction_in(loop_gpios[i].which);
                // gpio_set_debounce?
                set_gpio_triggering(loop_gpios[i].which, IRQ_TYPE_EDGE_BOTH);
                gpio_irqattach(loop_gpios[i].which, gpio_irq_event);
            } else {
                gpio_direction_out(loop_gpios[i].which, 0);
            }
        }
    }

    // Initial mirroring of GPIO inputs and unmasking of their interrupts.
    for (i = 0; i < (ARRAY_SIZE(loop_gpios) / 2); i++)
    {
        if ((loop_gpios[i].mask & mode) == loop_gpios[i].mask) {
            mirror(loop_gpios[i].which, gpio_get_value(loop_gpios[i].which));
            gpio_unmask_irq(loop_gpios[i].which);
        }
    }
}

static int uart_loopback_open(void)
{
    uart_dev = device_open(DEVICE_TYPE_UART_HW, 0);
    if (!uart_dev) {
        return -1;
    }

    int ret = device_uart_set_configuration(uart_dev, BAUD_115200,
                                            NO_PARITY, 8 /* bits */, ONE_STOP_BIT,
                                            1 /* flow control enabled */);
    if (ret) {
        return ret;
    }

    return 0;
}

// We need an empty callback or else the UART code will be stuck waiting on
// its TX semaphore which does not get posted.
static void uart_loopback_tx_callback_dummy(uint8_t *buffer, int length, int error)
{
}

static void uart_loopback_rx_callback(uint8_t *buffer, int length, int error)
{
    int sent;
    int ret = device_uart_start_transmitter(uart_dev, buffer,
                    length, NULL /* dma */, &sent,
                    uart_loopback_tx_callback_dummy /* callback, blocking without callback */);
    if (ret) {
        lldbg("ERROR: failed to write: %d\n", ret);
    }

    uart_loopback_rx_start();
}

static int uart_loopback_rx_start(void)
{
    int ret = device_uart_start_receiver(uart_dev,
                                         rx_buf,
                                         UART_RXBUFSIZE,
                                         NULL /* dma */,
                                         NULL /* got, not used with callback */,
                                         uart_loopback_rx_callback);
    if (ret) {
        lldbg("ERROR: Failed to start the receiver: %d\n", ret);
    }

    return ret;
}

int factory_mode(uint32_t mode)
{
    int ret = 0;

    lldbg("factory mode: 0x%08x\n", mode);

    if (mode & MODE_MASK_UNIPRO_COMMAND) {
        return unipro_test_command(mode & ~MODE_MASK_UNIPRO_COMMAND);
    }

    if (mode & MODE_MASK_GPIO_8_SET) {
        gpio_activate(8);
        gpio_direction_out(8, 1);
    }

    // The state of the BOOTRET pin is latched into the BOOTRET register only
    // once at power up of the TSB. Therefore, we will (optionally) mirror that
    // BOOTRET to GPIO 8 once here.
    if (mode & MODE_MASK_MIRROR_BOOTRET) {
        gpio_activate(8);
        gpio_direction_out(8, (getreg32(BOOTRET_REG) & BOOTRET_MASK));
    }

    gpio_loopback_config(mode);

    if (mode & MODE_MASK_UART_LOOPBACK) {
        // Initialize UART loopback
        // This is rather similar to uart.c from uart app.
        rx_buf = kmm_zalloc(UART_RXBUFSIZE);
        if (!rx_buf) {
            lldbg("ERROR: Failed to allocate rx buf\n");
            return -ENOMEM;
        }

        ret = uart_loopback_open();
        if (ret) {
            lldbg("ERROR: Failed to open UART: %s.\n", strerror(errno));
            goto error;
        }

        uart_loopback_rx_start();
    }
    return 0;

error:
    if (mode & MODE_MASK_UART_LOOPBACK) {
        kmm_free(rx_buf);
    }

    return ret;
}
