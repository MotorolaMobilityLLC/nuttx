/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arch/chip/gpio.h>
#include <arch/chip/unipro_p2p.h>

#include <nuttx/device.h>
#include <nuttx/device_gpio_tunnel.h>
#include <nuttx/gpio.h>
#include <nuttx/util.h>

#include <nuttx/unipro/unipro.h>

/* Definitions:
 *   RX: GPIO to UniPro
 *   TX: UniPro to GPIO
 *
 * Limitations:
 *   - GPIOs can only range from 0 to 127.  The MSB is used to encode the value
 *     and 0xff is used to encode an invalid GPIO.
 */

#define GPIO_TUNNEL_INVALID_RESOURCE (0xff)

#define GPIO_TUNNEL_INITIAL_VALUE (0)

#define GPIO_TUNNEL_PERIOD (121)

struct gpio_entry {
    uint8_t physical;
    uint8_t logical;
#if CONFIG_DEBUG_VERBOSE
    uint32_t count;
#endif
};

static struct gpio_tunnel {
    struct device *self;
    unsigned int cport;
    size_t rx_count;
    size_t tx_count;
    struct gpio_entry rx[CONFIG_TSB_GPIO_TUNNEL_NUM_RX];
    struct gpio_entry tx[CONFIG_TSB_GPIO_TUNNEL_NUM_TX];
} g_gpio_tunnel;

static int gpio_tunnel_irq(int irq, FAR void *context)
{
    struct gpio_tunnel *gpio_tunnel = &g_gpio_tunnel;
    size_t i;

#if CONFIG_ARCH_CHIP_TSB
    const uint8_t physical = irq - TSB_GPIO_CHIP_BASE;
#else
    #error Unsupported architecture
#endif

    for (i = 0; i < ARRAY_SIZE(gpio_tunnel->rx); i++) {
        struct gpio_entry *entry = &gpio_tunnel->rx[i];

        if (entry->physical != physical) {
            continue;
        }

        /* Found the GPIO */
        const uint8_t value = gpio_get_value(physical);
        const uint8_t buf = entry->logical | (value << 7);
        int ret = unipro_send(gpio_tunnel->cport, &buf, sizeof(buf));
        if (ret) {
            llvdbg("ERROR: failed to send: %d\n", ret);
        }

#if CONFIG_DEBUG_VERBOSE
        entry->count++;
        if (entry->count % GPIO_TUNNEL_PERIOD == 0) {
            llvdbg("physical=%d, logical=%d, count=%d, value=%d, buf=%x\n",
                  entry->physical, entry->logical, entry->count, value, buf);
        }
#endif
    }

    gpio_clear_interrupt(irq);
    return 0;
}

static int gpio_tunnel_unipro_rx_handler(unsigned int cportid,
                                         void *data, size_t len)
{
    struct gpio_tunnel *gpio_tunnel = &g_gpio_tunnel;
    const uint8_t *buffer = (const uint8_t *)data;
    size_t i;
    size_t j;

    for (i = 0; i < len; i++) {
        const uint8_t logical = buffer[i] & 0x7f;
        const uint8_t value = !!(buffer[i] & 0x80);

        for (j = 0; j < ARRAY_SIZE(gpio_tunnel->tx); j++) {
            struct gpio_entry *entry = &gpio_tunnel->tx[i];

            if (entry->logical != logical) {
                continue;
            }

            /* Found the GPIO */
            gpio_set_value(entry->physical, value);

#if CONFIG_DEBUG_VERBOSE
            entry->count++;
            if (entry->count % GPIO_TUNNEL_PERIOD == 0) {
                llvdbg("physical=%d, logical=%d, count=%d, value=%d, buf=%x\n",
                      entry->physical, entry->logical, entry->count, value, buffer[i]);
            }
#endif
            break;
        }
    }

    unipro_rxbuf_free(cportid, data);

    return 0;
}

const static struct unipro_driver gpio_tunnel_unipro_driver = {
    .name = "gpio_tunnel",
    .rx_handler = gpio_tunnel_unipro_rx_handler,
};

/**
 * @brief open device
 *
 * @param dev pointer to the UART device structure
 * dev->type must be DEVICE_TYPE_UART_HW.
 * dev->id will be mapped to /dev/ttySx (where x is the ID between 0 and 9).
 * @return Address of structure representing device or NULL on failure
 */
static int gpio_tunnel_dev_open(struct device *dev)
{
    size_t i;
    struct gpio_tunnel *gpio_tunnel;

    if (!dev) {
        return -ENODEV;
    }

    gpio_tunnel = device_get_private(dev);
    if (!gpio_tunnel) {
        return -ENOENT;
    }

    irqstate_t flags;
    flags = irqsave();

    /* RX: GPIO to UniPro */
    for (i = 0; i < ARRAY_SIZE(gpio_tunnel->rx); i++) {
        const struct gpio_entry *entry = &gpio_tunnel->rx[i];
        if (entry->physical == GPIO_TUNNEL_INVALID_RESOURCE ||
            entry->logical == GPIO_TUNNEL_INVALID_RESOURCE) {
            continue;
        }

        /* Enable GPIO interrupts. */
        gpio_direction_in(entry->physical);
        gpio_irqattach(entry->physical, gpio_tunnel_irq);
        set_gpio_triggering(entry->physical, IRQ_TYPE_EDGE_BOTH);
        gpio_clear_interrupt(entry->physical);
        gpio_unmask_irq(entry->physical);
    }

    /* TX: UniPro to GPIO */
    for (i = 0; i < ARRAY_SIZE(gpio_tunnel->tx); i++) {
        const struct gpio_entry *entry = &gpio_tunnel->tx[i];
        if (entry->physical == GPIO_TUNNEL_INVALID_RESOURCE ||
            entry->logical == GPIO_TUNNEL_INVALID_RESOURCE) {
            continue;
        }

        /* Set GPIOs to outputs. */
        gpio_direction_out(entry->physical, GPIO_TUNNEL_INITIAL_VALUE);
    }

    if (gpio_tunnel->tx_count) {
        /* Register the UniPro callback. */
        int ret = unipro_driver_register(
            (struct unipro_driver *)&gpio_tunnel_unipro_driver,
            gpio_tunnel->cport);
        if (ret) {
            dbg("ERROR: unipro register failed: %d\n", ret);
            return ret;
        }

#if CONFIG_UNIPRO_P2P_APBA
        /* Set-up and connect the CPORT. */
        dbg("setup cport=%d\n", gpio_tunnel->cport);
        unipro_p2p_setup_connection(gpio_tunnel->cport);
#endif
    }

    irqrestore(flags);

    return 0;
}

/**
 * @brief Close device
 * @param dev pointer to the UART device structure
 */
static void gpio_tunnel_dev_close(struct device *dev)
{
    size_t i;

    struct gpio_tunnel *gpio_tunnel = device_get_private(dev);
    if (!gpio_tunnel) {
        return;
    }

    irqstate_t flags;
    flags = irqsave();

    /* RX: GPIO to UniPro */
    for (i = 0; i < ARRAY_SIZE(gpio_tunnel->rx); i++) {
        const struct gpio_entry *entry = &gpio_tunnel->rx[i];
        if (entry->physical == GPIO_TUNNEL_INVALID_RESOURCE ||
            entry->logical == GPIO_TUNNEL_INVALID_RESOURCE) {
            continue;
        }

        /* Disable GPIO interrupts. */
        gpio_mask_irq(entry->physical);
        gpio_clear_interrupt(entry->physical);
        gpio_irqattach(entry->physical, NULL);
    }

    /* TX: UniPro to GPIO */
    for (i = 0; i < ARRAY_SIZE(gpio_tunnel->tx); i++) {
        const struct gpio_entry *entry = &gpio_tunnel->tx[i];
        if (entry->physical == GPIO_TUNNEL_INVALID_RESOURCE ||
            entry->logical == GPIO_TUNNEL_INVALID_RESOURCE) {
            continue;
        }

        /* Set GPIOs to inputs. */
        gpio_direction_in(entry->physical);
    }

    if (gpio_tunnel->tx_count) {
        /* Unregister the UniPro callback. */
        int ret = unipro_driver_unregister(gpio_tunnel->cport);
        if (ret) {
            dbg("ERROR: unipro unregister failed: %d\n", ret);
        }
    }

    /* Reset the CPORT. */
    dbg("reset cport=%d\n", gpio_tunnel->cport);
#if CONFIG_UNIPRO_P2P
    unipro_p2p_reset_connection(gpio_tunnel->cport);
#else
    _unipro_reset_cport(gpio_tunnel->cport);
#endif

    irqrestore(flags);
}

/**
 * @brief probe device
 * @param dev pointer to the device structure
 * @return 0 for success, -errno for failures.
 */
static int gpio_tunnel_probe(struct device *dev)
{
    struct gpio_tunnel *gpio_tunnel = &g_gpio_tunnel;
    size_t i;

    memset(gpio_tunnel, 0, sizeof(gpio_tunnel));
    gpio_tunnel->self = dev;
    gpio_tunnel->cport = CONFIG_TSB_GPIO_TUNNEL_CPORT_ID;

    /* RX: GPIO to UniPro */
    for (i = 0; i < ARRAY_SIZE(gpio_tunnel->rx); i++) {
        char name[16]; /* "rx_physical_127" */

        snprintf(name, sizeof(name), "rx_physical_%d", i);
        struct device_resource *physical = device_resource_get_by_name(dev,
            DEVICE_RESOURCE_TYPE_GPIO, name);

        snprintf(name, sizeof(name), "rx_logical_%d", i);
        struct device_resource *logical = device_resource_get_by_name(dev,
            DEVICE_RESOURCE_TYPE_GPIO, name);

        if (physical && logical) {
            gpio_tunnel->rx[i].physical = (uint8_t)physical->start;
            gpio_tunnel->rx[i].logical = (uint8_t)logical->start;

            vdbg("rx[%d]: physical=%d, logical=%d\n", i, physical->start, logical->start);

            gpio_tunnel->rx_count++;
        } else {
            gpio_tunnel->rx[i].physical = GPIO_TUNNEL_INVALID_RESOURCE;
            gpio_tunnel->rx[i].logical = GPIO_TUNNEL_INVALID_RESOURCE;
        }
    }

    /* TX: UniPro to GPIO */
    for (i = 0; i < ARRAY_SIZE(gpio_tunnel->tx); i++) {
        char name[16]; /* "tx_physical_127" */

        snprintf(name, sizeof(name), "tx_physical_%d", i);
        struct device_resource *physical = device_resource_get_by_name(dev,
            DEVICE_RESOURCE_TYPE_GPIO, name);

        snprintf(name, sizeof(name), "tx_logical_%d", i);
        struct device_resource *logical = device_resource_get_by_name(dev,
            DEVICE_RESOURCE_TYPE_GPIO, name);

        if (physical && logical) {
            gpio_tunnel->tx[i].physical = (uint8_t)physical->start;
            gpio_tunnel->tx[i].logical = (uint8_t)logical->start;

            vdbg("tx[%d]: physical=%d, logical=%d\n", i, physical->start, logical->start);

            gpio_tunnel->tx_count++;
        } else {
            gpio_tunnel->tx[i].physical = GPIO_TUNNEL_INVALID_RESOURCE;
            gpio_tunnel->tx[i].logical = GPIO_TUNNEL_INVALID_RESOURCE;
        }
    }

    /* Reset the CPORT. */
    dbg("reset cport=%d\n", gpio_tunnel->cport);
#if CONFIG_UNIPRO_P2P
    unipro_p2p_reset_connection(gpio_tunnel->cport);
#else
    _unipro_reset_cport(gpio_tunnel->cport);
#endif

    device_set_private(dev, gpio_tunnel);
    return 0;
}

/**
 * @brief remove device
 * @param dev pointer to the UART device structure
 */
static void gpio_tunnel_remove(struct device *dev)
{
    struct gpio_tunnel *gpio_tunnel = device_get_private(dev);
    if (gpio_tunnel) {
        device_set_private(dev, NULL);
        memset(gpio_tunnel, 0, sizeof(*gpio_tunnel));
    }
}

const static struct device_driver_ops gpio_tunnel_driver_ops = {
    .probe    = gpio_tunnel_probe,
    .remove   = gpio_tunnel_remove,
    .open     = gpio_tunnel_dev_open,
    .close    = gpio_tunnel_dev_close,
};

const struct device_driver gpio_tunnel_driver = {
    .type = DEVICE_TYPE_GPIO_TUNNEL_HW,
    .name = "gpio_tunnel",
    .desc = "GPIO Tunnel",
    .ops  = (struct device_driver_ops *)&gpio_tunnel_driver_ops,
};