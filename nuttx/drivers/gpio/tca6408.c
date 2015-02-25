/**
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
 *
 * @brief TCA6408 GPIO Expander Driver
 */

#include <nuttx/config.h>
#include <nuttx/i2c.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>
#include <nuttx/gpio/tca6408.h>
#include <nuttx/gpio.h>

#define TCA6408_INPUT_REG       0x00
#define TCA6408_OUTPUT_REG      0x01
#define TCA6408_POLARITY_REG    0x02
#define TCA6408_CONFIG_REG      0x03

#define TCA6408_TW      1 /* 1us (datasheet: reset pulse duration (Tw) is 4ns */
#define TCA6408_TRESET  1 /* 1us (datasheet: time to reset (Treset) is 600ns */

#define TCA6408_NR_GPIO 8

/* disable the verbose debug output */
#undef lldbg
#define lldbg(x...)

struct tca6408_platform_data
{
    struct i2c_dev_s *dev;
    uint8_t addr;
    uint8_t reset;
};

static struct tca6408_platform_data g_tca6408;

static int i2c_get(uint8_t addr, uint8_t regaddr, uint8_t *val)
{
    int ret;
    struct i2c_dev_s *dev = g_tca6408.dev;
    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = &regaddr,
            .length = 1,
        }, {
            .addr = addr,
            .flags = I2C_M_READ,
            .buffer = val,
            .length = 1,
        },
    };

    if (!dev) {
        return -EINVAL;
    }
    ret = I2C_TRANSFER(dev, msg, 2);

    if (ret == 0) {
        lldbg("addr=0x%02hhX, regaddr=0x%02hhX: read 0x%02hhX\n",
              addr, regaddr, *val);
    } else {
        lldbg("addr=0x%02hhX, regaddr=0x%02hhX: failed!\n",
              addr, regaddr);
    }

    return ret;
}

static int i2c_set(uint8_t addr, uint8_t regaddr, uint8_t val)
{
    int ret;
    struct i2c_dev_s *dev = g_tca6408.dev;
    uint8_t cmd[2] = {regaddr, val};
    uint8_t data8;
    struct i2c_msg_s msg[] = {
        {
            .addr = addr,
            .flags = 0,
            .buffer = cmd,
            .length = 2,
        }, {
            .addr = addr,
            .flags = I2C_M_READ,
            .buffer = &data8,
            .length = 1,
        },
    };

    lldbg("addr=0x%02hhX: regaddr=0x%02hhX, val=0x%02hhX\n",
          addr, regaddr, val);
    if (!dev) {
        return -EINVAL;
    }
    ret = I2C_TRANSFER(dev, msg, 2);

    return ret;
}

int tca6408_reset(bool en)
{
    /*
     * TCA6408 reset pin is active low.
     * If en = 1, maintain chip under reset.
     * If en = 0, get chip out of reset.
     */
    lldbg("gpio=0x%02hhX, en=%hhu\n", g_tca6408.reset, en);
    gpio_direction_out(g_tca6408.reset, 0);
    if (en == 0) {
        /* datasheet: reset pulse minimum duration (Tw) is 4ns */
        usleep(TCA6408_TW);
        gpio_direction_out(g_tca6408.reset, 1);
        /* datasheet: time to reset (Treset) is 600ns */
        usleep(TCA6408_TRESET);
    }

    return 0;
}

void tca6408_set_direction_in(uint8_t which)
{
    uint8_t addr;
    uint8_t reg;
    int ret;

    addr = g_tca6408.addr;
    lldbg("addr=0x%02hhX, which=%hhu\n", addr, which);
    /* Configure pin as input
     *
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    ret = i2c_get(addr, TCA6408_CONFIG_REG, &reg);
    if (ret != 0)
        return
    lldbg("current cfg=0x%02X\n", reg);
    reg |= (1 << which);
    lldbg("new cfg=0x%02X\n", reg);
    ret = i2c_set(addr, TCA6408_CONFIG_REG, reg);
}

int tca6408_set_default_outputs(uint8_t dflt)
{
    int ret;
    uint8_t addr = g_tca6408.addr;

    lldbg("addr=0x%02hhX, dflt=0x%02hhX\n", addr, dflt);
    /* Set output pins default value (before configuring it as output
     *
     * The Output Port Register (register 1) shows the outgoing logic
     * levels of the pins defined as outputs by the Configuration Register.
     */
    ret = i2c_set(addr, TCA6408_OUTPUT_REG, dflt);
    if (ret != 0)
        return -EIO;

    return 0;
}

void tca6408_set_direction_out(uint8_t which, uint8_t value)
{
    uint8_t addr;
    uint8_t reg;
    int ret;

    addr = g_tca6408.addr;
    lldbg("addr=0x%02hhX, which=%hhu\n", addr, which);
    /* Configure pin as output
     *
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    ret = i2c_get(addr, TCA6408_CONFIG_REG, &reg);
    if (ret != 0)
        return;
    lldbg("current cfg=0x%02X\n", reg);
    reg &= ~(1 << which);
    lldbg("new cfg=0x%02X\n", reg);
    ret = i2c_set(addr, TCA6408_CONFIG_REG, reg);
    if (ret != 0)
        return;

    tca6408_set(which, value);
}


int tca6408_get_direction(uint8_t which)
{
    uint8_t addr = g_tca6408.addr;
    uint8_t direction;
    int ret;

    lldbg("addr=0x%02hhX, which=%hhu", addr, which);
    /*
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    ret = i2c_get(addr, TCA6408_CONFIG_REG, &direction);
    if (ret != 0)
        return -EIO;
    direction = (direction & (1 << which)) >> which;
    return direction;
}


int tca6408_set_polarity_inverted(uint8_t which, uint8_t inverted)
{
    uint8_t addr = g_tca6408.addr;
    uint8_t polarity;
    int ret;

    lldbg("addr=0x%02hhX, which=%hhu inverted=%hhu\n",
          addr, which, inverted);
    /* Configure pin polarity inversion
     *
     * The Polarity Inversion Register (register 2) allows
     * polarity inversion of pins defined as inputs by the Configuration
     * Register. If a bit in this register is set (written with 1),
     * the corresponding port pin's polarity is inverted. If a bit in
     * this register is cleared (written with a 0), the corresponding
     * port pin's original polarity is retained.
     */
    ret = i2c_get(addr, TCA6408_POLARITY_REG, &polarity);
    if (ret != 0)
        return -EIO;
    lldbg("current polarity reg=0x%02hhX\n", polarity);
    if (inverted) {
        polarity |= (1 << which);
    } else {
        polarity &= ~(1 << which);
    }
    lldbg("new polarity reg=0x%02hhX\n", polarity);
    ret = i2c_set(addr, TCA6408_POLARITY_REG, polarity);
    if (ret != 0)
        return -EIO;

    return 0;
}


int tca6408_get_polarity_inverted(uint8_t which)
{
    uint8_t addr = g_tca6408.addr;
    uint8_t polarity;
    int ret;

    lldbg("addr=0x%02hhX, which=%hhu\n", addr, which);
    /*
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    ret = i2c_get(addr, TCA6408_POLARITY_REG, &polarity);
    if (ret != 0)
        return -EIO;
    lldbg("polarity reg=0x%02hhX\n", polarity);
    polarity = (polarity & (1 << which)) >> which;
    lldbg("polarity=0x%hhu\n", polarity);

    return polarity;
}


void tca6408_set(uint8_t which, uint8_t val)
{
    uint8_t addr = g_tca6408.addr;
    uint8_t reg;
    int ret;

    lldbg("addr=0x%02hhX, which=%hhu, val=%hhu\n",
          addr, which, val);
    /* Set output pins default value (before configuring it as output
     *
     * The Output Port Register (register 1) shows the outgoing logic
     * levels of the pins defined as outputs by the Configuration Register.
     */
    ret = i2c_get(addr, TCA6408_OUTPUT_REG, &reg);
    if (ret != 0)
        return;
    lldbg("current reg=0x%02hhX\n", reg);
    if (val) {
        reg |= (1 << which);
    } else {
        reg &= ~(1 << which);
    }
    lldbg("new reg=0x%02hhX\n", reg);
    ret = i2c_set(addr, TCA6408_OUTPUT_REG, reg);
}


uint8_t tca6408_get(uint8_t which)
{
    uint8_t addr = g_tca6408.addr;
    uint8_t in;
    int ret;

    lldbg("addr=0x%02hhX, which=%hhu\n", addr, which);
    /*
     * The Input Port Register (register 0) reflects the incoming logic
     * levels of the pins, regardless of whether the pin is defined as an
     * input or an output by the Configuration Register. They act only on
     * read operation.
     */
    ret = i2c_get(addr, TCA6408_INPUT_REG, &in);
    if (ret != 0)
        return -EIO;
    lldbg("input reg=0x%02hhX\n", in);
    in &= (1 << which);
    in = !!in;
    lldbg("in=%hhu\n", in);

    return in;
}

void tca6408_activate(uint8_t which)
{

}

void tca6408_deactivate(uint8_t which)
{

}

uint8_t tca6408_line_count(void)
{
    return TCA6408_NR_GPIO;
}

struct gpio_ops_s tca6408_gpio_ops = {
    .get_direction = tca6408_get_direction,
    .direction_in = tca6408_set_direction_in,
    .direction_out = tca6408_set_direction_out,
    .activate = tca6408_activate,
    .get_value = tca6408_get,
    .set_value = tca6408_set,
    .deactivate = tca6408_deactivate,
    .line_count = tca6408_line_count,
    .irqattach = NULL,
    .set_triggering = NULL,
    .mask_irq = NULL,
    .unmask_irq = NULL,
    .clear_interrupt = NULL,
};

void tca6408_init(struct i2c_dev_s *dev, uint8_t addr, uint8_t reset)
{
    g_tca6408.dev = dev;
    g_tca6408.addr = addr;
    g_tca6408.reset = reset;
    register_gpio_chip(&tca6408_gpio_ops, -1);
}
