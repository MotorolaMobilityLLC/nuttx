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
 * @brief TCA64XX GPIO Expander Driver
 * Supports the following parts: TCA6408, TCA6416, TCA6424
 * @author  Patrick Titiano, Jean Pihet
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <nuttx/gpio/tca64xx.h>
#include <nuttx/gpio.h>
#include <nuttx/wqueue.h>
#include <pthread.h>
#include <sys/wait.h>

#define TCA6408_INPUT_REG               0x00
#define TCA6408_OUTPUT_REG              0x01
#define TCA6408_POLARITY_REG            0x02
#define TCA6408_CONFIG_REG              0x03

#define TCA6408_NR_GPIOS                8

#define TCA6416_INPUT0_REG              0x00
#define TCA6416_INPUT1_REG              0x01
#define TCA6416_OUTPUT0_REG             0x02
#define TCA6416_OUTPUT1_REG             0x03
#define TCA6416_POLARITY0_REG           0x04
#define TCA6416_POLARITY1_REG           0x05
#define TCA6416_CONFIG0_REG             0x06
#define TCA6416_CONFIG1_REG             0x07

#define TCA6416_NR_GPIOS                16

#define TCA6424_INPUT0_REG              0x00
#define TCA6424_INPUT1_REG              0x01
#define TCA6424_INPUT2_REG              0x02
#define TCA6424_OUTPUT0_REG             0x04
#define TCA6424_OUTPUT1_REG             0x05
#define TCA6424_OUTPUT2_REG             0x06
#define TCA6424_POLARITY0_REG           0x08
#define TCA6424_POLARITY1_REG           0x09
#define TCA6424_POLARITY2_REG           0x0A
#define TCA6424_CONFIG0_REG             0x0C
#define TCA6424_CONFIG1_REG             0x0D
#define TCA6424_CONFIG2_REG             0x0E

#define TCA6424_NR_GPIOS                24

#define TCA64XX_NR_GPIO_MAX             TCA6424_NR_GPIOS

/* 1us (datasheet: reset pulse duration (Tw) is 4ns */
#define TCA64XX_TW                      1
/* 1us (datasheet: time to reset (Treset) is 600ns */
#define TCA64XX_TRESET                  1

#define TCA64XX_IRQ_TYPE_EDGE_BOTH      0x00000000
#define TCA64XX_IRQ_TYPE_EDGE_RISING    0x00000001
#define TCA64XX_IRQ_TYPE_EDGE_FALLING   0x00000002
#define TCA64XX_IRQ_TYPE_LEVEL_HIGH     0x00000001
#define TCA64XX_IRQ_TYPE_LEVEL_LOW      0x00000002

#define TCA64XX_IRQ_TYPE_EDGE           0x00000000
#define TCA64XX_IRQ_TYPE_LEVEL          0x00000001

#define tca64xx_irq_type_is_level(driver_data, gpio) \
    (tca64xx_get_gpio_triggering_type(driver_data, gpio) == TCA64XX_IRQ_TYPE_LEVEL)
#define tca64xx_irq_type_is_edge(driver_data, gpio) \
    (tca64xx_get_gpio_triggering_type(driver_data, gpio) == TCA64XX_IRQ_TYPE_EDGE)
#define tca64xx_irq_edge_trigger_is_both(driver_data, gpio) \
    (tca64xx_get_gpio_triggering_level(driver_data, gpio) == \
     TCA64XX_IRQ_TYPE_EDGE_BOTH)
#define tca64xx_irq_edge_trigger_is_falling(driver_data, gpio) \
    (tca64xx_get_gpio_triggering_level(driver_data, gpio) == \
     TCA64XX_IRQ_TYPE_EDGE_FALLING)
#define tca64xx_irq_edge_trigger_is_rising(driver_data, gpio) \
    (tca64xx_get_gpio_triggering_level(driver_data, gpio) == \
     TCA64XX_IRQ_TYPE_EDGE_RISING)
#define tca64xx_irq_level_trigger_is_low(driver_data, gpio) \
    (tca64xx_get_gpio_triggering_level(driver_data, gpio) == \
     TCA64XX_IRQ_TYPE_LEVEL_LOW)
#define tca64xx_irq_level_trigger_is_high(driver_data, gpio) \
    (tca64xx_get_gpio_triggering_level(driver_data, gpio) == \
     TCA64XX_IRQ_TYPE_LEVEL_HIGH)

#define WORKER_DEFPRIO                  50
#define WORKER_STACKSIZE                1024

#define TCA64XX_POLLING_TIME_US         500000

/* disable the verbose debug output */
#undef lldbg
#define lldbg(x...)
/* Enable error log */
#define lldbg_error lowsyslog

extern int irq_unexpected_isr(int irq, FAR void *context);

struct tca64xx_platform_data {
    tca64xx_part part;
    struct i2c_dev_s *dev;
    uint8_t addr;
    uint32_t reset;
    uint32_t irq;
    /* should be in another struct */
    uint32_t in;
    uint32_t intstat;
    uint32_t mask;
    uint32_t trigger;
    uint32_t level;

    struct list_head list;
    xcpt_t irq_vector[TCA64XX_NR_GPIO_MAX];
    uint16_t gpio_base[TCA64XX_NR_GPIO_MAX];

    struct work_s work;
    int worker_id;
    bool worker_exit;
};

/*
 * Since the Nuttx IRQ handlers do not allow to pass private data,
 * store the platform_data pointer in a list
 */
LIST_DECLARE(tca64xx_irq_pdata_list);

static uint8_t get_nr_gpios(tca64xx_part part)
{
    switch (part) {
    case TCA6408_PART:
        return TCA6408_NR_GPIOS;
    case TCA6416_PART:
        return TCA6416_NR_GPIOS;
    case TCA6424_PART:
        return TCA6424_NR_GPIOS;
    default:
        lldbg_error("%s: invalid part number %d\n", __func__, part);
        return -EINVAL;
    }

}
static uint8_t get_config_reg(tca64xx_part part, uint8_t which)
{
    uint8_t reg, nr_gpios = get_nr_gpios(part);

    switch (part) {
    case TCA6408_PART:
        reg = TCA6408_CONFIG_REG;
        break;
    case TCA6416_PART:
        reg = TCA6416_CONFIG0_REG;
        break;
    case TCA6424_PART:
        reg = TCA6424_CONFIG0_REG;
        break;
    default:
        lldbg_error("%s: invalid part number %d\n", __func__, part);
        return -EINVAL;
    }

    if (which >= nr_gpios) {
        lldbg_error("%s: invalid parameter which=%d\n", __func__, which);
        return -EINVAL;
    }

    return reg + (which / 8);
}

static uint8_t get_polarity_reg(tca64xx_part part, uint8_t which)
{
    uint8_t reg, nr_gpios = get_nr_gpios(part);

    switch (part) {
    case TCA6408_PART:
        reg = TCA6408_POLARITY_REG;
        break;
    case TCA6416_PART:
        reg = TCA6416_POLARITY0_REG;
        break;
    case TCA6424_PART:
        reg = TCA6424_POLARITY0_REG;
        break;
    default:
        lldbg_error("%s: invalid part number %d\n", __func__, part);
        return -EINVAL;
    }

    if (which >= nr_gpios) {
        lldbg_error("%s: invalid parameter which=%d\n", __func__, which);
        return -EINVAL;
    }

    return reg + (which / 8);
}

static uint8_t get_output_reg(tca64xx_part part, uint8_t which)
{
    uint8_t reg, nr_gpios = get_nr_gpios(part);

    switch (part) {
    case TCA6408_PART:
        reg = TCA6408_OUTPUT_REG;
        break;
    case TCA6416_PART:
        reg = TCA6416_OUTPUT0_REG;
        break;
    case TCA6424_PART:
        reg = TCA6424_OUTPUT0_REG;
        break;
    default:
        lldbg_error("%s: invalid part number %d\n", __func__, part);
        return -EINVAL;
    }

    if (which >= nr_gpios) {
        lldbg_error("%s: invalid parameter which=%d\n", __func__, which);
        return -EINVAL;
    }

    return reg + (which / 8);
}

static uint8_t get_input_reg(tca64xx_part part, uint8_t which)
{
    uint8_t reg, nr_gpios = get_nr_gpios(part);

    switch (part) {
    case TCA6408_PART:
        reg = TCA6408_INPUT_REG;
        break;
    case TCA6416_PART:
        reg = TCA6416_INPUT0_REG;
        break;
    case TCA6424_PART:
        reg = TCA6424_INPUT0_REG;
        break;
    default:
        lldbg_error("%s: invalid part number %d\n", __func__, part);
        return -EINVAL;
    }

    if (which >= nr_gpios) {
        lldbg_error("%s: invalid parameter which=%d\n", __func__, which);
        return -EINVAL;
    }

    return reg + (which / 8);
}

static struct tca64xx_platform_data *get_pdata(uint32_t irq)
{
    struct list_head *iter;
    struct tca64xx_platform_data *tca64xx;

    list_foreach(&tca64xx_irq_pdata_list, iter) {
        tca64xx = list_entry(iter, struct tca64xx_platform_data, list);
        if ((tca64xx->irq != TCA64XX_IO_UNUSED) && (tca64xx->irq == irq)) {
            return tca64xx;
        }
    }
    return NULL;
}

static int i2c_get(void *driver_data, uint8_t regaddr, uint8_t *val)
{
    int ret;
    struct tca64xx_platform_data *tca64xx = driver_data;
    struct i2c_dev_s *dev = tca64xx->dev;
    uint8_t addr = tca64xx->addr;
    struct i2c_msg_s msg[] = {
        {
         .addr = addr,
         .flags = 0,
         .buffer = &regaddr,
         .length = 1,
        },
        {
         .addr = addr,
         .flags = I2C_M_READ,
         .buffer = val,
         .length = 1,
        },
    };

    if (!dev) {
        return -EINVAL;
    }

    I2C_SETADDRESS(dev, addr, 7);

    ret = I2C_TRANSFER(dev, msg, 2);
    if (ret == OK) {
        lldbg("%s: addr=0x%02hhX, regaddr=0x%02hhX: read 0x%02hhX\n",
              __func__, addr, regaddr, *val);
    } else {
        lldbg_error("%s: addr=0x%02hhX, regaddr=0x%02hhX: failed, ret=%d!\n",
              __func__, addr, regaddr, ret);
    }

    return ret;
}

static int i2c_set(void *driver_data, uint8_t regaddr, uint8_t val)
{
    int ret;
    struct tca64xx_platform_data *tca64xx = driver_data;
    struct i2c_dev_s *dev = tca64xx->dev;
    uint8_t addr = tca64xx->addr;
    uint8_t cmd[2] = { regaddr, val };
    struct i2c_msg_s msg[] = {
        {
         .addr = addr,
         .flags = 0,
         .buffer = cmd,
         .length = 2,
        },
    };

    if (!dev) {
        return -EINVAL;
    }

    I2C_SETADDRESS(dev, addr, 7);

    ret = I2C_TRANSFER(dev, msg, 1);
    if (ret == OK) {
        lldbg("%s: addr=0x%02hhX, regaddr=0x%02hhX, val=0x%02hhX\n",
              __func__, addr, regaddr, val);
    } else {
        lldbg_error("%s: addr=0x%02hhX, regaddr=0x%02hhX: failed, ret=%d!\n",
              __func__, addr, regaddr, ret);
    }

    return ret;
}

int tca64xx_reset(void *driver_data, bool en)
{
    struct tca64xx_platform_data *tca64xx = driver_data;

    /*
     * TCA64XX reset pin is active low.
     * If en = 1, maintain chip under reset.
     * If en = 0, get chip out of reset.
     */
    lldbg("%s: gpio=0x%02hhX, en=%hhu\n", __func__, tca64xx->reset, en);
    gpio_direction_out(tca64xx->reset, 0);
    if (en == 0) {
        /* datasheet: reset pulse minimum duration (Tw) is 4ns */
        usleep(TCA64XX_TW);
        gpio_direction_out(tca64xx->reset, 1);
        /* datasheet: time to reset (Treset) is 600ns */
        usleep(TCA64XX_TRESET);
    }

    return 0;
}

void tca64xx_set(void *driver_data, uint8_t which, uint8_t value)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    uint8_t reg, val;
    int ret;

    lldbg("%s: addr=0x%02hhX, which=%hhu, val=%hhu\n", __func__,
          tca64xx->addr, which, val);
    /* Set output pins default value (before configuring it as output)
     *
     * The Output Port Register (register 1) shows the outgoing logic
     * levels of the pins defined as outputs by the Configuration Register.
     */
    reg = get_output_reg(tca64xx->part, which);
    if (reg < 0)
        return;
    ret = i2c_get(tca64xx, reg, &val);
    if (ret != 0)
        return;
    lldbg("%s: current val=0x%02hhX\n", __func__, val);
    if (value) {
        val |= (1 << (which % 8));
    } else {
        val &= ~(1 << (which % 8));
    }
    lldbg("%s: new val=0x%02hhX\n", __func__, val);
    ret = i2c_set(tca64xx, reg, val);
}

void tca64xx_set_direction_in(void *driver_data, uint8_t which)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    uint8_t reg, val;
    int ret;

    lldbg("%s: addr=0x%02hhX, which=%hhu\n", __func__, tca64xx->addr, which);

    /* Configure pin as input
     *
     * The Configuration Register configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    reg = get_config_reg(tca64xx->part, which);
    if (reg < 0)
        return;
    ret = i2c_get(tca64xx, reg, &val);
    if (ret != 0) {
        lldbg_error("%s: current cfg=0x%02X\n", __func__, val);
        return;
    }
    val |= (1 << (which % 8));
    lldbg("%s: new cfg=0x%02X\n", __func__, val);
    ret = i2c_set(tca64xx, reg, val);
}

void tca64xx_set_direction_out(void *driver_data, uint8_t which, uint8_t value)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    uint8_t reg, val;
    int ret;

    lldbg("%s: addr=0x%02hhX, which=%hhu\n", __func__, tca64xx->addr, which);
    /* Configure pin as output
     *
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    reg = get_config_reg(tca64xx->part, which);
    if (reg < 0) {
        return;
    }
    ret = i2c_get(tca64xx, reg, &val);
    if (ret != 0) {
        return;
    }
    lldbg("%s: current cfg=0x%02X\n", __func__, val);
    val &= ~(1 << (which % 8));
    lldbg("%s: new cfg=0x%02X\n", __func__, val);
    ret = i2c_set(tca64xx, reg, val);
    if (ret != 0) {
        return;
    }

    tca64xx_set(driver_data, which, value);
}

int tca64xx_get_direction(void *driver_data, uint8_t which)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    uint8_t reg;
    uint8_t direction;
    int ret;

    lldbg("addr=0x%02hhX, which=%hhu", tca64xx->addr, which);
    /*
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    reg = get_config_reg(tca64xx->part, which);
    if (reg < 0) {
        return reg;
    }
    ret = i2c_get(tca64xx, reg, &direction);
    if (ret != 0) {
        return -EIO;
    }
    direction = (direction & (1 << (which % 8))) >> (which % 8);

    return direction;
}

int tca64xx_set_polarity_inverted(void *driver_data, uint8_t which,
                                  uint8_t inverted)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    uint8_t reg, polarity;
    int ret;

    lldbg("%s: addr=0x%02hhX, which=%hhu inverted=%hhu\n", __func__,
          tca64xx->addr, which, inverted);
    /* Configure pin polarity inversion
     *
     * The Polarity Inversion Register (register 2) allows
     * polarity inversion of pins defined as inputs by the Configuration
     * Register. If a bit in this register is set (written with 1),
     * the corresponding port pin's polarity is inverted. If a bit in
     * this register is cleared (written with a 0), the corresponding
     * port pin's original polarity is retained.
     */
    reg = get_polarity_reg(tca64xx->part, which);
    if (reg < 0) {
        return reg;
    }
    ret = i2c_get(tca64xx, reg, &polarity);
    if (ret != 0) {
        return -EIO;
    }
    lldbg("%s: current polarity reg=0x%02hhX\n", __func__, polarity);
    if (inverted) {
        polarity |= (1 << (which % 8));
    } else {
        polarity &= ~(1 << (which % 8));
    }
    lldbg("%s: new polarity reg=0x%02hhX\n", __func__, polarity);
    ret = i2c_set(tca64xx, reg, polarity);
    if (ret != 0) {
        return -EIO;
    }

    return 0;
}

int tca64xx_get_polarity_inverted(void *driver_data, uint8_t which)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    uint8_t reg, polarity;
    int ret;

    lldbg("%s: addr=0x%02hhX, which=%hhu\n", __func__, tca64xx->addr, which);
    /*
     * The Configuration Register (register 3) configures the direction of
     * the I/O pins. If a bit in this register is set to 1,
     * the corresponding port pin is enabled as an input with a
     * high-impedance output driver. If a bit in this register is
     * cleared to 0, the corresponding port pin is enabled as an output.
     */
    reg = get_polarity_reg(tca64xx->part, which);
    if (reg < 0) {
        return reg;
    }
    ret = i2c_get(tca64xx, reg, &polarity);
    if (ret != 0) {
        return -EIO;
    }
    lldbg("%s: polarity reg=0x%02hhX\n", __func__, polarity);
    polarity = (polarity & (1 << (which % 8))) >> (which % 8);
    lldbg("%s: polarity=0x%hhu\n", __func__, polarity);

    return polarity;
}

static int tca64xx_get_gpio_triggering_type(void *driver_data, uint8_t which)
{
    struct tca64xx_platform_data *tca64xx = driver_data;

    return (tca64xx->trigger >> which) & 1;
}

static int tca64xx_get_gpio_triggering_level(void *driver_data, uint8_t which)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    int shift = which << 1;

    return (tca64xx->level >> shift) & 3;
}

static void intstat_update(void *driver_data, uint32_t in, uint32_t mask)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    int pin;
    uint32_t diff, nr_gpios = get_nr_gpios(tca64xx->part);
    irqstate_t flags;

    flags = irqsave();

    /* Check the changed bits from last read */
    in = (tca64xx->in & ~mask) | (in & mask);
    diff = tca64xx->in ^ in;

    if (!diff) {
        irqrestore(flags);
        return;
    }

    tca64xx->in = in;

    /*
     * TCA64XX doesn't support irq trigger, then we have to do it in software.
     */
    for (pin = 0; pin < nr_gpios; pin++) {
        if (tca64xx_irq_type_is_edge(tca64xx, pin) && (diff & 1)) {
            /* GPIO level change. Set interrupt in function of edge type */
            if (tca64xx_irq_edge_trigger_is_both(tca64xx, pin) ||
                (!(in & 1) &&
                 tca64xx_irq_edge_trigger_is_falling(tca64xx, pin)) ||
                ((in & 1) &&
                 tca64xx_irq_edge_trigger_is_rising(tca64xx, pin))) {
                tca64xx->intstat |= 1 << pin;
            }
        } else if (tca64xx_irq_type_is_level(tca64xx, pin)) {
            /* Trigger is set to level. Set intstat if in match level type. */
            if (((in & 1) &&
                 tca64xx_irq_level_trigger_is_high(tca64xx, pin)) ||
                (!(in & 1) &&
                 tca64xx_irq_level_trigger_is_low(tca64xx, pin))) {
                tca64xx->intstat |= 1 << pin;
            }
        }
        diff >>= 1, in >>= 1;
    }

    irqrestore(flags);
}

static void tca64xx_registers_update(void *driver_data)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    int i, ret;
    uint32_t in = 0;
    uint8_t val, reg;

    for (i = 0; i < (get_nr_gpios(tca64xx->part)); i += 8) {
        reg = get_input_reg(tca64xx->part, i);
        if (reg < 0) {
            return;
        }
        ret = i2c_get(tca64xx, reg, &val);
        if (ret != 0) {
            return;
        }
        in |= (val << i);
    }
    lldbg("%s: in=0x%08x\n", __func__, in);

    /* Update the input status with the 32 bits read from the expander */
    intstat_update(driver_data, in, ~0);
}

uint8_t tca64xx_get(void *driver_data, uint8_t which)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    uint8_t reg, val;
    int ret;

    lldbg("%s: addr=0x%02hhX, which=%hhu\n", __func__, tca64xx->addr, which);
    /*
     * The Input Port Register reflects the incoming logic
     * levels of the pins, regardless of whether the pin is defined as an
     * input or an output by the Configuration Register. They act only on
     * read operation.
     */
    reg = get_input_reg(tca64xx->part, which);
    if (reg < 0) {
        return -EINVAL;
    }
    ret = i2c_get(tca64xx, reg, &val);
    if (ret != 0) {
        return -EIO;
    }
    lldbg("%s: input reg(0x%02hhX)=0x%02hhX\n", __func__, reg, val);

    /* Update the input status with the 8 bits read from the expander */
    intstat_update(driver_data,
                   val << (which & 0xF8),
                   0xFF << (which & 0xF8));

    val &= (1 << (which % 8));
    val = !!val;
    lldbg("%s: val=%hhu\n", __func__, val);

    return val;
}

uint8_t tca64xx_line_count(void *driver_data)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    return get_nr_gpios(tca64xx->part);
}

int tca64xx_gpio_mask_irq(void *driver_data, uint8_t which)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    irqstate_t flags;

    flags = irqsave();
    tca64xx->mask |= (1 << which);
    irqrestore(flags);

    return 0;
}

int tca64xx_gpio_unmask_irq(void *driver_data, uint8_t which)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    irqstate_t flags;

    flags = irqsave();
    tca64xx->mask &= ~(1 << which);
    irqrestore(flags);

    return 0;
}

int tca64xx_gpio_clear_interrupt(void *driver_data, uint8_t which)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    irqstate_t flags;

    flags = irqsave();
    tca64xx->intstat &= ~(1 << which);
    irqrestore(flags);

    return 0;
}

uint32_t tca64xx_gpio_get_interrupt(void *driver_data)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    uint32_t mask;
    uint32_t intstat;
    irqstate_t flags;

    flags = irqsave();
    mask = ~tca64xx->mask;
    intstat = tca64xx->intstat & mask;
    irqrestore(flags);

    return intstat;
}

static void tca64xx_set_gpio_trigger(void *driver_data, uint8_t which, int trigger)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    irqstate_t flags;

    flags = irqsave();
    if (trigger) {
        tca64xx->trigger |= TCA64XX_IRQ_TYPE_LEVEL << which;
    } else {
        tca64xx->trigger &= ~(TCA64XX_IRQ_TYPE_LEVEL << which);
    }
    irqrestore(flags);
}

static void tca64xx_set_gpio_level(void *driver_data, uint8_t which, int level)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    int shift = which << 1;
    irqstate_t flags;

    flags = irqsave();
    tca64xx->level &= ~(0x03 << shift);
    tca64xx->level |= level << shift;
    irqrestore(flags);
}

static int tca64xx_set_gpio_triggering(void *driver_data, uint8_t which,
                                       int trigger)
{
    switch (trigger) {
    case IRQ_TYPE_NONE:
    case IRQ_TYPE_EDGE_BOTH:
        tca64xx_set_gpio_trigger(driver_data, which, TCA64XX_IRQ_TYPE_EDGE);
        tca64xx_set_gpio_level(driver_data, which, TCA64XX_IRQ_TYPE_EDGE_BOTH);
        break;
    case IRQ_TYPE_EDGE_RISING:
        tca64xx_set_gpio_trigger(driver_data, which, TCA64XX_IRQ_TYPE_EDGE);
        tca64xx_set_gpio_level(driver_data, which,
                               TCA64XX_IRQ_TYPE_EDGE_RISING);
        break;
    case IRQ_TYPE_EDGE_FALLING:
        tca64xx_set_gpio_trigger(driver_data, which, TCA64XX_IRQ_TYPE_EDGE);
        tca64xx_set_gpio_level(driver_data, which,
                               TCA64XX_IRQ_TYPE_EDGE_FALLING);
        break;
    case IRQ_TYPE_LEVEL_HIGH:
        tca64xx_set_gpio_trigger(driver_data, which, TCA64XX_IRQ_TYPE_LEVEL);
        tca64xx_set_gpio_level(driver_data, which, TCA64XX_IRQ_TYPE_LEVEL_HIGH);
        break;
    case IRQ_TYPE_LEVEL_LOW:
        tca64xx_set_gpio_trigger(driver_data, which, TCA64XX_IRQ_TYPE_LEVEL);
        tca64xx_set_gpio_level(driver_data, which, TCA64XX_IRQ_TYPE_LEVEL_LOW);
        break;
    default:
        break;
    }

    return 0;
}

static void _tca64xx_gpio_irq_handler(void *data)
{
    struct tca64xx_platform_data *tca64xx = data;
    void *context = data;
    uint32_t irqstat;
    uint16_t base;
    int pin;
    uint8_t nr_gpios = get_nr_gpios(tca64xx->part);

    tca64xx_registers_update(data);
    irqstat = tca64xx_gpio_get_interrupt(data);

    while (irqstat) {
        /* Now process each IRQ pending in the GPIO */
        for (pin = 0; (pin < nr_gpios) && (irqstat != 0);
             pin++, irqstat >>= 1) {
            if (irqstat & 1) {
                base = tca64xx->gpio_base[pin];
                tca64xx->irq_vector[pin] (base + pin, context);
                tca64xx_gpio_clear_interrupt(data, pin);
            }
        }
    }

    gpio_clear_interrupt(tca64xx->irq);
    gpio_unmask_irq(tca64xx->irq);
}

static int tca64xx_trigger_worker(struct tca64xx_platform_data *tca64xx)
{
    /*
     * We need to perform some i2c operations to get the gpios that cause
     * the interrupt. We can't do these operation in irq then do it in thread.
     * The tca64xx use low level irq trigger. We need to disable gpio
     * interrupt until thread done with tca64xx irq.
     */
    if (work_available(&tca64xx->work)) {
        gpio_mask_irq(tca64xx->irq);
        work_queue(HPWORK, &tca64xx->work, _tca64xx_gpio_irq_handler,
                   tca64xx, 0);
    }

    return 0;
}

static int tca64xx_gpio_irq_handler(int irq, void *context)
{
    struct tca64xx_platform_data *tca64xx = get_pdata(irq);

    if (!tca64xx) {
        return -ENODEV;
    }

    tca64xx_trigger_worker(tca64xx);

    return OK;
}

int tca64xx_gpio_irqattach(void *driver_data, uint8_t which, xcpt_t isr,
                           uint8_t base)
{
    struct tca64xx_platform_data *tca64xx = driver_data;
    irqstate_t flags;

    flags = irqsave();

    /*
     * If the new ISR is NULL, then the ISR is being detached.
     * In this case, disable the ISR and direct any interrupts
     * to the unexpected interrupt handler.
     */
    if (isr == NULL) {
        isr = irq_unexpected_isr;
    }

    /* Save the new ISR in the table. */
    tca64xx->irq_vector[which] = isr;
    tca64xx->gpio_base[which] = base;

    irqrestore(flags);

    return OK;
}

void tca64xx_activate(void *driver_data, uint8_t which)
{

}

void tca64xx_deactivate(void *driver_data, uint8_t which)
{

}

struct gpio_ops_s tca64xx_gpio_ops = {
    .get_direction = tca64xx_get_direction,
    .direction_in = tca64xx_set_direction_in,
    .direction_out = tca64xx_set_direction_out,
    .activate = tca64xx_activate,
    .get_value = tca64xx_get,
    .set_value = tca64xx_set,
    .deactivate = tca64xx_deactivate,
    .line_count = tca64xx_line_count,
    .irqattach = tca64xx_gpio_irqattach,
    .set_triggering = tca64xx_set_gpio_triggering,
    .mask_irq = tca64xx_gpio_mask_irq,
    .unmask_irq = tca64xx_gpio_unmask_irq,
    .clear_interrupt = tca64xx_gpio_clear_interrupt,
};

static int tca64xx_polling_worker(int argc, char *argv[])
{
    // Get the private data passed via argc/argv
    struct tca64xx_platform_data *tca64xx =
        (struct tca64xx_platform_data *) strtol(argv[1], NULL, 16);

    if (!tca64xx) {
        lldbg_error("%s: no tca64xx driver context\n", __func__);
        return ERROR;
    }

    /* Sometimes the tca64xx loses interrupt. Re-read to generate interrupt */
    while (!tca64xx->worker_exit) {
        tca64xx_trigger_worker(tca64xx);
        usleep(TCA64XX_POLLING_TIME_US);
    }
    return 0;
}

int tca64xx_init(void **driver_data, tca64xx_part part, struct i2c_dev_s *dev,
                 uint8_t addr, uint32_t reset, uint32_t irq, int gpio_base)
{
    struct tca64xx_platform_data *tca64xx;
    const char* argv[2];
    char buf[16];
    uint8_t nr_gpios;
    int ret;
    irqstate_t flags;

    flags = irqsave();

    nr_gpios = get_nr_gpios(part);
    if (nr_gpios < 0) {
        lldbg_error("%s: invalid part=%d\n", __func__, part);
        irqrestore(flags);
        return -EINVAL;
    }

    tca64xx = malloc(sizeof(struct tca64xx_platform_data));
    if (!tca64xx) {
        irqrestore(flags);
        return -ENOMEM;
    }

    tca64xx->dev = dev;
    tca64xx->addr = addr;
    tca64xx->irq = irq;
    tca64xx->reset = reset;
    tca64xx->part = part;
    tca64xx->mask = (1 << nr_gpios) - 1;
    tca64xx->in = 0;
    tca64xx->intstat = 0;

    if (reset != TCA64XX_IO_UNUSED)
        tca64xx_reset(tca64xx, 0);

    tca64xx_registers_update(tca64xx);

    register_gpio_chip(&tca64xx_gpio_ops, gpio_base, tca64xx);

    if (irq != TCA64XX_IO_UNUSED) {
        list_add(&tca64xx_irq_pdata_list, &tca64xx->list);
        gpio_activate(tca64xx->irq);
        gpio_direction_in(tca64xx->irq);
        gpio_irqattach(tca64xx->irq, tca64xx_gpio_irq_handler);
        /* Set to EDGE_BOTH to catch missed interrupt */
        set_gpio_triggering(tca64xx->irq, IRQ_TYPE_EDGE_BOTH);
        gpio_clear_interrupt(tca64xx->irq);
        gpio_unmask_irq(tca64xx->irq);

        /* Initialize the work queue */
        tca64xx->work.worker = NULL;

        /* Create polling worker */
        tca64xx->worker_exit = false;
        sprintf(buf, "%p", tca64xx);
        argv[0] = buf;
        argv[1] = NULL;
        ret = task_create("tca64xx_worker",
                          WORKER_DEFPRIO, WORKER_STACKSIZE,
                          tca64xx_polling_worker,
                          (char * const*) argv);
        if (ret == ERROR) {
            lldbg_error("%s: Failed to create polling worker\n", __func__);
            irqrestore(flags);
            return ERROR;
        }
        tca64xx->worker_id = ret;
    }

    *driver_data = tca64xx;

    irqrestore(flags);
    return 0;
}

void tca64xx_deinit(void *driver_data)
{
    struct tca64xx_platform_data *pdata, *tca64xx = driver_data;
    struct list_head *iter, *iter_next;
    int ret, status;
    irqstate_t flags;

    if (!driver_data) {
        lldbg_error("%s: NULL driver_data, aborting\n", __func__);
        return;
    }

    flags = irqsave();

    /* Unregister IRQ */
    if (tca64xx->irq != TCA64XX_IO_UNUSED) {
        /* Destroy polling worker */
        if (tca64xx->worker_id > 0) {
            tca64xx->worker_exit = true;
            ret = waitpid(tca64xx->worker_id, &status, 0);
            if (ret < 0) {
                lldbg_error("%s: waitpid failed with ret=%d\n", __func__, ret);
            }
        }
        /* Unregister IRQ */
        gpio_mask_irq(tca64xx->irq);
        gpio_irqattach(tca64xx->irq, NULL);
        list_foreach_safe(&tca64xx_irq_pdata_list, iter, iter_next) {
            pdata = list_entry(iter, struct tca64xx_platform_data, list);
            if (pdata == tca64xx) {
                list_del(iter);
            }
        }
    }

    /* Unregister gpio_chip */
    unregister_gpio_chip(driver_data);

    /* Free driver data */
    free(driver_data);

    irqrestore(flags);
}
