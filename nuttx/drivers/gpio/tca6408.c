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
#include <string.h>
#include <nuttx/gpio/tca6408.h>
#include <nuttx/gpio.h>
#include <nuttx/wqueue.h>
#include <pthread.h>

#define TCA6408_INPUT_REG       0x00
#define TCA6408_OUTPUT_REG      0x01
#define TCA6408_POLARITY_REG    0x02
#define TCA6408_CONFIG_REG      0x03

#define TCA6408_TW      1       /* 1us (datasheet: reset pulse duration (Tw) is 4ns */
#define TCA6408_TRESET  1       /* 1us (datasheet: time to reset (Treset) is 600ns */

#define TCA6408_NR_GPIO 8

#define TCA6408_IRQ_TYPE_EDGE_BOTH      0x00000000
#define TCA6408_IRQ_TYPE_EDGE_RISING    0x00000001
#define TCA6408_IRQ_TYPE_EDGE_FALLING   0x00000002
#define TCA6408_IRQ_TYPE_LEVEL_HIGH     0x00000001
#define TCA6408_IRQ_TYPE_LEVEL_LOW      0x00000002

#define TCA6408_IRQ_TYPE_EDGE           0x00000000
#define TCA6408_IRQ_TYPE_LEVEL          0x00000001

#define tca6408_irq_type_is_level(gpio) \
    (tca6408_get_gpio_triggering_type(gpio) == TCA6408_IRQ_TYPE_LEVEL)
#define tca6408_irq_type_is_edge(gpio) \
    (tca6408_get_gpio_triggering_type(gpio) == TCA6408_IRQ_TYPE_EDGE)
#define tca6408_irq_edge_trigger_is_both(gpio) \
    (tca6408_get_gpio_triggering_level(gpio) == TCA6408_IRQ_TYPE_EDGE_BOTH)
#define tca6408_irq_edge_trigger_is_falling(gpio) \
    (tca6408_get_gpio_triggering_level(gpio) == TCA6408_IRQ_TYPE_EDGE_FALLING)
#define tca6408_irq_edge_trigger_is_rising(gpio) \
    (tca6408_get_gpio_triggering_level(gpio) == TCA6408_IRQ_TYPE_EDGE_RISING)
#define tca6408_irq_level_trigger_is_low(gpio) \
    (tca6408_get_gpio_triggering_level(gpio) == TCA6408_IRQ_TYPE_LEVEL_LOW)
#define tca6408_irq_level_trigger_is_high(gpio) \
    (tca6408_get_gpio_triggering_level(gpio) == TCA6408_IRQ_TYPE_LEVEL_HIGH)

/* disable the verbose debug output */
#undef lldbg
#define lldbg(x...)

extern int irq_unexpected_isr(int irq, FAR void *context);

struct tca6408_platform_data {
    struct i2c_dev_s *dev;
    uint8_t addr;
    uint8_t reset;
    uint8_t irq;
    /* should be in another struct */
    uint8_t in;
    uint8_t intstat;
    uint8_t mask;
    uint8_t trigger;
    uint16_t level;

    xcpt_t irq_vector[TCA6408_NR_GPIO];
    uint8_t gpio_base[TCA6408_NR_GPIO];

    struct work_s work;
    pthread_t thread;
};

static struct tca6408_platform_data g_tca6408;

static int i2c_get(uint8_t addr, uint8_t regaddr, uint8_t * val)
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
        lldbg("addr=0x%02hhX, regaddr=0x%02hhX: failed!\n", addr, regaddr);
    }

    return ret;
}

static int i2c_set(uint8_t addr, uint8_t regaddr, uint8_t val)
{
    int ret;
    struct i2c_dev_s *dev = g_tca6408.dev;
    uint8_t cmd[2] = { regaddr, val };
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
        return lldbg("current cfg=0x%02X\n", reg);
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

    lldbg("addr=0x%02hhX, which=%hhu inverted=%hhu\n", addr, which, inverted);
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

    lldbg("addr=0x%02hhX, which=%hhu, val=%hhu\n", addr, which, val);
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

static int tca6408_get_gpio_triggering_type(uint8_t which)
{
    return (g_tca6408.trigger >> which) & 1;
}

static int tca6408_get_gpio_triggering_level(uint8_t which)
{
    int shift = which << 1;
    return (g_tca6408.level >> shift) & 3;
}

static void intstat_update(uint8_t in)
{
    int pin;
    uint8_t diff;

    diff = g_tca6408.in ^ in;
    g_tca6408.in = in;

    /*
     * TCA6408 doesn't support irq trigger, then we have to do it in software.
     */
    for (pin = 0; pin < TCA6408_NR_GPIO; pin++) {
        if (tca6408_irq_type_is_edge(pin) && (diff & 1)) {
            /* GPIO level change. Set interrupt in function of edge type */
            if (tca6408_irq_edge_trigger_is_both(pin) ||
                ((in & 1) == 0 && tca6408_irq_edge_trigger_is_falling(pin)) ||
                ((in & 1) == 1 && tca6408_irq_edge_trigger_is_rising(pin)))
                g_tca6408.intstat |= 1 << pin;
        } else if (tca6408_irq_type_is_level(pin)) {
            /* Trigger is set to level. Set intstat if in match level type. */
            if (((in & 1) == 1 && tca6408_irq_level_trigger_is_high(pin)) ||
                ((in & 1) == 0 && tca6408_irq_level_trigger_is_low(pin)))
                g_tca6408.intstat |= 1 << pin;
        }
        diff >>= 1, in >>= 1;
    }
}

static void tca6408_registers_update()
{
    int ret;
    uint8_t in;
    uint8_t addr = g_tca6408.addr;

    ret = i2c_get(addr, TCA6408_INPUT_REG, &in);
    if (ret != 0)
        return;
    intstat_update(in);
}

uint8_t tca6408_get(uint8_t which)
{
    uint8_t in;
    uint8_t addr = g_tca6408.addr;
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
    intstat_update(in);
    lldbg("input reg=0x%02hhX\n", in);
    in &= (1 << which);
    in = ! !in;
    lldbg("in=%hhu\n", in);

    return in;
}

uint8_t tca6408_line_count(void)
{
    return TCA6408_NR_GPIO;
}

int tca6408_gpio_mask_irq(uint8_t which)
{
    g_tca6408.mask |= (1 << which);
    return 0;
}

int tca6408_gpio_unmask_irq(uint8_t which)
{
    g_tca6408.mask &= ~(1 << which);
    return 0;
}

int tca6408_gpio_clear_interrupt(uint8_t which)
{
    g_tca6408.intstat &= ~(1 << which);
    return 0;
}

uint8_t tca6408_gpio_get_interrupt(void)
{
    uint8_t mask = ~g_tca6408.mask;
    uint8_t intstat = g_tca6408.intstat & mask;
    return intstat;
}

static void tca6408_set_gpio_trigger(uint8_t which, int trigger)
{
    if (trigger)
        g_tca6408.trigger |= TCA6408_IRQ_TYPE_LEVEL << which;
    else
        g_tca6408.trigger &= ~(TCA6408_IRQ_TYPE_LEVEL << which);
}

static void tca6408_set_gpio_level(uint8_t which, int level)
{
    int shift = which << 1;

    g_tca6408.level &= ~(0x03 << shift);
    g_tca6408.level |= level << shift;
}

static int tca6408_set_gpio_triggering(uint8_t which, int trigger)
{
    switch (trigger) {
    case IRQ_TYPE_NONE:
    case IRQ_TYPE_EDGE_BOTH:
        tca6408_set_gpio_trigger(which, TCA6408_IRQ_TYPE_EDGE);
        tca6408_set_gpio_level(which, TCA6408_IRQ_TYPE_EDGE_BOTH);
        break;
    case IRQ_TYPE_EDGE_RISING:
        tca6408_set_gpio_trigger(which, TCA6408_IRQ_TYPE_EDGE);
        tca6408_set_gpio_level(which, TCA6408_IRQ_TYPE_EDGE_RISING);
    case IRQ_TYPE_EDGE_FALLING:
        tca6408_set_gpio_trigger(which, TCA6408_IRQ_TYPE_EDGE);
        tca6408_set_gpio_level(which, TCA6408_IRQ_TYPE_EDGE_FALLING);
    case IRQ_TYPE_LEVEL_HIGH:
        tca6408_set_gpio_trigger(which, TCA6408_IRQ_TYPE_LEVEL);
        tca6408_set_gpio_level(which, TCA6408_IRQ_TYPE_LEVEL_HIGH);
    case IRQ_TYPE_LEVEL_LOW:
        tca6408_set_gpio_trigger(which, TCA6408_IRQ_TYPE_LEVEL);
        tca6408_set_gpio_level(which, TCA6408_IRQ_TYPE_LEVEL_LOW);
    }
    return 0;
}

static void _tca6408_gpio_irq_handler(void *data)
{
    void *context = data;
    uint32_t irqstat;
    uint8_t base;
    uint8_t in;
    int pin;

    tca6408_registers_update();
    irqstat = tca6408_gpio_get_interrupt();
    in = g_tca6408.in;

    while (irqstat) {
        /* Now process each IRQ pending in the GPIO */
        for (pin = 0; pin < TCA6408_NR_GPIO && irqstat != 0;
             pin++, irqstat >>= 1, in >>= 1) {
            if ((irqstat & 1) == 1) {
                base = g_tca6408.gpio_base[pin];
                g_tca6408.irq_vector[pin] (base + pin, context);
                tca6408_gpio_clear_interrupt(pin);
            }
        }

        tca6408_registers_update();
        irqstat = tca6408_gpio_get_interrupt();
        in = g_tca6408.in;
    }

    gpio_clear_interrupt(g_tca6408.irq);
    gpio_unmask_irq(g_tca6408.irq);
}

static int tca6408_gpio_irq_handler(int irq, void *context)
{
    /*
     * We need to perform some i2c operations to get the gpios that cause
     * the interrupt. We can't do these operation in irq then do it in thread.
     * The tca6408 use low level irq trigger. We need to disable gpio interrupt
     * until thread done with tca6408 irq.
     */
    gpio_mask_irq(g_tca6408.irq);
    work_queue(HPWORK, &g_tca6408.work, _tca6408_gpio_irq_handler, context, 0);
    return OK;
}

int tca6408_gpio_irqattach(uint8_t which, xcpt_t isr, uint8_t base)
{
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
    g_tca6408.irq_vector[which] = isr;
    g_tca6408.gpio_base[which] = base;
    irqrestore(flags);

    return OK;
}

void tca6408_activate(uint8_t which)
{

}

void tca6408_deactivate(uint8_t which)
{

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
    .irqattach = tca6408_gpio_irqattach,
    .set_triggering = tca6408_set_gpio_triggering,
    .mask_irq = tca6408_gpio_mask_irq,
    .unmask_irq = tca6408_gpio_unmask_irq,
    .clear_interrupt = tca6408_gpio_clear_interrupt,
};

static void *tca6408_polling(void *data)
{
    /* Sometime, tca6408 loses interrupt. Re read to generate interrupt */
    while (1) {
        gpio_get_value(21);
        usleep(100000);
    }
    return NULL;
}

void tca6408_init(struct i2c_dev_s *dev, uint8_t addr,
                  uint8_t reset, uint8_t irq)
{
    g_tca6408.dev = dev;
    g_tca6408.addr = addr;
    g_tca6408.irq = irq;
    g_tca6408.reset = reset;

    g_tca6408.mask = 0xFF;
    g_tca6408.in = 0;
    tca6408_registers_update(); /* init g_tca6408.in_diff */
    g_tca6408.intstat = 0;
    gpio_activate(g_tca6408.irq);
    gpio_direction_in(g_tca6408.irq);
    gpio_irqattach(g_tca6408.irq, tca6408_gpio_irq_handler);
    /* Set to EDGE_BOTH to catch missed interrupt */
    set_gpio_triggering(g_tca6408.irq, IRQ_TYPE_EDGE_BOTH);
    gpio_clear_interrupt(g_tca6408.irq);
    gpio_unmask_irq(g_tca6408.irq);

    register_gpio_chip(&tca6408_gpio_ops, -1);
    pthread_create(&g_tca6408.thread, NULL, tca6408_polling, NULL);
}
