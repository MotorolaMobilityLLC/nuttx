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
 *
 * Authors: Fabien Parent <fparent@baylibre.com>
 *          Benoit Cousson <bcousson@baylibre.com>
 */

#include <nuttx/gpio.h>
#include <arch/tsb/gpio.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include "tsb_scm.h"
#include "up_arch.h"
#include "nuttx/arch.h"
#include "irq/irq.h"

#define GPIO_BASE           0x40003000
#define GPIO_DATA           (GPIO_BASE)
#define GPIO_ODATA          (GPIO_BASE + 0x4)
#define GPIO_ODATASET       (GPIO_BASE + 0x8)
#define GPIO_ODATACLR       (GPIO_BASE + 0xc)
#define GPIO_DIR            (GPIO_BASE + 0x10)
#define GPIO_DIROUT         (GPIO_BASE + 0x14)
#define GPIO_DIRIN          (GPIO_BASE + 0x18)
#define GPIO_INTMASK        (GPIO_BASE + 0x1c)
#define GPIO_INTMASKSET     (GPIO_BASE + 0x20)
#define GPIO_INTMASKCLR     (GPIO_BASE + 0x24)
#define GPIO_RAWINTSTAT     (GPIO_BASE + 0x28)
#define GPIO_INTSTAT        (GPIO_BASE + 0x2c)
#define GPIO_INTCTRL0       (GPIO_BASE + 0x30)
#define GPIO_INTCTRL1       (GPIO_BASE + 0x34)
#define GPIO_INTCTRL2       (GPIO_BASE + 0x38)
#define GPIO_INTCTRL3       (GPIO_BASE + 0x3c)

#if defined(CONFIG_TSB_CHIP_REV_ES1)
#define NR_GPIO_IRQS 16
#else
#define NR_GPIO_IRQS 27
#endif

/* A table of handlers for each GPIO interrupt */
static xcpt_t tsb_gpio_irq_vector[NR_GPIO_IRQS];
static uint8_t tsb_gpio_irq_gpio_base[NR_GPIO_IRQS];
static volatile uint32_t refcount;

int tsb_gpio_get_direction(uint8_t which)
{
    uint32_t dir = getreg32(GPIO_DIR);
    return !(dir & (1 << which));
}

void tsb_gpio_direction_in(uint8_t which)
{
    putreg32(1 << which, GPIO_DIRIN);
}

void tsb_gpio_direction_out(uint8_t which, uint8_t value)
{
    tsb_gpio_set_value(which, value);
    putreg32(1 << which, GPIO_DIROUT);
}

void tsb_gpio_activate(uint8_t which)
{
    tsb_gpio_initialize();
}

uint8_t tsb_gpio_get_value(uint8_t which)
{
    return !!(getreg32(GPIO_DATA) & (1 << which));
}

void tsb_gpio_set_value(uint8_t which, uint8_t value)
{
    putreg32(1 << which, value ? GPIO_ODATASET : GPIO_ODATACLR);
}

void tsb_gpio_deactivate(uint8_t which)
{
    tsb_gpio_uninitialize();
}

uint8_t tsb_gpio_line_count(void)
{
    return NR_GPIO_IRQS;
}

int tsb_gpio_mask_irq(uint8_t which)
{
    putreg32(1 << which, GPIO_INTMASKSET);
    return 0;
}

int tsb_gpio_unmask_irq(uint8_t which)
{
    putreg32(1 << which, GPIO_INTMASKCLR);
    return 0;
}

int tsb_gpio_clear_interrupt(uint8_t which)
{
    putreg32(1 << which, GPIO_RAWINTSTAT);
    return 0;
}

uint32_t tsb_gpio_get_raw_interrupt(void)
{
    return getreg32(GPIO_RAWINTSTAT);
}

uint32_t tsb_gpio_get_interrupt(void)
{
    return getreg32(GPIO_INTSTAT);
}

int tsb_set_gpio_triggering(uint8_t which, int trigger)
{
    int tsb_trigger;
    uint32_t reg = GPIO_INTCTRL0 + ((which >> 1) & 0xfc);
    uint32_t shift = 4 * (which & 0x7);
    uint32_t v = getreg32(reg);

    switch(trigger) {
    case IRQ_TYPE_EDGE_RISING:
        tsb_trigger = TSB_IRQ_TYPE_EDGE_RISING;
        break;
    case IRQ_TYPE_EDGE_FALLING:
        tsb_trigger = TSB_IRQ_TYPE_EDGE_FALLING;
        break;
    case IRQ_TYPE_EDGE_BOTH:
        tsb_trigger = TSB_IRQ_TYPE_EDGE_BOTH;
        break;
    case IRQ_TYPE_LEVEL_HIGH:
        tsb_trigger = TSB_IRQ_TYPE_LEVEL_HIGH;
        break;
    case IRQ_TYPE_LEVEL_LOW:
        tsb_trigger = TSB_IRQ_TYPE_LEVEL_LOW;
        break;
    default:
        return -EINVAL;
    }

    putreg32(v | (tsb_trigger << shift), reg);
    return 0;
}

static int tsb_gpio_irq_handler(int irq, void *context)
{
    /*
     * Handle each pending GPIO interrupt.  "The GPIO MIS register is the masked
     * interrupt status register. Bits read High in GPIO MIS reflect the status
     * of input lines triggering an interrupt. Bits read as Low indicate that
     * either no interrupt has been generated, or the interrupt is masked."
     */
    uint32_t irqstat;
    uint8_t base;
    int pin;

    /*
     * Clear all GPIO interrupts that we are going to process. "The GPIO_RAWINTSTAT
     * register is the interrupt clear register. Writing a 1 to a bit in this
     * register clears the corresponding interrupt edge detection logic register.
     * Writing a 0 has no effect."
     */
    irqstat = tsb_gpio_get_interrupt();
    putreg32(irqstat, GPIO_RAWINTSTAT);

    /* Now process each IRQ pending in the GPIO */
    for (pin = 0; pin < NR_GPIO_IRQS && irqstat != 0; pin++, irqstat >>= 1) {
        if ((irqstat & 1) != 0) {
            base = tsb_gpio_irq_gpio_base[pin];
            tsb_gpio_irq_vector[pin](base + pin, context);
        }
    }

    return 0;
}

int tsb_gpio_irqattach(uint8_t irq, xcpt_t isr, uint8_t base)
{
    irqstate_t flags;

    if (irq >= NR_GPIO_IRQS)
        return -EINVAL;

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
    tsb_gpio_irq_vector[irq] = isr;
    tsb_gpio_irq_gpio_base[irq] = base;
    irqrestore(flags);

    return OK;
}

static void tsb_gpio_irqinitialize(void)
{
    int i;

    /* Point all interrupt vectors to the unexpected interrupt */
    for (i = 0; i < NR_GPIO_IRQS; i++) {
        tsb_gpio_irq_vector[i] = irq_unexpected_isr;
    }
}

void tsb_gpio_initialize(void)
{
    irqstate_t flags;

    flags = irqsave();
    refcount++;

    tsb_clk_enable(TSB_CLK_GPIO);
    tsb_reset(TSB_RST_GPIO);

    tsb_gpio_irqinitialize();

    /* Attach Interrupt Handler */
    irq_attach(TSB_IRQ_GPIO, tsb_gpio_irq_handler);

    /* Enable Interrupt Handler */
    up_enable_irq(TSB_IRQ_GPIO);

    irqrestore(flags);
}

void tsb_gpio_uninitialize(void)
{
    irqstate_t flags;

    flags = irqsave();
    refcount--;
    if (refcount > 0) {
        irqrestore(flags);
        return;
    }

    tsb_clk_disable(TSB_CLK_GPIO);

    /* Detach Interrupt Handler */
    irq_detach(TSB_IRQ_GPIO);

    irqrestore(flags);
}

struct gpio_ops_s tsb_gpio_ops = {
    .get_direction = tsb_gpio_get_direction,
    .direction_in = tsb_gpio_direction_in,
    .direction_out = tsb_gpio_direction_out,
    .activate = tsb_gpio_activate,
    .get_value = tsb_gpio_get_value,
    .set_value = tsb_gpio_set_value,
    .deactivate = tsb_gpio_deactivate,
    .line_count = tsb_gpio_line_count,
    .irqattach = tsb_gpio_irqattach,
    .set_triggering = tsb_set_gpio_triggering,
    .mask_irq = tsb_gpio_mask_irq,
    .unmask_irq = tsb_gpio_unmask_irq,
    .clear_interrupt = tsb_gpio_clear_interrupt,
};

int tsb_gpio_register(void)
{
    return register_gpio_chip(&tsb_gpio_ops, 0);
}
