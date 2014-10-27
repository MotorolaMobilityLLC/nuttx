/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 * Authors: Fabien Parent <fparent@baylibre.com>
 *          Benoit Cousson <bcousson@baylibre.com>
 */

#include <arch/tsb/gpio.h>

#include <stdint.h>
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

#define NR_GPIO_IRQS 16

/* A table of handlers for each GPIO interrupt */
static xcpt_t gpio_irq_vector[NR_GPIO_IRQS];
static volatile uint32_t refcount;

int gpio_get_direction(uint8_t which)
{
    uint32_t dir = getreg32(GPIO_DIR);
    return !(dir & (1 << which));
}

void gpio_direction_in(uint8_t which)
{
    putreg32(1 << which, GPIO_DIRIN);
}

void gpio_direction_out(uint8_t which, uint8_t value)
{
    gpio_set_value(which, value);
    putreg32(1 << which, GPIO_DIROUT);
}

void gpio_activate(uint8_t which)
{
    gpio_initialize();
}

uint8_t gpio_get_value(uint8_t which)
{
    return !!(getreg32(GPIO_DATA) & (1 << which));
}

void gpio_set_value(uint8_t which, uint8_t value)
{
    putreg32(1 << which, value ? GPIO_ODATASET : GPIO_ODATACLR);
}

int gpio_set_debounce(uint8_t which, uint16_t delay)
{
    uint8_t initial_reading;

    initial_reading = gpio_get_value(which);
    up_udelay(delay);
    return initial_reading == gpio_get_value(which);
}

void gpio_deactivate(uint8_t which)
{
    gpio_uninitialize();
}

uint8_t gpio_line_count(void)
{
    return NR_GPIO_IRQS;
}

void gpio_mask_irq(uint8_t which)
{
    putreg32(1 << which, GPIO_INTMASKSET);
}

void gpio_unmask_irq(uint8_t which)
{
    putreg32(1 << which, GPIO_INTMASKCLR);
}

void gpio_clear_interrupt(uint8_t which)
{
    putreg32(1 << which, GPIO_RAWINTSTAT);
}

uint32_t gpio_get_raw_interrupt(uint8_t which)
{
    return getreg32(GPIO_RAWINTSTAT);
}

uint32_t gpio_get_interrupt(void)
{
    return getreg32(GPIO_INTSTAT);
}

void set_gpio_triggering(uint8_t which, int trigger)
{
    uint32_t reg = GPIO_INTCTRL0 + ((which >> 1) & 0xfc);
    uint32_t shift = 4 * (which & 0x7);
    uint32_t v = getreg32(reg);

    putreg32(v | (trigger << shift), reg);
}

static int gpio_irq_handler(int irq, void *context)
{
    /*
     * Handle each pending GPIO interrupt.  "The GPIO MIS register is the masked
     * interrupt status register. Bits read High in GPIO MIS reflect the status
     * of input lines triggering an interrupt. Bits read as Low indicate that
     * either no interrupt has been generated, or the interrupt is masked."
     */
    uint32_t irqstat = gpio_get_interrupt();
    int pin;

    /*
     * Clear all GPIO interrupts that we are going to process. "The GPIO_RAWINTSTAT
     * register is the interrupt clear register. Writing a 1 to a bit in this
     * register clears the corresponding interrupt edge detection logic register.
     * Writing a 0 has no effect."
     */
    putreg32(irqstat, GPIO_RAWINTSTAT);

    /* Now process each IRQ pending in the GPIO */
    for (pin = 0; pin < NR_GPIO_IRQS && irqstat != 0; pin++, irqstat >>= 1) {
        if ((irqstat & 1) != 0) {
            gpio_irq_vector[pin](pin, context);
        }
    }

    return 0;
}

int gpio_irqattach(int irq, xcpt_t isr)
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
    gpio_irq_vector[irq] = isr;
    irqrestore(flags);

    return OK;
}

static void gpio_irqinitialize(void)
{
    int i;

    /* Point all interrupt vectors to the unexpected interrupt */
    for (i = 0; i < NR_GPIO_IRQS; i++) {
        gpio_irq_vector[i] = irq_unexpected_isr;
    }
}

void gpio_initialize(void)
{
    irqstate_t flags;

    flags = irqsave();
    refcount++;

    tsb_clk_enable(TSB_CLK_GPIO);
    tsb_reset(TSB_RST_GPIO);

    gpio_irqinitialize();

    /* Attach Interrupt Handler */
    irq_attach(TSB_IRQ_GPIO, gpio_irq_handler);

    /* Enable Interrupt Handler */
    up_enable_irq(TSB_IRQ_GPIO);

    irqrestore(flags);
}

void gpio_uninitialize(void)
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
