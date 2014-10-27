/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 * Authors: Fabien Parent <fparent@baylibre.com>
 *          Benoit Cousson <bcousson@baylibre.com>
 */

#ifndef _TSB_GPIO_H_
#define _TSB_GPIO_H_

#include <stdint.h>
#include <arch/irq.h>

int gpio_get_direction(uint8_t which);
void gpio_direction_in(uint8_t which);
void gpio_direction_out(uint8_t which, uint8_t value);
void gpio_activate(uint8_t which);
uint8_t gpio_get_value(uint8_t which);
void gpio_set_value(uint8_t which, uint8_t value);
int gpio_set_debounce(uint8_t which, uint16_t delay);
void gpio_deactivate(uint8_t which);
uint8_t gpio_line_count(void);
void gpio_initialize(void);
void gpio_uninitialize(void);
int gpio_irqattach(int irq, xcpt_t isr);
void set_gpio_triggering(uint8_t which, int trigger);
void gpio_mask_irq(uint8_t which);
void gpio_unmask_irq(uint8_t which);
void gpio_clear_interrupt(uint8_t which);
uint32_t gpio_get_raw_interrupt(uint8_t which);
uint32_t gpio_get_interrupt(void);

#define IRQ_TYPE_LEVEL_LOW      0x0
#define IRQ_TYPE_LEVEL_HIGH     0x1
#define IRQ_TYPE_EDGE_FALLING   0x2
#define IRQ_TYPE_EDGE_RISING    0x3
#define IRQ_TYPE_EDGE_BOTH      0x7

#endif /* _TSB_GPIO_H_ */

