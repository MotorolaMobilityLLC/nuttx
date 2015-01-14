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

