/*
 * Copyright (C) 2016 Motorola Mobility, LLC.
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
 */

#ifndef _GPIO_CHIP_H_
#define _GPIO_CHIP_H_

#include <nuttx/irq.h>
#include <nuttx/list.h>

#define IRQ_TYPE_NONE           0x00000000
#define IRQ_TYPE_EDGE_RISING    0x00000001
#define IRQ_TYPE_EDGE_FALLING   0x00000002
#define IRQ_TYPE_EDGE_BOTH      (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
#define IRQ_TYPE_LEVEL_HIGH     0x00000004
#define IRQ_TYPE_LEVEL_LOW      0x00000008

typedef void *gpio_cfg_t;

struct gpio_ops_s
{
    int (*get_direction)(void *driver_data, uint8_t which);
    void (*direction_in)(void *driver_data, uint8_t which);
    void (*direction_out)(void *driver_data, uint8_t which, uint8_t value);
    void (*activate)(void *driver_data, uint8_t which);
    uint8_t (*get_value)(void *driver_data, uint8_t which);
    void (*set_value)(void *driver_data, uint8_t which, uint8_t value);
    void (*deactivate)(void *driver_data, uint8_t which);
    uint8_t (*line_count)(void *driver_data);
    int (*irqattach)(void *driver_data, uint8_t which, xcpt_t isr,
                     uint8_t base);
    int (*irqattach_old)(void *driver_data, uint8_t which, xcpt_t isr,
                         uint8_t base, xcpt_t *old);
    int (*set_triggering)(void *driver_data, uint8_t which, int trigger);
    int (*mask_irq)(void *driver_data, uint8_t which);
    int (*unmask_irq)(void *driver_data, uint8_t which);
    int (*clear_interrupt)(void *driver_data, uint8_t which);
    gpio_cfg_t (*cfg_save)(void *driver_data, uint8_t which);
    void (*cfg_restore)(void *driver_data, uint8_t which, gpio_cfg_t cfg);
    void (*cfg_set)(void *driver_data, uint8_t which, gpio_cfg_t cfg);
};

struct gpio_chip_s
{
    struct list_head list;
    struct gpio_ops_s *ops;
    void *driver_data;
    uint8_t base;
    uint8_t end;
};

int gpio_get_direction(uint8_t which);
void gpio_direction_in(uint8_t which);
void gpio_direction_out(uint8_t which, uint8_t value);
void gpio_activate(uint8_t which);
uint8_t gpio_get_value(uint8_t which);
void gpio_set_value(uint8_t which, uint8_t value);
int gpio_set_debounce(uint8_t which, uint16_t delay);
void gpio_deactivate(uint8_t which);
uint8_t gpio_line_count(void);
int gpio_irqattach(uint8_t which, xcpt_t isr);
int gpio_irqattach_old(uint8_t which, xcpt_t isr, xcpt_t *old);
int set_gpio_triggering(uint8_t which, int trigger);
int gpio_mask_irq(uint8_t which);
int gpio_unmask_irq(uint8_t which);
int gpio_clear_interrupt(uint8_t which);
gpio_cfg_t gpio_cfg_save(uint8_t which);
void gpio_cfg_restore(uint8_t which, gpio_cfg_t cfg);
void gpio_cfg_set(uint8_t which, gpio_cfg_t cfg);

int register_gpio_chip(struct gpio_ops_s *ops, int base, void *driver_data);
int unregister_gpio_chip(void *driver_data);

#endif
