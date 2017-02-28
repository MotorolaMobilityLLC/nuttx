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

#include <errno.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>

#include <nuttx/gpio.h>
#include <nuttx/list.h>

#include "nuttx/arch.h"

uint8_t g_gpio_line_count = 0;
LIST_DECLARE(g_gpio_chip);

/**
 * @brief Register a driver to the gpio_chip framework
 *
 * driver_data is a unique handle passed to register and unregister the
 * driver
 */
int register_gpio_chip(struct gpio_ops_s *ops, int base, void *driver_data)
{
    struct list_head *iter;
    struct gpio_chip_s *chip;

    DEBUGASSERT(ops);
    DEBUGASSERT(ops->line_count);

    if (base == -1) {
        list_foreach(&g_gpio_chip, iter) {
            chip = list_entry(iter, struct gpio_chip_s, list);
            base = chip->end;
        }
    }

    chip = malloc(sizeof(*chip));
    if (!chip)
        return -ENOMEM;

    chip->base = base;
    chip->end = base + ops->line_count(driver_data);
    chip->ops = ops;
    chip->driver_data = driver_data;

    g_gpio_line_count += ops->line_count(driver_data);

    list_add(&g_gpio_chip, &chip->list);

    return 0;
}

int unregister_gpio_chip(void *driver_data)
{
    struct list_head *iter, *iter_next;
    struct gpio_chip_s *chip;

    list_foreach_safe(&g_gpio_chip, iter, iter_next) {
        chip = list_entry(iter, struct gpio_chip_s, list);
        if (chip->driver_data == driver_data) {
            g_gpio_line_count -= chip->ops->line_count(driver_data);
            list_del(iter);
            free(chip);
        }
    }

    return 0;
}

static struct gpio_chip_s *get_gpio_chip(uint8_t *which)
{
    struct list_head *iter;
    struct gpio_chip_s *chip;

    list_foreach(&g_gpio_chip, iter) {
        chip = list_entry(iter, struct gpio_chip_s, list);
        if (chip->base <= *which && *which < chip->end) {
            *which -= chip->base;
            return chip;
        }
    }
    return NULL;
}

int gpio_get_direction(uint8_t which)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);
    return chip->ops->get_direction(chip->driver_data, which);
}

void gpio_direction_in(uint8_t which)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    DEBUGASSERT(chip->ops->direction_in);
    chip->ops->direction_in(chip->driver_data, which);
}

void gpio_direction_out(uint8_t which, uint8_t value)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    DEBUGASSERT(chip->ops->direction_out);
    chip->ops->direction_out(chip->driver_data, which, value);
}

void gpio_activate(uint8_t which)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    DEBUGASSERT(chip->ops->activate);
    chip->ops->activate(chip->driver_data, which);
}

uint8_t gpio_get_value(uint8_t which)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    DEBUGASSERT(chip->ops->get_value);
    return chip->ops->get_value(chip->driver_data, which);
}

void gpio_set_value(uint8_t which, uint8_t value)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    DEBUGASSERT(chip->ops->set_value);
    chip->ops->set_value(chip->driver_data, which, value);
}

int gpio_set_debounce(uint8_t which, uint16_t delay)
{
    printf("%s - unimplemented\n", __func__);

    return -ENOSYS;
}

void gpio_deactivate(uint8_t which)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    DEBUGASSERT(chip->ops->deactivate);
    chip->ops->deactivate(chip->driver_data, which);
}

uint8_t gpio_line_count(void)
{
    return g_gpio_line_count;
}

int gpio_irqattach(uint8_t which, xcpt_t isr)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    if (chip->ops->irqattach)
        return chip->ops->irqattach(chip->driver_data, which, isr,
                                    chip->base);
    return -EINVAL;
}

int gpio_irqattach_old(uint8_t which, xcpt_t isr, xcpt_t *old)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    if (chip->ops->irqattach_old)
        return chip->ops->irqattach_old(chip->driver_data, which, isr,
                                        chip->base, old);
    return -EINVAL;
}

int set_gpio_triggering(uint8_t which, int trigger)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    if (chip->ops->set_triggering)
        return chip->ops->set_triggering(chip->driver_data, which, trigger);
    return -EINVAL;
}

int gpio_mask_irq(uint8_t which)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    if (chip->ops->mask_irq)
        return chip->ops->mask_irq(chip->driver_data, which);
    return -EINVAL;
}

int gpio_unmask_irq(uint8_t which)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    if (chip->ops->unmask_irq)
        return chip->ops->unmask_irq(chip->driver_data, which);
    return -EINVAL;
}

int gpio_clear_interrupt(uint8_t which)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    if (chip->ops->clear_interrupt)
        return chip->ops->clear_interrupt(chip->driver_data, which);
    return -EINVAL;
}

gpio_cfg_t gpio_cfg_save(uint8_t which)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    DEBUGASSERT(chip->ops->cfg_save);
    return chip->ops->cfg_save(chip->driver_data, which);
}

void gpio_cfg_restore(uint8_t which, gpio_cfg_t cfg)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    DEBUGASSERT(chip->ops->cfg_restore);
    chip->ops->cfg_restore(chip->driver_data, which, cfg);
}

void gpio_cfg_set(uint8_t which, gpio_cfg_t cfg)
{
    struct gpio_chip_s *chip = get_gpio_chip(&which);

    DEBUGASSERT(chip);
    DEBUGASSERT(chip->ops->cfg_set);
    chip->ops->cfg_set(chip->driver_data, which, cfg);
}
