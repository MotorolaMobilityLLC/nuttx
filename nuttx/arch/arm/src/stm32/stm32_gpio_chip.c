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
 * @brief STM32 GPIO Chip Driver
 *
 * STM32 has 16 GPIO pins per port. Pin A0 is pin 0, pin B0 is pin 16 etc.
 * @author Jean Pihet
 */

#include <nuttx/config.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <nuttx/gpio/stm32_gpio_chip.h>
#include <nuttx/gpio.h>

#include "stm32_gpio.h"

/* disable the verbose debug output */
#undef lldbg
#define lldbg(x...)


// Map pin number to cfgset used by the STM32 GPIO framework
static int map_pin_nr_to_cfgset(uint8_t pin, uint32_t *cfgset)
{
    if (pin > STM32_NGPIO)
        return -EINVAL;

    // 16 pins per port
    *cfgset = ((pin / 16) << GPIO_PORT_SHIFT) |
              ((pin % 16) << GPIO_PIN_SHIFT);

    return 0;
}

void stm32_gpio_set_direction_in(void *driver_data, uint8_t pin)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", pin);
        return;
    }

    // Configure pin as input, floating, weak PU
    cfgset |= GPIO_INPUT | GPIO_FLOAT | GPIO_PULLUP;
    lldbg("cfgset=0x%x\n", cfgset);

    ret = stm32_configgpio(cfgset);
    if (ret)
        lldbg("%s: stm32_configgpio returns %d\n", ret);
}

void stm32_gpio_set_direction_out(void *driver_data, uint8_t pin, uint8_t value)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", pin);
        return;
    }

    // Configure pin as output
    if (value)
        cfgset |= GPIO_OUTPUT | GPIO_OUTPUT_SET;
    else
        cfgset |= GPIO_OUTPUT | GPIO_OUTPUT_CLEAR;

    lldbg("cfgset=0x%x\n", cfgset);

    ret = stm32_configgpio(cfgset);
    if (ret)
        lldbg("%s: stm32_configgpio returns %d\n", ret);
}

// STM32 GPIO API does not have a direction query function
int stm32_gpio_get_direction(void *driver_data, uint8_t pin)
{
    return -EOPNOTSUPP;
}

void stm32_gpio_set(void *driver_data, uint8_t pin, uint8_t val)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu, val=%d\n", __func__, pin, val);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", pin);
        return;
    }

    stm32_gpiowrite(cfgset, val);
}

uint8_t stm32_gpio_get(void *driver_data, uint8_t pin)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", pin);
        return -EINVAL;
    }

    return stm32_gpioread(cfgset);
}

uint8_t stm32_gpio_line_count(void *driver_data)
{
    return STM32_NGPIO + 1;
}

void stm32_gpio_activate(void *driver_data, uint8_t pin)
{

}

// Configure pin as input floating
void stm32_gpio_deactivate(void *driver_data, uint8_t pin)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", pin);
        return;
    }

    stm32_unconfiggpio(cfgset);
}

struct gpio_ops_s stm32_gpio_ops = {
    .direction_in =     stm32_gpio_set_direction_in,
    .direction_out =    stm32_gpio_set_direction_out,
    .get_direction =    stm32_gpio_get_direction,
    .activate =         stm32_gpio_activate,
    .get_value =        stm32_gpio_get,
    .set_value =        stm32_gpio_set,
    .deactivate =       stm32_gpio_deactivate,
    .line_count =       stm32_gpio_line_count,
};

void stm32_gpio_init(void)
{
    register_gpio_chip(&stm32_gpio_ops, STM32_GPIO_CHIP_BASE, &stm32_gpio_ops);
}

void stm32_gpio_deinit(void)
{
    unregister_gpio_chip(&stm32_gpio_ops);
}
