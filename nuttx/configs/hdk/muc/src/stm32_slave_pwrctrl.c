/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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
#include <unistd.h>

#include <nuttx/device.h>
#include <nuttx/device_slave_pwrctrl.h>

#include <nuttx/greybus/mods-ctrl.h>

#include <arch/board/board.h>

#define GPIO_APBE_SPIBOOT_N       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                                   GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

#define GPIO_APBE_WAKE            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                                   GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)

#define GPIO_APBE_RST_N           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                                   GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN1)

#define GPIO_APBE_PWR_EN          (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                                   GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN14)

#define APBE_PWR_EN_DELAY (100 * 1000)
#define APBE_RST_N_DELAY (100 * 1000)

static void apbe_power_off(void)
{
  lldbg("\n");
  stm32_gpiowrite(GPIO_APBE_SPIBOOT_N, 0);
  stm32_gpiowrite(GPIO_APBE_WAKE, 0);
  stm32_gpiowrite(GPIO_APBE_PWR_EN, 0);
  stm32_gpiowrite(GPIO_APBE_RST_N, 0);
}

static void apbe_power_on_normal(void)
{
  lldbg("\n");
  stm32_gpiowrite(GPIO_APBE_SPIBOOT_N, 0);
  stm32_gpiowrite(GPIO_APBE_WAKE, 0);
  stm32_gpiowrite(GPIO_APBE_PWR_EN, 1);
  usleep(APBE_PWR_EN_DELAY);
  stm32_gpiowrite(GPIO_APBE_RST_N, 1);
  usleep(APBE_RST_N_DELAY);
}

static void apbe_power_on_flash(void)
{
  lldbg("\n");
  stm32_gpiowrite(GPIO_APBE_SPIBOOT_N, 0);
  stm32_gpiowrite(GPIO_APBE_WAKE, 1);
  stm32_gpiowrite(GPIO_APBE_PWR_EN, 1);
  usleep(APBE_PWR_EN_DELAY);
  stm32_gpiowrite(GPIO_APBE_RST_N, 1);
  usleep(APBE_RST_N_DELAY);
}

static int slave_pwrctrl_op_get_mask(struct device *dev, uint32_t *mask)
{
    if (mask)
        *mask = MB_CONTROL_SLAVE_MASK_APBE;

    lldbg("mask=0x%x\n", (mask ? *mask : 0));
    return OK;
}

static int slave_pwrctrl_op_set_mode(struct device *dev, enum slave_pwrctrl_mode mode)
{
    lldbg("mode=%d\n", mode);
    switch (mode) {
    case SLAVE_PWRCTRL_POWER_ON:
#if CONFIG_SLAVE_PWRCTRL_FORCE_FLASH
        apbe_power_on_flash();
#else
        apbe_power_on_normal();
#endif
        break;
    case SLAVE_PWRCTRL_POWER_FLASH_MODE:
        apbe_power_on_flash();
        break;
    case SLAVE_PWRCTRL_POWER_OFF:
    default:
        apbe_power_off();
        break;
    }

    return OK;
}

static int slave_pwrctrl_probe(struct device *dev)
{
    lldbg("\n");
    stm32_configgpio(GPIO_APBE_SPIBOOT_N);
    stm32_configgpio(GPIO_APBE_WAKE);
    stm32_configgpio(GPIO_APBE_PWR_EN);
    stm32_configgpio(GPIO_APBE_RST_N);
    return OK;
}

static void slave_pwrctrl_remove(struct device *dev)
{
    lldbg("\n");
    apbe_power_off();
}

static struct device_slave_pwrctrl_type_ops slave_pwrctrl_type_ops = {
    .get_mask = slave_pwrctrl_op_get_mask,
    .set_mode = slave_pwrctrl_op_set_mode,
};

static struct device_driver_ops slave_pwrctrl_driver_ops = {
    .probe    = &slave_pwrctrl_probe,
    .remove   = &slave_pwrctrl_remove,
    .type_ops = &slave_pwrctrl_type_ops,
};

const struct device_driver slave_pwrctrl_driver = {
    .type   = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
    .name   = "slave_pwrctrl",
    .desc   = "slave power control",
    .ops    = &slave_pwrctrl_driver_ops,
};
