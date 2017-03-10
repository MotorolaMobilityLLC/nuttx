/*
 * Copyright (c) 2017 Motorola Mobility, LLC.
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
#include <debug.h>
#include <unistd.h>

#include <nuttx/util.h>

#include "stm32_adc.h"

/* ADC1 */
#define ADC1_INTERFACE (1)
#define ADC1_DEVPATH "/dev/adc0"

static const uint32_t ADC1_PINS[]  = {
#if CONFIG_STM32_ADC1_CHAN0_INIT
    GPIO_ADC1_IN0,
#endif
#if CONFIG_STM32_ADC1_CHAN1_INIT
    GPIO_ADC1_IN1,
#endif
#if CONFIG_STM32_ADC1_CHAN2_INIT
    GPIO_ADC1_IN2,
#endif
#if CONFIG_STM32_ADC1_CHAN3_INIT
    GPIO_ADC1_IN3,
#endif
#if CONFIG_STM32_ADC1_CHAN4_INIT
    GPIO_ADC1_IN4,
#endif
#if CONFIG_STM32_ADC1_CHAN5_INIT
    GPIO_ADC1_IN5,
#endif
#if CONFIG_STM32_ADC1_CHAN6_INIT
    GPIO_ADC1_IN6,
#endif
#if CONFIG_STM32_ADC1_CHAN7_INIT
    GPIO_ADC1_IN7,
#endif
#if CONFIG_STM32_ADC1_CHAN8_INIT
    GPIO_ADC1_IN8,
#endif
#if CONFIG_STM32_ADC1_CHAN9_INIT
    GPIO_ADC1_IN9,
#endif
#if CONFIG_STM32_ADC1_CHAN10_INIT
    GPIO_ADC1_IN10,
#endif
#if CONFIG_STM32_ADC1_CHAN11_INIT
    GPIO_ADC1_IN11,
#endif
#if CONFIG_STM32_ADC1_CHAN12_INIT
    GPIO_ADC1_IN12,
#endif
#if CONFIG_STM32_ADC1_CHAN13_INIT
    GPIO_ADC1_IN13,
#endif
#if CONFIG_STM32_ADC1_CHAN14_INIT
    GPIO_ADC1_IN14,
#endif
#if CONFIG_STM32_ADC1_CHAN15_INIT
    GPIO_ADC1_IN15,
#endif
};

static const uint8_t ADC1_CHANNELS[] = {
#if CONFIG_STM32_ADC1_CHAN0_INIT
    0,
#endif
#if CONFIG_STM32_ADC1_CHAN1_INIT
    1,
#endif
#if CONFIG_STM32_ADC1_CHAN2_INIT
    2,
#endif
#if CONFIG_STM32_ADC1_CHAN3_INIT
    3,
#endif
#if CONFIG_STM32_ADC1_CHAN4_INIT
    4,
#endif
#if CONFIG_STM32_ADC1_CHAN5_INIT
    5,
#endif
#if CONFIG_STM32_ADC1_CHAN6_INIT
    6,
#endif
#if CONFIG_STM32_ADC1_CHAN7_INIT
    7,
#endif
#if CONFIG_STM32_ADC1_CHAN8_INIT
    8,
#endif
#if CONFIG_STM32_ADC1_CHAN9_INIT
    9,
#endif
#if CONFIG_STM32_ADC1_CHAN10_INIT
    10,
#endif
#if CONFIG_STM32_ADC1_CHAN11_INIT
    11,
#endif
#if CONFIG_STM32_ADC1_CHAN12_INIT
    12,
#endif
#if CONFIG_STM32_ADC1_CHAN13_INIT
    13,
#endif
#if CONFIG_STM32_ADC1_CHAN14_INIT
    14,
#endif
#if CONFIG_STM32_ADC1_CHAN15_INIT
    15,
#endif
};

/* ADC2 */
#define ADC2_INTERFACE (2)
#define ADC2_DEVPATH "/dev/adc1"

static const uint8_t ADC2_CHANNELS[] = {
#if CONFIG_STM32_ADC2_CHAN0_INIT
    0,
#endif
#if CONFIG_STM32_ADC2_CHAN1_INIT
    1,
#endif
#if CONFIG_STM32_ADC2_CHAN2_INIT
    2,
#endif
#if CONFIG_STM32_ADC2_CHAN3_INIT
    3,
#endif
#if CONFIG_STM32_ADC2_CHAN4_INIT
    4,
#endif
#if CONFIG_STM32_ADC2_CHAN5_INIT
    5,
#endif
#if CONFIG_STM32_ADC2_CHAN6_INIT
    6,
#endif
#if CONFIG_STM32_ADC2_CHAN7_INIT
    7,
#endif
#if CONFIG_STM32_ADC2_CHAN8_INIT
    8,
#endif
#if CONFIG_STM32_ADC2_CHAN9_INIT
    9,
#endif
#if CONFIG_STM32_ADC2_CHAN10_INIT
    10,
#endif
#if CONFIG_STM32_ADC2_CHAN11_INIT
    11,
#endif
#if CONFIG_STM32_ADC2_CHAN12_INIT
    12,
#endif
#if CONFIG_STM32_ADC2_CHAN13_INIT
    13,
#endif
#if CONFIG_STM32_ADC2_CHAN14_INIT
    14,
#endif
#if CONFIG_STM32_ADC2_CHAN15_INIT
    15,
#endif
};

/* ADC3 */
#define ADC3_INTERFACE (3)
#define ADC3_DEVPATH "/dev/adc2"

static const uint8_t ADC3_CHANNELS[] = {
#if CONFIG_STM32_ADC3_CHAN0_INIT
    0,
#endif
#if CONFIG_STM32_ADC3_CHAN1_INIT
    1,
#endif
#if CONFIG_STM32_ADC3_CHAN2_INIT
    2,
#endif
#if CONFIG_STM32_ADC3_CHAN3_INIT
    3,
#endif
#if CONFIG_STM32_ADC3_CHAN4_INIT
    4,
#endif
#if CONFIG_STM32_ADC3_CHAN5_INIT
    5,
#endif
#if CONFIG_STM32_ADC3_CHAN6_INIT
    6,
#endif
#if CONFIG_STM32_ADC3_CHAN7_INIT
    7,
#endif
#if CONFIG_STM32_ADC3_CHAN8_INIT
    8,
#endif
#if CONFIG_STM32_ADC3_CHAN9_INIT
    9,
#endif
#if CONFIG_STM32_ADC3_CHAN10_INIT
    10,
#endif
#if CONFIG_STM32_ADC3_CHAN11_INIT
    11,
#endif
#if CONFIG_STM32_ADC3_CHAN12_INIT
    12,
#endif
#if CONFIG_STM32_ADC3_CHAN13_INIT
    13,
#endif
#if CONFIG_STM32_ADC3_CHAN14_INIT
    14,
#endif
#if CONFIG_STM32_ADC3_CHAN15_INIT
    15,
#endif
};

/* ADC4 */
#define ADC4_INTERFACE (4)
#define ADC4_DEVPATH "/dev/adc3"

static const uint8_t ADC4_CHANNELS[] = {
#if CONFIG_STM32_ADC4_CHAN0_INIT
    0,
#endif
#if CONFIG_STM32_ADC4_CHAN1_INIT
    1,
#endif
#if CONFIG_STM32_ADC4_CHAN2_INIT
    2,
#endif
#if CONFIG_STM32_ADC4_CHAN3_INIT
    3,
#endif
#if CONFIG_STM32_ADC4_CHAN4_INIT
    4,
#endif
#if CONFIG_STM32_ADC4_CHAN5_INIT
    5,
#endif
#if CONFIG_STM32_ADC4_CHAN6_INIT
    6,
#endif
#if CONFIG_STM32_ADC4_CHAN7_INIT
    7,
#endif
#if CONFIG_STM32_ADC4_CHAN8_INIT
    8,
#endif
#if CONFIG_STM32_ADC4_CHAN9_INIT
    9,
#endif
#if CONFIG_STM32_ADC4_CHAN10_INIT
    10,
#endif
#if CONFIG_STM32_ADC4_CHAN11_INIT
    11,
#endif
#if CONFIG_STM32_ADC4_CHAN12_INIT
    12,
#endif
#if CONFIG_STM32_ADC4_CHAN13_INIT
    13,
#endif
#if CONFIG_STM32_ADC4_CHAN14_INIT
    14,
#endif
#if CONFIG_STM32_ADC4_CHAN15_INIT
    15,
#endif
};

static int stm32_adc_init_pins(const uint32_t *pins, size_t count)
{
    size_t i;

    /* Configure the pins as analog inputs for the selected channels */
    for (i = 0; i < count; i++) {
        stm32_configgpio(pins[i]);
    }

    return OK;
}

static int stm32_adc_init_channel(int interface,
                                  const uint8_t *channels, size_t count)
{
    struct adc_dev_s *adc;
    int ret;

    /* Call stm32_adcinitialize() to get an instance of the ADC interface */
    adc = stm32_adcinitialize(interface, channels, count);
    if (!adc) {
        dbg("ERROR: Failed to get ADC interface: errno=%d\n", errno);
        return -ENODEV;
    }

    /* Register the ADC driver */
    ret = adc_register(ADC1_DEVPATH, adc);
    if (ret < 0)
    {
        dbg("ERROR: Failed to register ADC device: devpath=%s ret=%d\n",
            ADC1_DEVPATH, ret);
    }

    return ret;
}

int stm32_adc_initialize(void)
{
#if CONFIG_STM32_ADC1
    stm32_adc_init_pins(ADC1_PINS, ARRAY_SIZE(ADC1_PINS));
    stm32_adc_init_channel(ADC1_INTERFACE, ADC1_CHANNELS, ARRAY_SIZE(ADC1_CHANNELS));
#endif
#if CONFIG_STM32_ADC2
    stm32_adc_init_pins(ADC1_PINS, ARRAY_SIZE(ADC2_PINS));
    stm32_adc_init_channel(ADC2_INTERFACE, ADC2_CHANNELS, ARRAY_SIZE(ADC2_CHANNELS));
#endif
#if CONFIG_STM32_ADC3
    stm32_adc_init_pins(ADC3_PINS, ARRAY_SIZE(ADC3_PINS));
    stm32_adc_init_channel(ADC3_INTERFACE, ADC3_CHANNELS, ARRAY_SIZE(ADC3_CHANNELS));
#endif
#if CONFIG_STM32_ADC4
    stm32_adc_init_pins(ADC4_PINS, ARRAY_SIZE(ADC4_PINS));
    stm32_adc_init_channel(ADC4_INTERFACE, ADC4_CHANNELS, ARRAY_SIZE(ADC4_CHANNELS));
#endif

    return OK;
}
