/*
 * arch/arm/src/stm32/stm32_adc.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Diego Sanchez <dsanchez@nx-engineering.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *   Copyright (c) 2015 Google Inc.
 *   All rights reserved.
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

/**
 * @file    configs/ara/svc/src/up_adc.c
 * @brief   SVC STM32 integrated ADC (Analog to Digital Converter) support.
 *          Based on Nuttx STM32 ADC driver.
 * @author  Patrick Titiano, Gregory Nutt, Diego Sanchez
 */

#define DBG_COMP ARADBG_POWER

#include <nuttx/config.h>
#include <errno.h>
#include <ara_debug.h>
#include <arch/board/board.h>
#include <up_adc.h>
#include <stm32_adc.h>
#include <chip.h>
#include <up_arch.h>

#define CHECK_NULL_ARG(arg) \
    if ((void *) arg == NULL) { \
        dbg_error("%s(): %s == NULL!!!\n", __func__, #arg); \
        return -EINVAL; \
    }

#define CHECK_ARG_ADC_DEV(adc) \
    if ((adc == 0) || (adc > ADC_COUNT)) { \
        dbg_error("%s(): invalid ADC instance!!! (%u) !!!\n", \
            __func__, #adc); \
        return -EINVAL; \
    }

#define CHECK_ARG_ADC_CHANNEL(channel) \
    if (channel >= ADC_NCHANNEL) { \
        dbg_error("%s(): invalid ADC channel!!! (%u) !!!\n", \
            __func__, #channel); \
        return -EINVAL; \
    }

#define CHECK_ARG_AVG_COUNT(count) \
    if ((count == 0) || (count > ADC_MAX_AVG_SAMPLE_COUNT)) { \
        dbg_error("%s(): invalid AVG count!!! (%u) !!!\n", \
            __func__, #count); \
        return -EINVAL; \
    }

/* Store GPIO bit-encoded description for each channel for each ADC instance */
static const uint32_t adc_gpio[ADC_NCHANNEL][ADC_COUNT] = {
    {GPIO_ADC1_IN0, GPIO_ADC2_IN0, GPIO_ADC3_IN0},
    {GPIO_ADC1_IN1, GPIO_ADC2_IN1, GPIO_ADC3_IN1},
    {GPIO_ADC1_IN2, GPIO_ADC2_IN2, GPIO_ADC3_IN2},
    {GPIO_ADC1_IN3, GPIO_ADC2_IN3, GPIO_ADC3_IN3},
    {GPIO_ADC1_IN4, GPIO_ADC2_IN4, GPIO_ADC3_IN4},
    {GPIO_ADC1_IN5, GPIO_ADC2_IN5, GPIO_ADC3_IN5},
    {GPIO_ADC1_IN6, GPIO_ADC2_IN6, GPIO_ADC3_IN6},
    {GPIO_ADC1_IN7, GPIO_ADC2_IN7, GPIO_ADC3_IN7},
    {GPIO_ADC1_IN8, GPIO_ADC2_IN8, GPIO_ADC3_IN8},
    {GPIO_ADC1_IN9, GPIO_ADC2_IN9, GPIO_ADC3_IN9},
    {GPIO_ADC1_IN10, GPIO_ADC2_IN10, GPIO_ADC3_IN10},
    {GPIO_ADC1_IN11, GPIO_ADC2_IN11, GPIO_ADC3_IN11},
    {GPIO_ADC1_IN12, GPIO_ADC2_IN12, GPIO_ADC3_IN12},
    {GPIO_ADC1_IN13, GPIO_ADC2_IN13, GPIO_ADC3_IN13},
    {GPIO_ADC1_IN14, GPIO_ADC2_IN14, GPIO_ADC3_IN14},
    {GPIO_ADC1_IN15, GPIO_ADC2_IN15, GPIO_ADC3_IN15}
};

/* Number of averaging samples (common to all channels) */
static uint8_t adc_avg_count;

/**
 * @brief           Set averaging sample count.
 * @param[in]       count: averaging sample count
 */
static inline void adc_set_averaging(uint8_t count)
{
    adc_avg_count = count;
    dbg_verbose("%s(): new averaging sample count = %u\n", __func__,
                adc_avg_count);
}

/**
 * @brief           Return the current averaging sample count.
 * @return          current averaging sample count
 */
static inline uint8_t adc_get_averaging(void)
{
    dbg_verbose("%s(): ADC avg count = %u\n", __func__,
                adc_avg_count);
    return adc_avg_count;
}

/**
 * @brief           Return the ADC channel GPIO bit-encoded description.
 * @return          32-bit ADC channel GPIO bit-encoded description
 * @param[in]       adc: ADC instance (1..ADC_COUNT)
 * @param[in]       channel: ADC channel (0..ADC_NCHANNEL-1)
 */
static inline uint32_t adc_get_gpio_pin(uint8_t adc, uint8_t channel)
{
    dbg_verbose("%s(): adc=%u, channel=%u => gpio=%08X\n",
                __func__, adc, channel, adc_gpio[channel][adc - 1]);
    return adc_gpio[channel][adc - 1];
}

/**
 * @brief           Return the base address of the ADC registers.
 * @return          32-bit physical base address of the ADC registers
 * @param[in]       adc: ADC instance (1..ADC_COUNT)
 */
static uint32_t adc_get_base(uint32_t adc)
{
    switch (adc) {
    case ADC1:
        dbg_verbose("%s(): ADC#%u base is 0x%08X\n", __func__,
                    adc, STM32_ADC1_BASE);
        return STM32_ADC1_BASE;
    case ADC2:
        dbg_verbose("%s(): ADC#%u base is 0x%08X\n", __func__,
                    adc, STM32_ADC2_BASE);
        return STM32_ADC2_BASE;
    case ADC3:
        dbg_verbose("%s(): ADC#%u base is 0x%08X\n", __func__,
                    adc, STM32_ADC3_BASE);
        return STM32_ADC3_BASE;
    default:
        dbg_error("%s(): wrong ADC instance! (%u)\n", __func__, adc);
        return 0;
    }
}

/**
 * @brief           Read the value of an ADC register.
 * @return          32-bit ADC register content
 * @param[in]       base: base address of registers unique to this ADC block
 * @param[in]       offset: offset to the register to read
 */
static uint32_t adc_getreg(uint32_t base, int offset)
{
    return getreg32(base + offset);
}

/**
 * @brief           Write value to ADC register.
 * @return          32-bit ADC register content
 * @param[in]       base: base address of registers unique to this ADC block
 * @param[in]       offset: offset to the register to write to
 * @param[in]       value: 32-bit value to be written to the register
 */
static void adc_putreg(uint32_t base, int offset, uint32_t value)
{
    putreg32(value, base + offset);
}

/**
 * @brief           Deinitialize the ADCx peripheral registers to their default
 *                  reset values.
 * @param[in]       base: base address of registers unique to this ADC block
 * @param[in]       reset: condition, set (true) or reset (false)
 */
static void adc_rccreset(uint32_t base, bool reset)
{
    uint32_t regval;

    dbg_verbose("%s: reset=%d\n", __func__, reset);
    /* Set or clear the selected bit in the APB2 reset register */
    regval = getreg32(STM32_RCC_APB2RSTR);
    if (reset) {
        /* Enable  ADC reset state */
        regval |= RCC_APB2RSTR_ADCRST;
    } else {
        /* Release ADC from reset state */
        regval &= ~RCC_APB2RSTR_ADCRST;
    }
    putreg32(regval, STM32_RCC_APB2RSTR);
}

/**
 * @brief           Enable or disable the specified ADC peripheral.
 * @warning         NB: does not start conversion.
 * @param[in]       base: base address of registers unique to this ADC block
 * @param[in]       enable: enable ADC unit (true) or disable ADC unit (false)
 */
static void adc_enable(uint32_t base, bool enable)
{
    uint32_t regval;

    dbg_verbose("%s(): enable=%d\n", __func__, enable);
    regval  = adc_getreg(base, STM32_ADC_CR2_OFFSET);
    if (enable) {
        regval |= ADC_CR2_ADON;
    } else {
        regval &= ~ADC_CR2_ADON;
    }
    adc_putreg(base, STM32_ADC_CR2_OFFSET, regval);
}

/**
 * @brief           Start the ADC conversion process.
 * @warning         Block until conversion is completed.
 * @return          sampled data
 * @param[in]       base: base address of registers unique to this ADC block
 * @param[in]       channel: ADC channel (0..ADC_NCHANNEL-1)
 */
static uint32_t adc_startconv(uint32_t base, uint8_t channel)
{
    uint32_t regval, data;

    dbg_verbose("%s(): start conversion... (base=0x%8X channel=%u)\n", __func__,
                base, channel);

    /* Configure regular channel to be sampled */
    regval = adc_getreg(base, STM32_ADC_SQR3_OFFSET);
    regval |= channel;
    adc_putreg(base, STM32_ADC_SQR3_OFFSET,regval);

    /* Start conversion of regular channels */
    regval = adc_getreg(base, STM32_ADC_CR2_OFFSET);
    regval |= ADC_CR2_SWSTART;
    adc_putreg(base, STM32_ADC_CR2_OFFSET,regval);

    /* Wait until completion */
    do {
        regval = adc_getreg(base, STM32_ADC_SR_OFFSET);
    } while (!(regval & ADC_SR_EOC));

    /* Retrieve data */
    data = adc_getreg(base, STM32_ADC_DR_OFFSET);
    dbg_verbose("%s(): data=%u\n", __func__, data);
    return data;
}

/**
 * @brief           Program the number of cycles required by ADC to sample data.
 * @return          0 on success, -EINVAL otherwise
 * @param[in]       base: base address of registers unique to this ADC block
 * @param[in]       count: sampling cycles (ADC_SMPR_xyz)
 */
static int adc_set_sampling_cycles(uint32_t base, uint8_t count)
{
    uint32_t regval;

    if (count > ADC_SMPR_480) {
        dbg_error("%s(): invalid count argument! (%u)\n", __func__, count);
        return -EINVAL;
    }

    regval  = adc_getreg(base, STM32_ADC_SMPR1_OFFSET);
    regval &= ~(ADC_SMPR1_SMP10_MASK | ADC_SMPR1_SMP11_MASK |
                ADC_SMPR1_SMP12_MASK | ADC_SMPR1_SMP13_MASK |
                ADC_SMPR1_SMP14_MASK | ADC_SMPR1_SMP15_MASK |
                ADC_SMPR1_SMP16_MASK | ADC_SMPR1_SMP17_MASK |
                ADC_SMPR1_SMP18_MASK);
    regval |= ((count << ADC_SMPR1_SMP10_SHIFT) |
               (count << ADC_SMPR1_SMP11_SHIFT) |
               (count << ADC_SMPR1_SMP12_SHIFT) |
               (count << ADC_SMPR1_SMP13_SHIFT) |
               (count << ADC_SMPR1_SMP14_SHIFT) |
               (count << ADC_SMPR1_SMP15_SHIFT) |
               (count << ADC_SMPR1_SMP16_SHIFT) |
               (count << ADC_SMPR1_SMP17_SHIFT) |
               (count << ADC_SMPR1_SMP18_SHIFT));
    adc_putreg(base, STM32_ADC_SMPR1_OFFSET, regval);

    regval  = adc_getreg(base, STM32_ADC_SMPR2_OFFSET);
    regval &= ~(ADC_SMPR2_SMP0_MASK | ADC_SMPR2_SMP1_MASK |
                ADC_SMPR2_SMP2_MASK | ADC_SMPR2_SMP3_MASK |
                ADC_SMPR2_SMP4_MASK | ADC_SMPR2_SMP5_MASK |
                ADC_SMPR2_SMP6_MASK | ADC_SMPR2_SMP7_MASK |
                ADC_SMPR2_SMP8_MASK | ADC_SMPR2_SMP9_MASK);
    regval |= ((count << ADC_SMPR2_SMP0_SHIFT) |
               (count << ADC_SMPR2_SMP1_SHIFT) |
               (count << ADC_SMPR2_SMP2_SHIFT) |
               (count << ADC_SMPR2_SMP3_SHIFT) |
               (count << ADC_SMPR2_SMP4_SHIFT) |
               (count << ADC_SMPR2_SMP5_SHIFT) |
               (count << ADC_SMPR2_SMP6_SHIFT) |
               (count << ADC_SMPR2_SMP7_SHIFT) |
               (count << ADC_SMPR2_SMP8_SHIFT) |
               (count << ADC_SMPR2_SMP9_SHIFT));
    adc_putreg(base, STM32_ADC_SMPR2_OFFSET, regval);

    return 0;
}

/**
 * @brief           Return the number of cycles required by ADC to sample data.
 * @return          Number of cycles required by ADC to sample data (>0)
 * @param[in]       base: base address of registers unique to this ADC block
 */
static uint16_t adc_get_sampling_cycles(uint32_t base)
{
    uint32_t regval;
    uint16_t count;

    regval  = adc_getreg(base, STM32_ADC_SMPR2_OFFSET);
    regval &= ADC_SMPR2_SMP0_MASK;
    switch (regval) {
    case ADC_SMPR_3:
        count = 3;
        break;
    case ADC_SMPR_15:
        count = 15;
        break;
    case ADC_SMPR_28:
        count = 28;
        break;
    case ADC_SMPR_56:
        count = 56;
        break;
    case ADC_SMPR_84:
        count = 84;
        break;
    case ADC_SMPR_112:
        count = 112;
        break;
    case ADC_SMPR_144:
        count = 144;
        break;
    case ADC_SMPR_480:
        count = 480;
        break;
    default:
        /* Cannot happen */
        count = 0;
    }

    dbg_verbose("%s(): sampling cycles = %u\n", __func__, count);
    return count;
}

/**
 * @brief           Reset the ADC device. Called early to initialize hardware.
 * @param[in]       base: base address of registers unique to this ADC block
 */
static void adc_reset(uint32_t base)
{
    uint32_t regval;

    dbg_verbose("%s(): resetting ADC...\n", __func__);

    /* Enable ADC reset state */
    adc_rccreset(base, true);
    /* Release ADC from reset state */
    adc_rccreset(base, false);

    /* Select 144 sampling cycles (to get good acquisition accuracy). */
    adc_set_sampling_cycles(base, ADC_SMPR_144);
    adc_get_sampling_cycles(base);

    /* ADC CR1 Configuration */
    regval  = adc_getreg(base, STM32_ADC_CR1_OFFSET);
    /* Set the resolution of the conversion */
    regval |= ADC_CR1_RES_12BIT;
    adc_putreg(base, STM32_ADC_CR1_OFFSET, regval);

    /* ADC CR2 Configuration */
    regval  = adc_getreg(base, STM32_ADC_CR2_OFFSET);
    /* Clear CONT, continuous mode disable */
    regval &= ~ADC_CR2_CONT;
    /* The EOC bit is set at the end of each regular conversion */
    regval |= ADC_CR2_EOCS;
    /* Set ALIGN (Right = 0) */
    regval &= ~ADC_CR2_ALIGN;
    adc_putreg(base, STM32_ADC_CR2_OFFSET, regval);

    /* ADC CCR configuration */
    regval  = getreg32(STM32_ADC_CCR);
    regval &= ~(ADC_CCR_MULTI_MASK | ADC_CCR_DELAY_MASK | ADC_CCR_DDS |
              ADC_CCR_DMA_MASK | ADC_CCR_ADCPRE_MASK | ADC_CCR_VBATE |
              ADC_CCR_TSVREFE);
    regval |= (ADC_CCR_MULTI_NONE | ADC_CCR_DMA_DISABLED | ADC_CCR_ADCPRE_DIV4);
    putreg32(regval, STM32_ADC_CCR);

    /* Set ADON to wake up the ADC from Power Down state. */
    adc_enable(base, true);

    dbg_verbose("SR:   0x%08x CR1:  0x%08x CR2:  0x%08x\n",
                adc_getreg(base, STM32_ADC_SR_OFFSET),
    adc_getreg(base, STM32_ADC_CR1_OFFSET),
                adc_getreg(base, STM32_ADC_CR2_OFFSET));
    dbg_verbose("SQR1: 0x%08x SQR2: 0x%08x SQR3: 0x%08x\n",
                adc_getreg(base, STM32_ADC_SQR1_OFFSET),
                adc_getreg(base, STM32_ADC_SQR2_OFFSET),
                adc_getreg(base, STM32_ADC_SQR3_OFFSET));
    dbg_verbose("CCR:  0x%08x\n",
                getreg32(STM32_ADC_CCR));

    dbg_verbose("%s(): ADC reset done.\n", __func__);
}

/**
 * @brief           Initialize ADC instance.
 *
 *                  Reset ADC unit, save averaging sample count.
 * @return          0 on success, -EINVAL otherwise.
 * @param[in]       adc: ADC instance (1..ADC_COUNT)
 * @param[in]       channel: ADC channel (0..ADC_NCHANNEL-1)
 * @param[in]       count: averaging sample count
 */
int adc_init(uint8_t adc, uint8_t channel, uint8_t count)
{
    uint32_t base, gpio;

    CHECK_ARG_ADC_DEV(adc);
    CHECK_ARG_ADC_CHANNEL(channel);
    CHECK_ARG_AVG_COUNT(count);

    /* Retrieve ADC registers base address */
    base = adc_get_base(adc);
    /* Retrieve ADC GPIO pin */
    gpio = adc_get_gpio_pin(adc, channel);
    /* Configure ADC channel pin as analog input */
    stm32_configgpio(gpio);

    /* Due to Nuttx architecture (static global variables) */
    adc_set_averaging(count);
    /* Init ADC */
    adc_reset(base);

    return 0;
}

/**
 * @brief           Deinitialize ADC instance (shutdown ADC block).
 * @return          0 on success, -EINVAL otherwise.
 * @param[in]       adc: ADC instance (1..ADC_COUNT)
 * @param[in]       channel: ADC channel (0..ADC_NCHANNEL-1)
 */
int adc_deinit(uint8_t adc, uint8_t channel)
{
    uint32_t base, gpio;

    CHECK_ARG_ADC_DEV(adc);
    CHECK_ARG_ADC_CHANNEL(channel);

    /* Retrieve ADC registers base address */
    base = adc_get_base(adc);
    /* Retrieve ADC GPIO pin */
    gpio = adc_get_gpio_pin(adc, channel);
    /* Unconfigure GPIO ADC channel pin */
    stm32_unconfiggpio(gpio);
    /* Shutdown ADC */
    adc_rccreset(base, 1);

    return 0;
}

/**
 * @brief           Return the ADC sampling time.
 * @return          sampling time in microseconds (>0), -EINVAL otherwise.
 * @param[in]       adc: ADC instance (1..ADC_COUNT)
 */
int32_t adc_get_sampling_time(uint8_t adc)
{
    uint32_t base, stime, count;

    CHECK_ARG_ADC_DEV(adc);

    /* Retrieve ADC registers base address */
    base = adc_get_base(adc);
    /* Retrieve configured ADC sampling cycles */
    count = adc_get_sampling_cycles(base);
    /*
     * ADCCLK, common to all ADCs, is generated from the APB2 clock
     * divided by ADCPRE programmable prescaler in ADC_CCR register.
     */
    stime = getreg32(STM32_ADC_CCR);
    stime &= ADC_CCR_ADCPRE_MASK;
    stime = 2 * ((stime >> ADC_CCR_ADCPRE_SHIFT) + 1);
    dbg_verbose("%s(): prescaler ADCPRE=%u\n", __func__, stime);
    stime = STM32_PCLK2_FREQUENCY / stime / 1000000; /* in MHz */
    dbg_verbose("%s(): PCLK2=%uHz ADCCLK=%uMHz (PCLK2/ADCPRE)\n", __func__,
                STM32_PCLK2_FREQUENCY, stime);
    stime = count / stime;
    dbg_verbose("%s(): sampling time=%uus\n", __func__, stime);
    stime = stime * adc_get_averaging();
    dbg_verbose("%s(): sampling time (with avg=%u)=%uus\n", __func__,
                adc_get_averaging(), stime);

    return stime;
}

/**
 * @brief           Sample data on selected ADC channel.
 * @warning         May include averaging, as configured via adc_init().
 * @return          0 on success, -EINVAL otherwise.
 * @param[in]       adc: ADC instance (1..ADC_COUNT)
 * @param[in]       channel: ADC channel (0..ADC_NCHANNEL-1)
 * @param[out]      data: sampled data
 */
int adc_get_data(uint8_t adc, uint8_t channel, uint32_t *data)
{
    uint32_t base;
    uint8_t i, avg_count;
    uint32_t sample;

    CHECK_ARG_ADC_DEV(adc);
    CHECK_ARG_ADC_CHANNEL(channel);
    CHECK_NULL_ARG(data);

    *data = 0;
    /* Retrieve ADC registers base address */
    base = adc_get_base(adc);
    /* Retrieve averaging sample count */
    avg_count = adc_get_averaging();

    for (i = 0; i < avg_count; i++) {
        /* Start ADC conversion, blocking until completed */
        sample = adc_startconv(base, channel);
        dbg_verbose("%s(): new sample=%u\n", __func__, sample);
        /* Samples are 12-bit, no overflow risk */
        *data += sample;
    }
    *data /= avg_count;

    dbg_verbose("%s(): data=%u (avg count=%u)\n", __func__, *data, avg_count);
    return 0;
}
