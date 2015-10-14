/*
 * Copyright (c) 2015 Google Inc.
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

/*
 * @file    configs/ara/svc/src/up_spring_pm.c
 * @brief   ARA Spring Power Measurement Library
 * @author  Patrick Titiano
 */

#define DBG_COMP ARADBG_POWER

#include <up_adc.h>
#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>

#include <nuttx/analog/adc.h>
#include <arch/board/board.h>
#include <ara_board.h>

#include "stm32_adc.h"
#include "chip.h"
#include "up_arch.h"
#include <ara_debug.h>
#include <up_spring_pm.h>
#include <svc.h>
#include <assert.h>
#include <interface.h>

#define ADC_VREF                    ((uint32_t) 1800000) /* uV */
#define ADC_VOLTAGE_LSB             ((uint32_t) (ADC_VREF / (1 << ADC_PRECISION))) /* uV */
#define MAX9929F_RSENSE_RECIPROCAL  ((uint32_t) 100) /* 0.010 ohms */
#define MAX9929F_AV_GAIN            ((uint32_t) 50) /* 50V/V */

#define CHECK_NULL_ARG(arg) \
    if ((void *) arg == NULL) { \
        dbg_error("%s(): %s == NULL!!!\n", __func__, #arg); \
        return -EINVAL; \
    }

#define CHECK_SPRING(spring) \
    if ((spring == 0) || (spring > spwrm_get_count())) { \
        dbg_error("%s(): invalid spring!!! (%u) !!!\n", \
            __func__, #spring); \
        return -EINVAL; \
    }

#define CHECK_ARG_AVG_COUNT(count) \
    if ((count == 0) || (count > ADC_MAX_AVG_SAMPLE_COUNT)) { \
        dbg_error("%s(): invalid AVG count!!! (%u) !!!\n", \
            __func__, #count); \
        return -EINVAL; \
    }

/**
 * @brief           Convert sampled 12-bit data into voltage (microvolts),
 *                  considering ADC reference voltage and voltage LSB.
 * @return          Voltage (microvolts)
 * @param[in]       data: ADC sampled data
 */
static inline uint32_t adc_get_uV(uint32_t data)
{
    uint32_t uV;

    uV = data * ADC_VOLTAGE_LSB;
    dbg_verbose("%s(): data=%u, lsb=%uuV, uV=%uuV\n", __func__,
                data, ADC_VOLTAGE_LSB, uV);

    return uV;
}

/**
 * @brief           Convert voltage into current, following MAX9929F device
 *                  specifications.
 * @return          Current (microamps)
 * @param[in]       uV: voltage (in microvolts)
 */
static inline uint32_t max9929f_get_Iload(uint32_t uV)
{
    uint32_t uA;

    uA = (uV / MAX9929F_AV_GAIN) * MAX9929F_RSENSE_RECIPROCAL;
    dbg_verbose("%s(): uV=%uuV, Rs=100 mohms, gain=%u uA=%u\n", __func__,
                uV, MAX9929F_AV_GAIN, uA);
    return uA;
}

/**
 * @brief           Return the number of spring(s) on this platform.
 * @return          Number of spring(s)
 */
uint8_t spwrm_get_count(void)
{
    return interface_get_spring_count();
}

/**
 * @brief           Return the name of a given spring.
 * warning          'spring' index starts at 1, not 0.
 * @return          Spring name (string)
 * @param[in]       spring: selected spring ([1..SPRING_COUNT])
 */
const char *spwrm_get_name(uint8_t spring)
{
    struct interface *iface;

    DEBUGASSERT(spring > 0);
    DEBUGASSERT(spring <= interface_get_spring_count());
    iface = interface_spring_get(spring - 1);
    DEBUGASSERT(iface);

    return interface_get_name(iface);
}

/**
 * @brief           Return the ADC instance used to measure current consumption
 *                  of a given spring.
 * warning          'spring' index starts at 1, not 0.
 * @return          ADC instance
 * @param[in]       spring: selected spring ([1..SPRING_COUNT])
 */
static uint8_t spwrm_get_adc_device(uint8_t spring)
{
    struct interface *iface;

    DEBUGASSERT(spring > 0);
    DEBUGASSERT(spring <= interface_get_spring_count());
    iface = interface_spring_get(spring - 1);
    DEBUGASSERT(iface);

    return interface_pm_get_adc(iface);
}

/**
 * @brief           Return the ADC channel used to measure current consumption
 *                  of a given spring.
 * warning          'spring' index starts at 1, not 0.
 * @return          ADC channel
 * @param[in]       spring: selected spring ([1..SPRING_COUNT])
 */
static uint8_t spwrm_get_adc_channel(uint8_t spring)
{
    struct interface *iface;

    DEBUGASSERT(spring > 0);
    DEBUGASSERT(spring <= interface_get_spring_count());
    iface = interface_spring_get(spring - 1);
    DEBUGASSERT(iface);

    return interface_pm_get_chan(iface);
}

/**
 * @brief           Return the GPIO configuration of the pin used to get
 *                  the sign of the current measurement
 * warning          'spring' index starts at 1, not 0.
 * @return          Current measurement sign pin GPIO configuration
 * @param[in]       spring: selected spring ([1..SPRING_COUNT])
 */
static uint32_t spwrm_get_sign_pin(uint8_t spring)
{
    struct interface *iface;

    DEBUGASSERT(spring > 0);
    DEBUGASSERT(spring <= interface_get_spring_count());
    iface = interface_spring_get(spring - 1);
    DEBUGASSERT(iface);

    return interface_pm_get_spin(iface);
}

/**
 * @brief           Initialize dedicated HW (GPIO, ADC) and configure averaging
 *                  to enable current measurement of a given spring.
 * @warning         Shall be called prior to any other spwrm_* call.
 * warning          'spring' index starts at 1, not 0.
 * @return          0 on success, standard error codes otherwise.
 * @param[in]       spring: selected spring ([1..SPRING_COUNT])
 * @param[in]       count: averaging sample count
 *                  ([1..ADC_MAX_AVG_SAMPLE_COUNT])
 */
int spwrm_init(uint8_t spring, uint8_t count)
{
    uint8_t adc, chan;
    uint32_t spin;
    int ret;

    CHECK_SPRING(spring);
    CHECK_ARG_AVG_COUNT(count);

    adc = spwrm_get_adc_device(spring);
    chan = spwrm_get_adc_channel(spring);
    spin = spwrm_get_sign_pin(spring);

    dbg_verbose("%s(): Configuring %s... (adc=%u, spin=0x%08X)\n",
                __func__, spwrm_get_name(spring), adc, spin);

    /* Configure SIGN pin as input */
    stm32_configgpio(spin);

    ret = adc_init(adc, chan, count);
    if (ret) {
        dbg_error("%s(): %s initialization failed!!! (%d).\n", __func__,
                    spwrm_get_name(spring), ret);
    } else {
        dbg_verbose("%s(): %s initialization done.\n", __func__,
                    spwrm_get_name(spring));
    }

    return ret;
}


/**
 * @brief           Deinitialize dedicated current measurement HW
 *                  (GPIO, ADC) of a given spring.
 * warning          'spring' index starts at 1, not 0.
 * @return          0 on success, standard error otherwise.
 * @param[in]       spring: selected spring ([1..SPRING_COUNT])
 */
int spwrm_deinit(uint8_t spring)
{
    int ret;
    uint8_t adc, chan;
    uint32_t spin;

    CHECK_SPRING(spring);

    dbg_verbose("%s(): De-initializing %s...\n", __func__,
                spwrm_get_name(spring));

    adc = spwrm_get_adc_device(spring);
    chan = spwrm_get_adc_channel(spring);
    spin = spwrm_get_sign_pin(spring);

    /* Unconfigure GPIO ADC channel pin */
    stm32_unconfiggpio(spin);
    /* Shutdown ADC */
    ret = adc_deinit(adc, chan);
    if (ret) {
        dbg_error("%s(): %s de-initialization failed!!! (%d)\n", __func__,
                  spwrm_get_name(spring), ret);
    } else {
        dbg_verbose("%s(): %s de-initialization done.\n", __func__,
                    spwrm_get_name(spring));
    }
    return ret;
}

/**
 * @brief           Measure the current consumption of a given spring.
 * @return          current consumption of a given spring (in microamps)
 * warning          'spring' index starts at 1, not 0.
 * @param[in]       spring: selected spring ([1..SPRING_COUNT])
 * @param[out]      uA: selected spring current measurement, in microamps)
 */
int spwrm_get_current(uint8_t spring, int32_t *uA)
{
    int ret;
    uint8_t adc, chan;
    uint32_t spin, uV;

    CHECK_SPRING(spring);
    CHECK_NULL_ARG(uA);

    adc = spwrm_get_adc_device(spring);
    chan = spwrm_get_adc_channel(spring);
    spin = spwrm_get_sign_pin(spring);
    *uA = 0;

    ret = adc_get_data(adc, chan, &uV);
    if (ret) {
        dbg_error("%s(): failed to get %s data! (%d)\n", __func__,
                  spwrm_get_name(spring), ret);
        return ret;
    }
    /* Convert to uV */
    uV = adc_get_uV(uV);
    /* Get data sign */
    if (stm32_gpioread(spin) == 0) {
        dbg_verbose("%s(): pin=0 => sign=-\n", __func__);
        *uA = -((int32_t) max9929f_get_Iload(uV));
    } else {
        dbg_verbose("%s(): pin=1 => sign=+\n", __func__);
        *uA = (int32_t) max9929f_get_Iload(uV);
    }
    dbg_verbose("%s(): measured voltage=%uuV => current=%duA\n",
                __func__, uV, *uA);

    return 0;
}

/**
 * @brief           Return time required to sample spring current consumption
 * warning          'spring' index starts at 1, not 0.
 * @return          spring current consumption sampling time (in microseconds),
 *                  standard error codes otherwise (<0)
 * @param[in]       spring: selected spring ([1..SPRING_COUNT])
 */
int32_t spwrm_get_sampling_time(uint8_t spring)
{
    uint8_t adc;

    CHECK_SPRING(spring);

    adc = spwrm_get_adc_device(spring);
    dbg_verbose("%s(): %s sampling time = %dus\n",
                __func__, spwrm_get_name(spring), adc_get_sampling_time(adc));
    return adc_get_sampling_time(adc);
}
