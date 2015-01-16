/****************************************************************************
 * configs/endo/svc/src/up_epm.c
 * Endo/SVC EPM support
 *
 * There are control signals for only 2 EPMS: A1, A2.
 *
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 ****************************************************************************/
#define DBG_COMP DBG_EPM
#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_debug.h"
#include "up_epm.h"
#include "up_internal.h"
#include "stm32.h"

/* Timings in us */
#define EPM_FRONT_L_TIME    18      /* Pulse duration for front EPMs */
#define EPM_REAR_L_TIME     53      /* Pulse duration for REAR EPMs */
#define EPM_S_TIME          50000   /* Delay between pulses */
#define EPM_NB_PULSES       4       /* Number of consecutive pulses */
#define EPM_T_TIME          100000  /* 28V charge pump stabilisation delay */

/* Base frequency for timers, derived from HCLK */
#define TIMER_FREQ          STM32_PCLK1_FREQUENCY
#define FREQ_1MHZ           1000000ul

static uint32_t epm_state;


/*
 * Set default lines states to prevent any forbidden state
 *
 * The states where HB*_N and HB*_P are simultaneously set high
 * are forbidden, this causes HW damage
 */
static void epm_init_hb_lines(void)
{
    /* Initialize EPM HB pins as output low by default */
    stm32_configgpio(GPIO_HB0_N | GPIO_EPM_DEFAULT);
    stm32_gpiowrite(GPIO_HB0_N, false);
    stm32_configgpio(GPIO_HB0_P | GPIO_EPM_DEFAULT);
    stm32_gpiowrite(GPIO_HB0_P, false);
    stm32_configgpio(GPIO_HB_A1_N | GPIO_EPM_DEFAULT);
    stm32_gpiowrite(GPIO_HB_A1_N, false);
    stm32_configgpio(GPIO_HB_A1_P | GPIO_EPM_DEFAULT);
    stm32_gpiowrite(GPIO_HB_A1_P, false);
    stm32_configgpio(GPIO_HB_A2_N | GPIO_EPM_DEFAULT);
    stm32_gpiowrite(GPIO_HB_A2_N, false);
    stm32_configgpio(GPIO_HB_A2_P | GPIO_EPM_DEFAULT);
    stm32_gpiowrite(GPIO_HB_A2_P, false);
}

/* Get charge pump state */
int epm_get_28v_state(void)
{
    return (epm_state & (1 << EPM_STATE_28V)) ? 1 : 0;
}

/* Set charge pump state */
int epm_activate_28v(uint32_t state)
{
    dbg_info("%s(): state=%d\n", __func__, state);

    /*
     * Control the 28V charge pump:
     *  Enable it
     *  Wait for stabilisation, to be replaced by an auto-off timer
     *
     * Note: the charge pump is not present on the BDB
     */

    /* Update EPM state */
    if (state)
        epm_state |= (1 << EPM_STATE_28V);
    else
        epm_state &= ~(1 << EPM_STATE_28V);

    return 0;
}

/* Get EPM state */
int epm_get_state(uint32_t epm_nr)
{
    if (epm_nr >= NR_EPMS)
        return -1;

    return (epm_state & (1 << epm_nr)) ? 1 : 0;
}

/* Activate or de-activate EPM */
int epm_set_state(uint32_t epm_nr, uint32_t state)
{
    uint32_t timer_nr, channel, high_gpio_pin, pulse_duration;
    uint32_t reload_value;
    struct stm32_tim_dev_s *timer;
    int i;

    if (epm_nr >= NR_EPMS)
        return -1;

    dbg_info("Setting EPM %d state to %d\n", epm_nr, state);

    /*
     * Set default lines states to prevent any forbidden state
     *
     * The states where HB*_N and HB*_P are simultaneously set high
     * are forbidden, this causes HW damage
     */
    epm_init_hb_lines();

    /*
     * Identify which lines to toggle via GPIO or via the timers.
     * The time critical line is driven by the timer for accuracy.
     *
     * The EPM lines states are: F (float), N (0V) and P (28V).
     *
     * When locking an EPM, the EPM_xx lines have a fixed value and
     * a pulse F-P-F is generated on EPM0:
     * - HB0_N, HB_xx_N, HB_xx_P are GPIOs,
     * - HB0_P is controlled by a timer.
     *
     * When unlocking an EPM, EPM0 has a fixed value and a pulse F-P-F
     * is generated on the APM_xx lines:
     * - HB0_N, HB0_P, HB_xx_N are GPIOs,
     * - HB_xx_P is controlled by a timer.
     */
    if (state) {
        /* Lock EPM */
        switch (epm_nr) {
        case EPM_A1:
            /*
             * GPIO_HB0_N stays low, GPIO_HB0_P gets a timer pulse
             * GPIO_HB_A1_N is set high, GPIO_HB_A1_P stays low
             */
            pulse_duration = EPM_REAR_L_TIME;
            /* T2C2, Timer pin is GPIO_HB0_P */
            timer_nr = 2;
            channel = 2;
            high_gpio_pin = GPIO_HB_A1_N;
            break;
        case EPM_A2:
            /*
             * GPIO_HB0_N stays low, GPIO_HB0_P gets a timer pulse
             * GPIO_HB_A2_N is set high, GPIO_HB_A2_P stays low
             */
            pulse_duration = EPM_REAR_L_TIME;
            /* T2C2, Timer pin is GPIO_HB0_P */
            timer_nr = 2;
            channel = 2;
            high_gpio_pin = GPIO_HB_A2_N;
            break;
        default:
            dbg_warn("%s(): Wrong EPM number %d (state=%d), exiting\n",
                     __func__, epm_nr, state);
            return ERROR;
        }
    } else {
        /* Unlock EPM */
        switch (epm_nr) {
        case EPM_A1:
            /*
             * GPIO_HB0_N is set high, GPIO_HB0_P stays low
             * GPIO_HB_A1_N stays low, GPIO_HB_A1_P gets a timer pulse
             */
            pulse_duration = EPM_REAR_L_TIME;
            /* T2C3, Timer pin is GPIO_HB_A1_P */
            timer_nr = 2;
            channel = 3;
            high_gpio_pin = GPIO_HB0_N;
            break;
        case EPM_A2:
            /*
             * GPIO_HB0_N is set high, GPIO_HB0_P stays low
             * GPIO_HB_A2_N stays low, GPIO_HB_A2_P gets a timer pulse
             */
            pulse_duration = EPM_REAR_L_TIME;
            /* T2C4, Timer pin is GPIO_HB_A2_P */
            timer_nr = 2;
            channel = 4;
            high_gpio_pin = GPIO_HB0_N;
            break;
        default:
            dbg_warn("%s(): Wrong EPM number %d (state=%d), exiting\n",
                   __func__, epm_nr, state);
            return ERROR;
        }
    }

    /* Get timer instance */
    timer = stm32_tim_init(timer_nr);
    if (!timer) {
        dbg_error("%s(): Cannot get timer %d for EPM %d\n", __func__,
                  timer_nr, epm_nr);
        return ERROR;
    }

    /* Configure timer as OPM (One Pulse Mode) 0-1-0 on timer_pin */
    reload_value = (TIMER_FREQ / FREQ_1MHZ) * pulse_duration;
    /*  Configure ARR (reload value), defines the pulse duration */
    STM32_TIM_SETPERIOD(timer, reload_value + 1);
    /*  Configure CCR (pre-delay), must be different from the initial count */
    STM32_TIM_SETCOMPARE(timer, channel, 1);
    /*  Configure timer channel and
     *  change GPIO config to TIM<timer>_CH<channel>OUT */
    STM32_TIM_SETCHANNEL(timer, channel,
                         STM32_TIM_CH_OUTPWM | STM32_TIM_CH_POLARITY_NEG);

    /* Assert high_gpio_pin */
    stm32_gpiowrite(high_gpio_pin, true);

    /* Generate the required amount of pulses on timer_pin */
    for (i = 0; i < EPM_NB_PULSES; i++) {
        /*  Configure mode: OPM, up counting, not enabled */
        STM32_TIM_SETMODE(timer,
                          STM32_TIM_MODE_PULSE | STM32_TIM_MODE_DONOTSTART);
        /* Set the clock, reload and start timer to drive timer_pin */
        STM32_TIM_SETCLOCK(timer, TIMER_FREQ);
        /* Delay between consecutive pulses (S time) */
        up_udelay(EPM_S_TIME);
    }

    /*
     * Set back default lines states to prevent any forbidden state
     *
     * The states where HB*_N and HB*_P are simultaneously set high
     * are forbidden, this causes HW damage
     */
    epm_init_hb_lines();

    /* Release timer instance */
    stm32_tim_deinit(timer);

    /* Update state */
    if (state)
        epm_state |= (1 << epm_nr);
    else
        epm_state &= ~(1 << epm_nr);

    return 0;
}

int epm_init(void)
{
    dbg_info("%s()\n", __func__);

    /* Disable charge pump */
    epm_activate_28v(0);

    /*
     * Set default lines states to prevent any forbidden state
     *
     * The states where HB*_N and HB*_P are simultaneously set high
     * are forbidden, this causes HW damage
     */
    epm_init_hb_lines();

    epm_state = 0;

    return 0;
}
