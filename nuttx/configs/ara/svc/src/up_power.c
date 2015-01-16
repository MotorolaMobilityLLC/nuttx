/****************************************************************************
 * configs/endo/svc/src/up_power.c
 * Endo/SVC power support: bridges power supply control, wake and detect
 *
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 ****************************************************************************/
#define DBG_COMP DBG_POWER
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
#include "up_gpio.h"
#include "up_i2c.h"
#include "up_internal.h"
#include "up_ioexp.h"
#include "up_power.h"
#include "stm32.h"

/* Internal power supply stabilisation time, in us */
#define POWER_INTERNAL_STABILISATION_TIME   20000
/* Switch power supply stabilisation time (1V1, 1V8), in us */
#define POWER_SWITCH_ON_STABILISATION_TIME  50000
/* Switch power supply shutdown time (1V1, 1V8), in us */
#define POWER_SWITCH_OFF_STABILISATION_TIME 100000

/* Interfaces power supply state. Springs B & E are always enabled */
static uint32_t power_state = (1 << PWR_SPRING_B) | (1 << PWR_SPRING_E);


/*
 * Enable/Disable the switch power supplies (1V1 and 1V8).
 * The sequencing is described in the switch spec '10.5.1 Power On Reset'.
 * The reset line should be asserted before cycling the power supplies,
 * and de-asserted after enabling them.
 * Sysclk is already enabled since power up.
 */
void power_cycle_switch(void)
{
    dbg_info("%s()\n", __func__);

    /* Power off */
    stm32_gpiowrite(GPIO_1V1_ON_EN, false);
    stm32_gpiowrite(GPIO_1V8_ON_EN, false);
    up_udelay(POWER_SWITCH_OFF_STABILISATION_TIME);

    /* Power on */
    /* First 1V1, wait for stabilisation */
    stm32_gpiowrite(GPIO_1V1_ON_EN, true);
    up_udelay(POWER_SWITCH_ON_STABILISATION_TIME);
    /* Then 1V8, wait for stabilisation */
    stm32_gpiowrite(GPIO_1V8_ON_EN, true);
    up_udelay(POWER_SWITCH_ON_STABILISATION_TIME);
}

/*
 * Enable internal SVC power supplies. The switch power is controlled by
 * power_cycle_switch
 */
void power_enable_internal(void)
{
    dbg_info("%s()\n", __func__);

    stm32_configgpio(GPIO_LIMIT_ON_EN);
    stm32_configgpio(GPIO_1V8_ON_EN);
    stm32_configgpio(GPIO_1V1_ON_EN);
    stm32_configgpio(GPIO_2V95_ON_EN);
    stm32_gpiowrite(GPIO_LIMIT_ON_EN, true);
    stm32_gpiowrite(GPIO_2V95_ON_EN, true);

    /* Stabilization time */
    up_udelay(POWER_INTERNAL_STABILISATION_TIME);
}

/*
 * Configure WAKEOUT as input, floating so that it does not interfere
 * with the wake and detect input pin
 */
static void wakeout_init(void)
{
    dbg_info("%s()\n", __func__);

    stm32_configgpio(GPIO_WAKE_OUT_A | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_B | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_C | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_D | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_E | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_F | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_G | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_H | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_I | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_J | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_K | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_L | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_M | GPIO_INPUT);
    stm32_configgpio(GPIO_WAKE_OUT_N | GPIO_INPUT);
}

/* Init pins to interface block power control and WAKEOUT */
void power_interface_block_init(void)
{
    dbg_info("%s()\n", __func__);

    /* Power supply control */
    stm32_configgpio(GPIO_PWR_SW_N1);
    stm32_configgpio(GPIO_PWR_SW_N3);
    stm32_configgpio(GPIO_PWR_SW_N4);
    stm32_configgpio(GPIO_PWR_SW_N6);
    stm32_configgpio(GPIO_PWR_SW_N7);
    stm32_configgpio(GPIO_PWR_SW_N8);
    stm32_configgpio(GPIO_PWR_SW_N9);
    stm32_configgpio(GPIO_PWR_SW_N10);
    stm32_configgpio(GPIO_PWR_SW_N11);
    stm32_configgpio(GPIO_PWR_SW_N12);
    stm32_configgpio(GPIO_PWR_SW_N13);
    stm32_configgpio(GPIO_PWR_SW_N14);
    stm32_gpiowrite(GPIO_PWR_SW_N1, true);
    stm32_gpiowrite(GPIO_PWR_SW_N3, true);
    stm32_gpiowrite(GPIO_PWR_SW_N4, true);
    stm32_gpiowrite(GPIO_PWR_SW_N6, true);
    stm32_gpiowrite(GPIO_PWR_SW_N7, true);
    stm32_gpiowrite(GPIO_PWR_SW_N8, true);
    stm32_gpiowrite(GPIO_PWR_SW_N9, true);
    stm32_gpiowrite(GPIO_PWR_SW_N10, true);
    stm32_gpiowrite(GPIO_PWR_SW_N11, true);
    stm32_gpiowrite(GPIO_PWR_SW_N12, true);
    stm32_gpiowrite(GPIO_PWR_SW_N13, true);
    stm32_gpiowrite(GPIO_PWR_SW_N14, true);

    /* WAKEOUT pins */
    wakeout_init();
}

/* Get interface power supply state */
bool power_get_power(uint32_t int_nr)
{
    if (int_nr >= PWR_SPRING_NR)
        return false;

    return (power_state & (1 << int_nr)) ? true : false;
}

/* Control interface block power */
int power_set_power(uint32_t int_nr, bool enable)
{
    uint32_t enable_pin;

    dbg_info("%s(): interface %d, enable=%d\n", __func__, int_nr, enable);

    switch(int_nr) {
    case PWR_SPRING_A:
        enable_pin = GPIO_PWR_SW_N1;
        break;
    case PWR_SPRING_B:
        /* No power control for Spring B, used for debug only */
        return 0;
        break;
    case PWR_SPRING_C:
        enable_pin = GPIO_PWR_SW_N3;
        break;
    case PWR_SPRING_D:
        enable_pin = GPIO_PWR_SW_N4;
        break;
    case PWR_SPRING_E:
        /* No power control for Spring E, used for debug only */
        return 0;
        break;
    case PWR_SPRING_F:
        enable_pin = GPIO_PWR_SW_N6;
        break;
    case PWR_SPRING_G:
        enable_pin = GPIO_PWR_SW_N7;
        break;
    case PWR_SPRING_H:
        enable_pin = GPIO_PWR_SW_N8;
        break;
    case PWR_SPRING_I:
        enable_pin = GPIO_PWR_SW_N9;
        break;
    case PWR_SPRING_J:
        enable_pin = GPIO_PWR_SW_N10;
        break;
    case PWR_SPRING_K:
        enable_pin = GPIO_PWR_SW_N11;
        break;
    case PWR_SPRING_L:
        enable_pin = GPIO_PWR_SW_N12;
        break;
    case PWR_SPRING_M:
        enable_pin = GPIO_PWR_SW_N13;
        break;
    case PWR_SPRING_N:
        enable_pin = GPIO_PWR_SW_N14;
        break;
    default:
        return ERROR;
    }

    stm32_gpiowrite(enable_pin, !enable);

    /* Update state */
    if (enable)
        power_state |= (1 << int_nr);
    else
        power_state &= ~(1 << int_nr);

    return 0;
}

/*
 * Generate a WAKEOUT signal to wake-up/power-up modules.
 * If assert is true, keep the WAKEOUT lines asserted.
 *
 * The corresponding power supplies must already be enabled.
 */
void power_set_wakeout(uint32_t int_mask, bool assert)
{
    dbg_info("%s(): interface mask=0x%08x, assert=%d\n", __func__,
             int_mask, assert);

    /*
     * Assert the WAKEOUT line on the interfaces in order to power up the
     * modules.
     * When the WAKEOUT signal is de-asserted the bridges have to assert
     * the PS_HOLD signal asap in order to stay powered up.
     */
    if (int_mask & (1 << PWR_SPRING_A))
        stm32_configgpio(GPIO_WAKE_OUT_A | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_B))
        stm32_configgpio(GPIO_WAKE_OUT_B | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_C))
        stm32_configgpio(GPIO_WAKE_OUT_C | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_D))
        stm32_configgpio(GPIO_WAKE_OUT_D | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_E))
        stm32_configgpio(GPIO_WAKE_OUT_E | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_F))
        stm32_configgpio(GPIO_WAKE_OUT_F | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_G))
        stm32_configgpio(GPIO_WAKE_OUT_G | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_H))
        stm32_configgpio(GPIO_WAKE_OUT_H | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_I))
        stm32_configgpio(GPIO_WAKE_OUT_I | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_J))
        stm32_configgpio(GPIO_WAKE_OUT_J | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_K))
        stm32_configgpio(GPIO_WAKE_OUT_K | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_L))
        stm32_configgpio(GPIO_WAKE_OUT_L | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_M))
        stm32_configgpio(GPIO_WAKE_OUT_M | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    if (int_mask & (1 << PWR_SPRING_N))
        stm32_configgpio(GPIO_WAKE_OUT_N | GPIO_OUTPUT | GPIO_OUTPUT_SET);

    if (!assert) {
        /* Wait for the bridges to react */
        usleep(WAKEOUT_PULSE_DURATION);

        /* De-assert the lines */
        if (int_mask & (1 << PWR_SPRING_A))
            stm32_configgpio(GPIO_WAKE_OUT_A | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_B))
            stm32_configgpio(GPIO_WAKE_OUT_B | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_C))
            stm32_configgpio(GPIO_WAKE_OUT_C | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_D))
            stm32_configgpio(GPIO_WAKE_OUT_D | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_E))
            stm32_configgpio(GPIO_WAKE_OUT_E | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_F))
            stm32_configgpio(GPIO_WAKE_OUT_F | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_G))
            stm32_configgpio(GPIO_WAKE_OUT_G | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_H))
            stm32_configgpio(GPIO_WAKE_OUT_H | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_I))
            stm32_configgpio(GPIO_WAKE_OUT_I | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_J))
            stm32_configgpio(GPIO_WAKE_OUT_J | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_K))
            stm32_configgpio(GPIO_WAKE_OUT_K | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_L))
            stm32_configgpio(GPIO_WAKE_OUT_L | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_M))
            stm32_configgpio(GPIO_WAKE_OUT_M | GPIO_INPUT);
        if (int_mask & (1 << PWR_SPRING_N))
            stm32_configgpio(GPIO_WAKE_OUT_N | GPIO_INPUT);
    }
}

/* Read Wake and Detect states, for ~10s then exit */
void power_read_wake_detect(void)
{
    uint8_t msg[4];
    uint32_t x = 0;

    dbg_info("%s()\n", __func__);

    // configure IO Expander pins as input
    msg[0] = 0x06;
    msg[1] = 0xFF;
    msg[2] = 0xFF;
    i2c_ioexp_write(msg, 3, I2C_ADDR_IOEXP_U601);

    msg[0] = 0x06;
    msg[1] = 0xFF;
    msg[2] = 0xFF;
    i2c_ioexp_write(msg, 3, I2C_ADDR_IOEXP_U602);

    svc_irq_enable();

    while (x < 500) {
        /* Check if we got an IRQ, if so let's handle it */
        if (sem_trywait(&svc_irq_sem) == 0)
            ioexp_read_iopins();

        /* Generate a wake up pulse on spring M (display) every x cycles */
        if ((x & 0xff) == 14) {
            dbg_info("%s(): wakeout pulse on\n", __func__);
            power_set_wakeout(1 << PWR_SPRING_M, false);
        }

        // Wait
        usleep(20000);

        x++;
    }
}
