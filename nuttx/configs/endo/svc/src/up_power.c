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
#include "stm32.h"


/* Init APB1 power control */
void bdb_apb1_init(void)
{
    dbg_info("%s()\n", __func__);

    stm32_configgpio(GPIO_VAPB1_1P1_EN);
    stm32_configgpio(GPIO_VAPB1_1P2_EN);
    stm32_configgpio(GPIO_VAPB1_1P8_EN);
}

/* Enable APB1 power */
void bdb_apb1_enable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB1_1P1_EN, true);
    stm32_gpiowrite(GPIO_VAPB1_1P2_EN, true);
    stm32_gpiowrite(GPIO_VAPB1_1P8_EN, true);
}

/* Disable APB1 power */
void bdb_apb1_disable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB1_1P1_EN, false);
    stm32_gpiowrite(GPIO_VAPB1_1P2_EN, false);
    stm32_gpiowrite(GPIO_VAPB1_1P8_EN, false);
}

/* Init APB2 power control */
void bdb_apb2_init(void)
{
    dbg_info("%s()\n", __func__);

    stm32_configgpio(GPIO_VAPB2_1P1_EN);
    stm32_configgpio(GPIO_VAPB2_1P2_EN);
    stm32_configgpio(GPIO_VAPB2_1P8_EN);
}

/* Enable APB2 power */
void bdb_apb2_enable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB2_1P1_EN, true);
    stm32_gpiowrite(GPIO_VAPB2_1P2_EN, true);
    stm32_gpiowrite(GPIO_VAPB2_1P8_EN, true);
}

/* Disable APB2 power */
void bdb_apb2_disable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB2_1P1_EN, false);
    stm32_gpiowrite(GPIO_VAPB2_1P2_EN, false);
    stm32_gpiowrite(GPIO_VAPB2_1P8_EN, false);
}

/* Init APB3 power control */
void bdb_apb3_init(void)
{
    dbg_info("%s()\n", __func__);

    stm32_configgpio(GPIO_VAPB3_1P1_EN);
    stm32_configgpio(GPIO_VAPB3_1P2_EN);
    stm32_configgpio(GPIO_VAPB3_1P8_EN);
}

/* Enable APB3 power */
void bdb_apb3_enable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB3_1P1_EN, true);
    stm32_gpiowrite(GPIO_VAPB3_1P2_EN, true);
    stm32_gpiowrite(GPIO_VAPB3_1P8_EN, true);
}

/* Disable APB3 power */
void bdb_apb3_disable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB3_1P1_EN, false);
    stm32_gpiowrite(GPIO_VAPB3_1P2_EN, false);
    stm32_gpiowrite(GPIO_VAPB3_1P8_EN, false);
}

/* Init GPB1 power control */
void bdb_gpb1_init(void)
{
    dbg_info("%s()\n", __func__);

    stm32_configgpio(GPIO_VGPB1_1P1_EN);
    stm32_configgpio(GPIO_VGPB1_1P2_EN);
    stm32_configgpio(GPIO_VGPB1_1P8_EN);
    stm32_configgpio(GPIO_VGPB1_SDIO_EN);
}

/* Enable GPB1 power */
void bdb_gpb1_enable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VGPB1_1P1_EN, true);
    stm32_gpiowrite(GPIO_VGPB1_1P2_EN, true);
    stm32_gpiowrite(GPIO_VGPB1_1P8_EN, true);
    stm32_gpiowrite(GPIO_VGPB1_SDIO_EN, true);
}

/* Disable GPB1 power */
void bdb_gpb1_disable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VGPB1_1P1_EN, false);
    stm32_gpiowrite(GPIO_VGPB1_1P2_EN, false);
    stm32_gpiowrite(GPIO_VGPB1_1P8_EN, false);
    stm32_gpiowrite(GPIO_VGPB1_SDIO_EN, false);
}

/* Init GPB2 power control */
void bdb_gpb2_init(void)
{
    dbg_info("%s()\n", __func__);

    /*  Power control to the GPB2 */
    stm32_configgpio(GPIO_VGPB2_1P1_EN);
    stm32_configgpio(GPIO_VGPB2_1P2_EN);
    stm32_configgpio(GPIO_VGPB2_1P8_EN);
    stm32_configgpio(GPIO_VGPB2_SDIO_EN);
}

/* Enable GPB2 power */
void bdb_gpb2_enable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VGPB2_1P1_EN, true);
    stm32_gpiowrite(GPIO_VGPB2_1P2_EN, true);
    stm32_gpiowrite(GPIO_VGPB2_1P8_EN, true);
    stm32_gpiowrite(GPIO_VGPB2_SDIO_EN, true);
}

/* Disable APB2 power */
void bdb_gpb2_disable(void)
{
    dbg_info("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VGPB2_1P1_EN, false);
    stm32_gpiowrite(GPIO_VGPB2_1P2_EN, false);
    stm32_gpiowrite(GPIO_VGPB2_1P8_EN, false);
    stm32_gpiowrite(GPIO_VGPB2_SDIO_EN, false);
}

/* Wake and detect validation code */
void validate_wake_detect(void)
{
    /*
     * APB1 pins:
     * - interface pin: R310/R318
     * - detect_in: U96/P01, R368
     * - wake_in: U96/P02, R367
     * - wake_out: PI0
     *
     * BB1 pins:
     * - interface pin: J87/1
     * - detect_in: U96/P20
     * - wake_in: U96/P21
     * - wake_out: PH13
     */
    uint8_t msg[4];
    uint32_t x = 0;

    dbg_info("%s()\n", __func__);

    // configure IO Expander pins as input
    msg[0] = 0x8C;
    msg[1] = 0xFF;
    msg[2] = 0xFF;
    msg[3] = 0xFF;
    i2c_ioexp_write(msg, 4, I2C_ADDR_IOEXP_U96);

    msg[0] = 0x03;
    msg[1] = 0xFF;
    i2c_ioexp_write(msg, 2, I2C_ADDR_IOEXP_U90);

    /* Configure wakeout as input */
    stm32_configgpio(GPIO_APB1_WAKE_OUT | GPIO_INPUT);
    stm32_configgpio(GPIO_BB1_WAKE_OUT | GPIO_INPUT);

    svc_irq_enable();

    while (1) {
        /* Check if we got an IRQ, if so let's handle it */
        if (sem_trywait(&svc_irq_sem) == 0)
            ioexp_read_iopins();

        // Generate a module wake up pulse every x cycles
        if ((x & 0xff) == 14) {
            dbg_info("%s(): wakeout pulse on\n", __func__);
            /* Configure wakeout as output, value high */
            stm32_configgpio(GPIO_APB1_WAKE_OUT | GPIO_OUTPUT |
                             GPIO_OUTPUT_SET);
            stm32_configgpio(GPIO_BB1_WAKE_OUT | GPIO_OUTPUT | GPIO_OUTPUT_SET);
        }
        if ((x & 0xff) == 15) {
            dbg_info("%s(): wakeout pulse off\n", __func__);
            /* Re-configure wakeout as input */
            stm32_configgpio(GPIO_APB1_WAKE_OUT | GPIO_INPUT);
            stm32_configgpio(GPIO_BB1_WAKE_OUT | GPIO_INPUT);
        }

        // Wait
        usleep(20000);

        x++;
    }
}
