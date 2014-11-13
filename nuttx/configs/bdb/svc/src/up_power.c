/****************************************************************************
 * configs/bdb/svc/src/up_power.c
 * BDB/SVC power support
 *
 * Copyright (C) 2014 Google, Inc.
 *
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"

#include "bdb-internal.h"

#undef lldbg
#define lldbg printk


/* Init APB1 power control */
void bdb_apb1_init(void)
{
    printk("%s()\n", __func__);

    stm32_configgpio(GPIO_VAPB1_1P1_EN);
    stm32_configgpio(GPIO_VAPB1_1P2_EN);
    stm32_configgpio(GPIO_VAPB1_1P8_EN);
}

/* Enable APB1 power */
void bdb_apb1_enable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB1_1P1_EN, true);
    stm32_gpiowrite(GPIO_VAPB1_1P2_EN, true);
    stm32_gpiowrite(GPIO_VAPB1_1P8_EN, true);
}

/* Disable APB1 power */
void bdb_apb1_disable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB1_1P1_EN, false);
    stm32_gpiowrite(GPIO_VAPB1_1P2_EN, false);
    stm32_gpiowrite(GPIO_VAPB1_1P8_EN, false);
}

/* Init APB2 power control */
void bdb_apb2_init(void)
{
    printk("%s()\n", __func__);

    stm32_configgpio(GPIO_VAPB2_1P1_EN);
    stm32_configgpio(GPIO_VAPB2_1P2_EN);
    stm32_configgpio(GPIO_VAPB2_1P8_EN);
}

/* Enable APB2 power */
void bdb_apb2_enable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB2_1P1_EN, true);
    stm32_gpiowrite(GPIO_VAPB2_1P2_EN, true);
    stm32_gpiowrite(GPIO_VAPB2_1P8_EN, true);
}

/* Disable APB2 power */
void bdb_apb2_disable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB2_1P1_EN, false);
    stm32_gpiowrite(GPIO_VAPB2_1P2_EN, false);
    stm32_gpiowrite(GPIO_VAPB2_1P8_EN, false);
}

/* Init APB3 power control */
void bdb_apb3_init(void)
{
    printk("%s()\n", __func__);

    stm32_configgpio(GPIO_VAPB3_1P1_EN);
    stm32_configgpio(GPIO_VAPB3_1P2_EN);
    stm32_configgpio(GPIO_VAPB3_1P8_EN);
}

/* Enable APB3 power */
void bdb_apb3_enable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB3_1P1_EN, true);
    stm32_gpiowrite(GPIO_VAPB3_1P2_EN, true);
    stm32_gpiowrite(GPIO_VAPB3_1P8_EN, true);
}

/* Disable APB3 power */
void bdb_apb3_disable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VAPB3_1P1_EN, false);
    stm32_gpiowrite(GPIO_VAPB3_1P2_EN, false);
    stm32_gpiowrite(GPIO_VAPB3_1P8_EN, false);
}

/* Init GPB1 power control */
void bdb_gpb1_init(void)
{
    printk("%s()\n", __func__);

    stm32_configgpio(GPIO_VGPB1_1P1_EN);
    stm32_configgpio(GPIO_VGPB1_1P2_EN);
    stm32_configgpio(GPIO_VGPB1_1P8_EN);
    stm32_configgpio(GPIO_VGPB1_SDIO_EN);
}

/* Enable GPB1 power */
void bdb_gpb1_enable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VGPB1_1P1_EN, true);
    stm32_gpiowrite(GPIO_VGPB1_1P2_EN, true);
    stm32_gpiowrite(GPIO_VGPB1_1P8_EN, true);
    stm32_gpiowrite(GPIO_VGPB1_SDIO_EN, true);
}

/* Disable GPB1 power */
void bdb_gpb1_disable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VGPB1_1P1_EN, false);
    stm32_gpiowrite(GPIO_VGPB1_1P2_EN, false);
    stm32_gpiowrite(GPIO_VGPB1_1P8_EN, false);
    stm32_gpiowrite(GPIO_VGPB1_SDIO_EN, false);
}

/* Init GPB2 power control */
void bdb_gpb2_init(void)
{
    printk("%s()\n", __func__);

    /*  Power control to the GPB2 */
    stm32_configgpio(GPIO_VGPB2_1P1_EN);
    stm32_configgpio(GPIO_VGPB2_1P2_EN);
    stm32_configgpio(GPIO_VGPB2_1P8_EN);
    stm32_configgpio(GPIO_VGPB2_SDIO_EN);
}

/* Enable GPB2 power */
void bdb_gpb2_enable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VGPB2_1P1_EN, true);
    stm32_gpiowrite(GPIO_VGPB2_1P2_EN, true);
    stm32_gpiowrite(GPIO_VGPB2_1P8_EN, true);
    stm32_gpiowrite(GPIO_VGPB2_SDIO_EN, true);
}

/* Disable APB2 power */
void bdb_gpb2_disable(void)
{
    printk("%s()\n", __func__);

    stm32_gpiowrite(GPIO_VGPB2_1P1_EN, false);
    stm32_gpiowrite(GPIO_VGPB2_1P2_EN, false);
    stm32_gpiowrite(GPIO_VGPB2_1P8_EN, false);
    stm32_gpiowrite(GPIO_VGPB2_SDIO_EN, false);
}
