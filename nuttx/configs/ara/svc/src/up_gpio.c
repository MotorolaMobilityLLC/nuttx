/****************************************************************************
 * configs/endo/svc/src/up_gpio.c
 * Endo/SVC GPIO support
 *
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 ****************************************************************************/
#define DBG_COMP DBG_GPIO
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/i2c.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_debug.h"
#include "up_internal.h"
#include "stm32.h"

/* SVC IRQ handler */
static int svc_irq_handler(int irq, void *context);
/* SVC IRQ semaphore */
sem_t svc_irq_sem;


/* Init GPIO control lines */
void gpio_init(void)
{
    dbg_info("%s()\n", __func__);

    /* Configure GPIOs for output */
    /* Enable RGB LED control */
    stm32_configgpio(GPIO_R_LED_EN);
    stm32_gpiowrite(GPIO_R_LED_EN, true);
    stm32_configgpio(GPIO_G_LED_EN);
    stm32_gpiowrite(GPIO_G_LED_EN, true);
    stm32_configgpio(GPIO_B_LED_EN);
    stm32_gpiowrite(GPIO_B_LED_EN, true);
    /* Switch reset line */
    stm32_configgpio(GPIO_SW_RST_40uS);
}

void svc_irq_enable(void)
{
    /* Init semaphore */
    sem_init(&svc_irq_sem, 0, 1);
    /* Configure switch and expanders IRQ line: falling edge */
    stm32_configgpio(GPIO_SVC_IRQ);
    stm32_gpiosetevent(GPIO_SVC_IRQ, false, true, true, svc_irq_handler);
}

void svc_irq_disable(void)
{
    stm32_gpiosetevent(GPIO_SVC_IRQ, false, true, true, NULL);
}

static int svc_irq_handler(int irq, void *context)
{
    /*
     * The I2C code cannot be called from the IRQ handler (since it uses
     * semaphores.
     * Let the main loop handle the I2C reads and do the handling.
     */
    sem_post(&svc_irq_sem);

    return OK;
}
