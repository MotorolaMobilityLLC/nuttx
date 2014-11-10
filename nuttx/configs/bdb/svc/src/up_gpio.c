 /****************************************************************************
  * configs/bdb/svc/src/up_gpio.c
  * BDB/SVC GPIO support
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

#include <nuttx/i2c.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"

#include "bdb-internal.h"

/* SVC IRQ handler */
static int svc_irq_handler(int irq, void *context);
/* SVC IRQ semaphore */
sem_t svc_irq_sem;


/* Init GPIO control lines */
void gpio_init(void)
{
    printk("%s()\n", __func__);
    /* Configure GPIOs for output */
    /*  Debug pins to J53 */
    stm32_configgpio(GPIO_DBG_1);
    stm32_configgpio(GPIO_DBG_2);
    stm32_configgpio(GPIO_DBG_3);
    stm32_configgpio(GPIO_DBG_4);
    /* Enable LEDs control: SVC_LED_EN */
    stm32_configgpio(GPIO_LED_EN);
    stm32_gpiowrite(GPIO_LED_EN, true);
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

/* Clear DBG_1 debug line */
void gpio_clr_debug(void)
{
    printk("%s()\n", __func__);
    stm32_gpiowrite(GPIO_DBG_1, false);
}

/* Set DBG_1 debug line */
void gpio_set_debug(void)
{
    printk("%s()\n", __func__);
    stm32_gpiowrite(GPIO_DBG_1, true);
}

/* Dump state of the DBG_* debug lines */
void stm32_gpio_dump(void)
{
    printk("%s()\n", __func__);
    stm32_dumpgpio(GPIO_DBG_1, "dumpgpio");
    stm32_dumpgpio(GPIO_DBG_2, "dumpgpio");
    stm32_dumpgpio(GPIO_DBG_3, "dumpgpio");
    stm32_dumpgpio(GPIO_DBG_4, "dumpgpio");
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
