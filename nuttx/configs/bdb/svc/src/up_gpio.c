/*
 * Copyright (c) 2014 Google Inc.
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

/****************************************************************************
 * configs/bdb/svc/src/up_gpio.c
 * BDB/SVC GPIO support
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

#include "bdb-internal.h"

/* SVC IRQ handler */
static int svc_irq_handler(int irq, void *context);
/* SVC IRQ semaphore */
sem_t svc_irq_sem;


/* Init GPIO control lines */
void gpio_init(void)
{
    dbg_info("%s()\n", __func__);

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

    /* Assert the Springs wakeout lines */
    stm32_configgpio(GPIO_BB1_WAKE_OUT | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    stm32_configgpio(GPIO_BB2_WAKE_OUT | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    stm32_configgpio(GPIO_BB3_WAKE_OUT | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    stm32_configgpio(GPIO_BB4_WAKE_OUT | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    stm32_configgpio(GPIO_BB5_WAKE_OUT | GPIO_OUTPUT | GPIO_OUTPUT_SET);
    stm32_configgpio(GPIO_BB6_WAKE_OUT | GPIO_OUTPUT | GPIO_OUTPUT_SET);
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
    dbg_verbose("%s()\n", __func__);
    stm32_gpiowrite(GPIO_DBG_1, false);
}

/* Set DBG_1 debug line */
void gpio_set_debug(void)
{
    dbg_verbose("%s()\n", __func__);
    stm32_gpiowrite(GPIO_DBG_1, true);
}

/* Dump state of the DBG_* debug lines */
void stm32_gpio_dump(void)
{
    dbg_verbose("%s()\n", __func__);
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
