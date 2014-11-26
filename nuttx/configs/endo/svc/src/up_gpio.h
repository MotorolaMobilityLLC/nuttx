/****************************************************************************
 * configs/endo/svc/src/up_gpio.h
 * Endo/SVC GPIO support
 *
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 ****************************************************************************/
#ifndef __CONFIGS_ENDO_INCLUDE_UP_GPIO_H
#define __CONFIGS_ENDO_INCLUDE_UP_GPIO_H

/* SVC IRQ semaphore */
extern sem_t svc_irq_sem;

/* Exported functions */
void gpio_init(void);
void gpio_clr_debug(void);
void gpio_set_debug(void);

void svc_irq_enable(void);
void svc_irq_disable(void);

#endif // __CONFIGS_ENDO_INCLUDE_UP_GPIO_H
