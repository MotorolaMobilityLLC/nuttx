/*
 * Copyright (C) 2014 Google
 * Author: Benoit Cousson <bcousson@baylibre.com>
 */

#include <sys/types.h>
#include <nuttx/config.h>
#include <nuttx/serial/uart_16550.h>

#include "up_arch.h"


#ifdef CONFIG_16550_UART

uart_datawidth_t uart_getreg(uart_addrwidth_t base, unsigned int offset)
{
	return getreg32(base + offset);
}

void uart_putreg(uart_addrwidth_t base, unsigned int offset, uart_datawidth_t value)
{
	putreg32(value, base + offset);
}

#endif
