/****************************************************************************
 * configs/bdb/svc/src/up_switch.c
 * BDB SVC support for the Unipro switch
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

#undef lldbg
#define lldbg printk


int switch_control(int state)
{
	printk("%s(): state %d\n", __func__, state);

	return 0;
}
