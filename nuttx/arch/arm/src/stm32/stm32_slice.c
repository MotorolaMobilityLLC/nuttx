/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#include <arch/board/board.h>
#include <arch/stm32/slice.h>

void slice_init(void)
{
    stm32_configgpio(GPIO_SLICE_CORE_INT);
}

void slice_uninit(void)
{
    stm32_unconfiggpio(GPIO_SLICE_CORE_INT);
}

void slice_host_int_set(bool value)
{
    // Interrupt is active low, so reverse polarity
    stm32_gpiowrite(GPIO_SLICE_CORE_INT, !value);
}

