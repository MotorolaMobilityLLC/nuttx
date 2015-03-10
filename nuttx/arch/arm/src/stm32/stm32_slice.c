/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#include <arch/board/board.h>
#include <arch/stm32/slice.h>

int slice_init(void)
{
    return stm32_configgpio(GPIO_SLICE_CORE_INT);
}

int slice_uninit(void)
{
    return stm32_unconfiggpio(GPIO_SLICE_CORE_INT);
}

void slice_host_int_set(bool value)
{
    stm32_gpiowrite(GPIO_SLICE_CORE_INT, value);
}

