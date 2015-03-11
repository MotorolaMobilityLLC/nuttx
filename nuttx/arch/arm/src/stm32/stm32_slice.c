/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#include <arch/board/board.h>
#include <arch/stm32/slice.h>

int slice_init(xcpt_t irqhandler)
{
    int ret = stm32_configgpio(GPIO_SLICE_CORE_INT);

    if (ret == OK)
      {
        // The gpio is configured inside stm32_gpiosetevent
        stm32_gpiosetevent(GPIO_SLICE_BASE_DET_N, true, true, true, irqhandler);
      }

    return ret;
}

int slice_uninit(void)
{
    int ret = stm32_unconfiggpio(GPIO_SLICE_BASE_DET_N);
    if (ret != OK)
        return ret;

    return stm32_unconfiggpio(GPIO_SLICE_CORE_INT);
}

void slice_host_int_set(bool value)
{
    stm32_gpiowrite(GPIO_SLICE_CORE_INT, value);
}

bool slice_base_det_read(void)
{
    return stm32_gpioread(GPIO_SLICE_BASE_DET_N);
}

