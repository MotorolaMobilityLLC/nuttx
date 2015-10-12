/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#include <arch/board/board.h>
#include <arch/stm32/slice.h>

#include <unistd.h>

int slice_init(xcpt_t irqhandler)
{
    int ret = stm32_configgpio(GPIO_SLICE_CORE_INT);
    if (ret != OK)
        return ret;

    ret = stm32_configgpio(GPIO_SLICE_SL_VBUS_EN_SW);
    if (ret != OK)
        goto vbus_en_sw_failed;

    ret = stm32_configgpio(GPIO_SLICE_CHG_CE_N);
    if (ret != OK)
        goto chg_ce_n_failed;

    ret = stm32_configgpio(GPIO_SLICE_VIB_EN);
    if (ret != OK)
        goto vib_en_failed;

    // The gpio is configured inside stm32_gpiosetevent
    stm32_gpiosetevent(GPIO_SLICE_SL_BPLUS_EN, true, true, true, irqhandler);

    return OK;

vib_en_failed:
    stm32_unconfiggpio(GPIO_SLICE_CHG_CE_N);
chg_ce_n_failed:
    stm32_unconfiggpio(GPIO_SLICE_SL_VBUS_EN_SW);
vbus_en_sw_failed:
    stm32_unconfiggpio(GPIO_SLICE_CORE_INT);
    return ret;
}

int slice_uninit(void)
{
    int ret = stm32_unconfiggpio(GPIO_SLICE_SL_BPLUS_EN);
    if (ret != OK)
        return ret;

    ret = stm32_unconfiggpio(GPIO_SLICE_VIB_EN);
    if (ret != OK)
        return ret;

    ret = stm32_unconfiggpio(GPIO_SLICE_SL_VBUS_EN_SW);
    if (ret != OK)
        return ret;

    ret = stm32_unconfiggpio(GPIO_SLICE_CHG_CE_N);
    if (ret != OK)
        return ret;

    return stm32_unconfiggpio(GPIO_SLICE_CORE_INT);
}

void slice_host_int_set(bool value)
{
    stm32_gpiowrite(GPIO_SLICE_CORE_INT, value);
}

/* Has a base (phone) been attached */
bool slice_is_base_present(void)
{
    return stm32_gpioread(GPIO_SLICE_SL_BPLUS_EN);
}

void slice_vbus_en_sw(bool value)
{
    // Enable disable slice charging based on whether slice is charging base
    if (value)
        stm32_gpiowrite(GPIO_SLICE_CHG_CE_N, value);

    stm32_gpiowrite(GPIO_SLICE_SL_VBUS_EN_SW, value);

    if (!value)
      {
        // To avoid brief moment of charging (and therefore a brief flash of
        // the green LED, wait before re-enabling slice charging.
        usleep(100000);
        stm32_gpiowrite(GPIO_SLICE_CHG_CE_N, value);
      }
}

