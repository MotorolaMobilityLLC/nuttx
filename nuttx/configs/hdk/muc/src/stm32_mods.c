/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#include <arch/board/board.h>
#include <arch/board/mods.h>
#include <nuttx/power/bq24292.h>

#include <unistd.h>

void mods_init(void)
{
    stm32_configgpio(GPIO_MODS_CORE_INT);
}

#ifdef CONFIG_MODS_USER_INIT
// This initialization call is performed when mods app starts
int mods_user_init(void)
{
    int ret = 0;
#if defined(CONFIG_CHARGER_BQ24292)
    ret = bq24292_driver_init(GPIO_MODS_CHG_PG_N);
    if (ret)
        return ret;
#endif

    return ret;
}
#endif

void mods_host_int_set(bool value)
{
    stm32_gpiowrite(GPIO_MODS_CORE_INT, value);
}

void mods_vbus_en_sw(bool value)
{
#if defined(CONFIG_CHARGER_BQ24292)
    if (value)
        bq24292_set_chg(BQ24292_OTG_1300MA);
    else
        bq24292_set_chg(BQ24292_CHG_BATTERY);
#endif
}
