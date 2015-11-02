/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef _STM32_MODS_H_
#define _STM32_MODS_H_

#include <stdbool.h>
#include <nuttx/gpio.h>

#define BOARD_REVISION           (CONFIG_ARCH_BOARDID_PID & 0x0000FFFF)

#define CALC_GPIO_NUM(port, pin)  ((16 * (port - 'A')) + pin)

#define GPIO_MODS_INT            CALC_GPIO_NUM('A', 11)
#define GPIO_MODS_SL_BPLUS_EN    CALC_GPIO_NUM('A', 12)
#define GPIO_MODS_RFR            CALC_GPIO_NUM('B',  0)
#define GPIO_MODS_WAKE_N         CALC_GPIO_NUM('B',  1)
#define GPIO_MODS_CHG_PG_N       CALC_GPIO_NUM('C',  2)

#if (BOARD_REVISION >= 1)
#  define GPIO_MODS_SPI_CS_N     CALC_GPIO_NUM('B', 12)
#endif

static inline void mods_rfr_init(void)
{
    /* On this board, RFR is active high */
    gpio_direction_out(GPIO_MODS_RFR, 0);
}

static inline void mods_rfr_set(uint8_t value)
{
    /* On this board, RFR is active high */
    gpio_set_value(GPIO_MODS_RFR, value);
}

static inline uint8_t mods_rfr_get(void)
{
    /* On this board, RFR is active high */
    return gpio_get_value(GPIO_MODS_RFR);
}

void mods_host_int_set(bool value);

#endif /* _STM32_MODS_H_ */
