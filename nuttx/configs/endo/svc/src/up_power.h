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
 * configs/endo/svc/src/up_power.h
 * Endo/SVC power support
 *
 ****************************************************************************/
#ifndef __CONFIGS_ENDO_INCLUDE_UP_POWER_H
#define __CONFIGS_ENDO_INCLUDE_UP_POWER_H


/* Interface block number for power and WAKEOUT control */
enum {
    PWR_SPRING_A,
    PWR_SPRING_B,
    PWR_SPRING_C,
    PWR_SPRING_D,
    PWR_SPRING_E,
    PWR_SPRING_F,
    PWR_SPRING_G,
    PWR_SPRING_H,
    PWR_SPRING_I,
    PWR_SPRING_J,
    PWR_SPRING_K,
    PWR_SPRING_L,
    PWR_SPRING_M,
    PWR_SPRING_N,
    PWR_SPRING_NR,      /* Number of interfaces */
};

#define WAKEOUT_PULSE_DURATION  100000

void power_enable_internal(void);
void power_cycle_switch(void);
void power_interface_block_init(void);
int power_set_power(uint32_t int_nr, bool enable);
bool power_get_power(uint32_t int_nr);
void power_set_wakeout(uint32_t int_mask, bool assert);

void power_read_wake_detect(void);
#endif // __CONFIGS_ENDO_INCLUDE_UP_POWER_H
