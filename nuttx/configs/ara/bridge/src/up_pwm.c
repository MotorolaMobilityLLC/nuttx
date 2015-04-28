/**
 * Copyright (c) 2014-2015 Google Inc.
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

#include <nuttx/config.h>
#include <arch/tsb/pwm.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/pwm.h>

#ifdef CONFIG_PWM

/*
 * Caution:
 *   For TSB, PWM1 is pinshared with GPIO9, and their use is therefore exclusive.
 *   Applications intending to control PWM1 must ensure that the PINSHARE register is
 *   configured appropriately.
 */
int pwm_devinit(void)
{
    static int pwm0_initialized = 0;
    static int pwm1_initialized = 0;
    struct pwm_lowerhalf_s *pwm;
    int ret;

    /* Have we already initialized PWM0? */
    if (!pwm0_initialized) {
        /* Call tsb_pwminitialize() to get an instance of the PWM0 interface */
        pwm = tsb_pwminitialize(0);
        if (!pwm) {
            adbg("Failed to get the TSB PWM0 lower half\n");
            return -ENODEV;
        }

        /* Register the PWM0 driver at "/dev/pwm0" */

        ret = pwm_register("/dev/pwm0", pwm);
        if (ret < 0) {
            adbg("pwm_register for /dev/pwm0 failed: %d\n", ret);
            return ret;
        }

        /* Now PWM0 is initialized */
        pwm0_initialized = 1;
    }

    /* Have we already initialized PWM1? */
    if (!pwm1_initialized) {
        /* Call tsb_pwminitialize() to get an instance of the PWM1 interface */
        pwm = tsb_pwminitialize(1);
        if (!pwm) {
            adbg("Failed to get the TSB PWM1 lower half\n");
            return -ENODEV;
        }

        /* Register the PWM1 driver at "/dev/pwm1" */
        ret = pwm_register("/dev/pwm1", pwm);
        if (ret < 0) {
            adbg("pwm_register for /dev/pwm1 failed: %d\n", ret);
            return ret;
        }

        /* Now PWM0 is initialized */
        pwm1_initialized = 1;
    }

    return 0;
}

#endif /* CONFIG_PWM */
