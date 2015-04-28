/*
 * Copyright (c) 2015 Google Inc.
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

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/pwm.h>
#include <nuttx/config.h>

#include <sys/types.h>

#include "up_arch.h"

#include "chip.h"
#include "tsb_pwm.h"
#include "tsb_scm.h"

/* This structure represents the state of one PWM */
struct tsb_pwm_s {
    const struct pwm_ops_s *ops;        /* PWM operations */
    uint32_t base;              /* The base address of the pwm */
    uint32_t pincfg;            /* Output pin configuration */
    uint32_t pclk;              /* The frequency of the pwm clock (after divider) */
#ifdef CONFIG_PWM_PULSECOUNT
    void *handle;               /* Handle used for upper-half callback */
#endif
};

static void tsb_pwm_write(struct tsb_pwm_s *dev, uint32_t addr, uint32_t v)
{
    putreg32(v, dev->base + addr);
}

static uint32_t tsb_pwm_read(struct tsb_pwm_s *dev, uint32_t addr)
{
    return getreg32(dev->base + addr);
}

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
    struct tsb_pwm_s *priv = (struct tsb_pwm_s *)dev;

    /* Enable Clock */
    tsb_clk_enable(TSB_CLK_PWMODP);
    tsb_clk_enable(TSB_CLK_PWMODS);

    /* Reset */
    tsb_reset(TSB_RST_PWMODP);
    tsb_reset(TSB_RST_PWMODS);

    /* Set LVS to 1 when freq = 0 and DIV = 16 */
    tsb_pwm_write(priv, TSB_PWM_CR, PWM_CR_DIV(TSB_PWM_DIV));

    return OK;
}

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
    tsb_clk_disable(TSB_CLK_PWMODP);
    tsb_clk_disable(TSB_CLK_PWMODS);

    return OK;
}

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info, void *handle)
#else
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
#endif
{
    uint32_t cr;
    uint32_t freq;
    uint32_t duty;
    struct tsb_pwm_s *priv = (struct tsb_pwm_s *)dev;

    if (info->frequency >= priv->pclk)
        return -EINVAL;

    freq = priv->pclk / info->frequency;
    if (freq == 1)
        return -EINVAL;
    duty = freq * info->duty / 0xffff;

    if (freq)
        DEBUGASSERT(duty && freq > duty);

    lldbg("freq = %d, duty = %d\n", freq, duty);

    cr = tsb_pwm_read(priv, TSB_PWM_CR) & ~PWM_CR_ENB;
    tsb_pwm_write(priv, TSB_PWM_CR, cr);        //stop pwm

    tsb_pwm_write(priv, TSB_PWM_FREQ, freq);
    tsb_pwm_write(priv, TSB_PWM_DUTY, duty);

#ifdef CONFIG_PWM_PULSECOUNT
    /* Check if a pulsecount has been selected */
    if (info->count > 0) {
        tsb_pwm_write(priv, TSB_PWM_ITERATION, info->count);
    }
    /* Save the handle */
    priv->handle = handle;
#else
    tsb_pwm_write(priv, TSB_PWM_ITERATION, 0);
#endif

    /* Start the time */
    tsb_pwm_write(priv, TSB_PWM_CR, cr | PWM_CR_UPD | PWM_CR_ENB);

    return OK;
}

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
    uint32_t cr;
    struct tsb_pwm_s *priv = (struct tsb_pwm_s *)dev;

    cr = tsb_pwm_read(priv, TSB_PWM_CR);
    tsb_pwm_write(priv, TSB_PWM_CR, cr & ~PWM_CR_ENB);  //stop pwm

    return OK;
}

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg)
{
    return -ENOTTY;
}

static const struct pwm_ops_s g_pwmops = {
    .setup = pwm_setup,
    .shutdown = pwm_shutdown,
    .start = pwm_start,
    .stop = pwm_stop,
    .ioctl = pwm_ioctl,
};

static struct tsb_pwm_s g_pwm1dev = {
    .ops = &g_pwmops,
    .base = TSB_PWM0,
    .pclk = TSB_PWM_CLK,
};

static struct tsb_pwm_s g_pwm2dev = {
    .ops = &g_pwmops,
    .base = TSB_PWM1,
    .pclk = TSB_PWM_CLK,
};

/**
 * @brief Initialize one PWM for use with the upper_level PWM driver.
 * @param pwm A number identifying the PWM to use.
 * @return On success, a pointer to the TSB lower half PWM driver is returned.
 *         NULL is returned on any failure.
 *
 */
struct pwm_lowerhalf_s *tsb_pwminitialize(int pwm)
{
    struct tsb_pwm_s *lower;

    if (pwm == 0)
        lower = &g_pwm1dev;
    else if (pwm == 1)
        lower = &g_pwm2dev;
    else
        return NULL;
    return (struct pwm_lowerhalf_s *)lower;
}
