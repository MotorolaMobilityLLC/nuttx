/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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

#ifndef __INCLUDE_NUTTX_POWER_PAD_DET_H
#define __INCLUDE_NUTTX_POWER_PAD_DET_H

#include <stdbool.h>

/* The type of the pad detect callback function */
typedef void (*pad_detect_callback_t)(void *arg, bool docked);

/****************************************************************************
 * Name: pad_det_register_callback
 *
 * Description:
 *   Register for notifications in docked state changes.
 *
 * Input Parameters:
 *   callback - Function to call when docked state changes.
 *   arg      - Parameter to pass to callback function.
 *
 * Returned Value:
 *   0 on success or negative errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_PAD_DETECT
int pad_det_register_callback(pad_detect_callback_t callback, void *arg);
#else
static inline int pad_det_register_callback(pad_detect_callback_t callback, void *arg)
{
    (void)callback;
    (void)arg;
    return -ENODEV;
}
#endif

#endif /* __INCLUDE_NUTTX_POWER_PAD_DET_H */
