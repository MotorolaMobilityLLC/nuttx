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

#include <debug.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>

#include <nuttx/unipro/unipro.h>

#include "tsb_scm.h"

int up_getchipid(uint32_t *vid, uint32_t *pid)
{
    if (vid) {
        uint32_t result_code = 0;
        uint32_t val = 0;
        result_code = unipro_attr_peer_read(DME_DDBL1_MANUFACTURERID, &val, 0 /* selector */);
        if (!result_code) {
            *vid = val;
        }
    }

    if (pid) {
        uint32_t result_code = 0;
        uint32_t val = 0;
        result_code = unipro_attr_peer_read(DME_DDBL1_PRODUCTID, &val, 0 /* selector */);
        if (!result_code) {
            *pid = val;
        }
    }

    return OK;
}
