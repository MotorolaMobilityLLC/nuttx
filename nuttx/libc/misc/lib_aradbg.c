/*
 * Copyright (c) 2014, 2015 Google Inc.
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

#define ARADBG_COMP ARADBG_DBG    /* ARADBG_COMP macro of the component */
#include <ara_debug.h>

dbg_ctrl_t dbg_ctrl = { ARADBG_ALL, ARADBG_INFO };

/* Get the level and components to enable debug for */
void dbg_get_config(uint32_t *comp, uint32_t *level)
{
    *comp = dbg_ctrl.comp;
    *level = dbg_ctrl.lvl;

    dbg_info("%s(): debug comp=0x%x, level=%d\n", __func__,
           dbg_ctrl.comp, dbg_ctrl.lvl);
}

/* Configure the level and components to enable debug for */
int dbg_set_config(uint32_t comp, uint32_t level)
{
    /* ARADBG_MAX is always enabled */
    if (level > ARADBG_MAX)
        level = ARADBG_MAX;

    dbg_ctrl.comp = comp;
    dbg_ctrl.lvl = level;

    dbg_info("%s(): debug comp=0x%x, level=%d\n", __func__,
           dbg_ctrl.comp, dbg_ctrl.lvl);

    return 0;
}
