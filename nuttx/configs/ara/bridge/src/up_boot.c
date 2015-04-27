/**
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

#include <nuttx/config.h>
#include <arch/tsb/unipro.h>
#include <debug.h>

#ifdef CONFIG_ARCH_LOWPUTC
#define early_dbg(fmt, ...) lowsyslog(fmt, ##__VA_ARGS__)
#endif

#ifndef ARA_FW_VERSION
#define ARA_FW_VERSION "unknown"
#endif

#ifndef ARA_FW_BUILD_TIME
#define ARA_FW_BUILD_TIME "unknown"
#endif

void tsb_boardinitialize(void) {
    early_dbg("\n********************************************************************************\n");
    early_dbg("Ara Bridge FW version: %s\n", ARA_FW_VERSION);
    early_dbg("Build: %s\n", ARA_FW_BUILD_TIME);
    early_dbg("Chip Revision: %s\n", CONFIG_TSB_CHIP_REV);

#if defined CONFIG_BOOT_COPYTORAM
    early_dbg("Boot type: SPI flash copy to RAM\n");
#elif defined CONFIG_BOOT_RUNFROMISRAM
    early_dbg("Boot type: Internal SRAM\n");
#else
#error "Unsupported configuration"

#endif
    early_dbg("********************************************************************************\n");
}

