/*
 * Copyright (c) 2014-2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

#ifndef __ARCH_ARM_INCLUDE_TSB_CHIP_H
#define __ARCH_ARM_INCLUDE_TSB_CHIP_H

/* No peripheral interrupts enabled at the moment */
#define ARMV7M_PERIPHERAL_INTERRUPTS 126

/* Buffer RAM */

#define BUFRAM_BANK_SIZE 0xC000

#define BUFRAM0_BASE    0x20000000
#define BUFRAM1_BASE    (BUFRAM0_BASE + BUFRAM_BANK_SIZE)
#define BUFRAM2_BASE    (BUFRAM1_BASE + BUFRAM_BANK_SIZE)
#define BUFRAM3_BASE    (BUFRAM2_BASE + BUFRAM_BANK_SIZE)

#define BUFRAM_BASE         BUFRAM0_BASE
#define BUFRAM_BANK_COUNT   4
#define BUFRAM_SIZE         (BUFRAM_BANK_SIZE * BUFRAM_BANK_COUNT)

#endif  /* __ARCH_ARM_INCLUDE_TSB_CHIP_H */
