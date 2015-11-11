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
 *
 * @author Kim Mui
 * @brief Define macros to generate PL330 instructions.
 */

#ifndef __ARCH_ARM_SRC_TSB_DMA_PL330_ASM_H
#define __ARCH_ARM_SRC_TSB_DMA_PL330_ASM_H

/* Define values in PL320 instructions */
/* Define values in PL320 instructions */

/* Define values in PL320 instructions */
#define DMA_UINT32(value)           \
    (value & 0xFF), ((value >> 8) & 0xFF), ((value >> 16) & 0xFF), \
    ((value >> 24) & 0xFF)
#define DMAADD_sar                  0x00
#define DMAADD_dar                  0x20
#define DMAGO_s                     0x00
#define DMAGO_ns                    0x02
#define DMALP_lc0                   0x00
#define DMALP_lc1                   0x02
#define DMALPEND_lc0                0x10
#define DMALPEND_lc1                0x14
#define DMALPEND_FE                 0x00
#define DMALPENDT_lc0               0x00
#define DMALPENDT_lc1               0x04
#define DMAMOV_sar                  0x00
#define DMAMOV_dar                  0x02
#define DMAMOV_ccr                  0x01
#define DMAWFE_                     0x00
#define DMAWFE_invalid              0x02
#define DMAWFP_S                    0x00
#define DMAWFP_B                    0x02
#define DMAWFP_P                    0x01

/* define MACROs for PL330 instruction. */
#define DMAADDH(reg, value)         (0x54 | DMAADD_##reg), DMA_UINT32(value)
#define DMAADDNH(reg, value)        (0x5C | DMAADD_##reg), DMA_UINT32(value)
#define DMAEND                      (0x00)
#define DMAFLUSH(peripheral)        (0x35), (peripheral << 3)
#define DMAGO(ch, secure, addr)     \
    (0xA0 | DMAGO_##secure), ch, DMA_UINT32((uint32_t)addr)
#define DMAKILL                     (0x01)
#define DMALD                       (0x04)
#define DMALDS                      (0x05)
#define DMALDB                      (0x07)
#define DMALDPS(peripheral)         (0x25), (peripheral << 3)
#define DMALDPB(peripheral)         (0x27), (peripheral << 3)
#define DMALP(lc, iter)             (0x20 | DMALP_##lc), iter
#define DMALPEND(lc, value)         (0x28 | DMALPEND_##lc), value
#define DMALPENDS(lc, value)        (0x39 | DMALPENDT_##lc), value
#define DMALPENDB(lc, value)        (0x3B | DMALPENDT_##lc), value
#define DMALPFE
#define DMAMOV(reg, value)          \
    (0xBC), (DMAMOV_##reg), DMA_UINT32((uint32_t)value)
#define DMANOP                      (0x18)
#define DMARMB                      (0x12)
#define DMASEV(event)               (0x34), (event << 3)
#define DMAST                       (0x08)
#define DMASTS                      (0x09)
#define DMASTB                      (0x0B)
#define DMASTPS(peripheral)         (0x29), (peripheral << 3)
#define DMASTPB(peripheral)         (0x2B), (peripheral << 3)
#define DMASTZ                      (0x0C)
#define DMAWFE(event,inval_i_cache) \
    (0x36), (DMAWFE_##inval_i_cache | event << 3)
#define DMAWFP(peripheral, type)    (0x30 | DMAWFP_##type), (peripheral << 3)
#define DMAWMB                      (0x13)

/* Define size of PL330 instructions. We need them to calculate the offset to
 * modify the values in the PL330 programs.
 */
#define DMA_ADD_SIZE                3
#define DMA_END_SIZE                1
#define DMA_FLUSHP_SIZE             2
#define DMA_GO_SIZE                 6
#define DMA_KILL_SIZE               1
#define DMA_LD_SIZE                 1
#define DMA_LDP_SIZE                2
#define DMA_LP_SIZE                 2
#define DMA_LPEND_SIZE              2
#define DMA_MOV_SIZE                6
#define DMA_NOP_SIZE                1
#define DMA_RMB_SIZE                1
#define DMA_SEV_SIZE                2
#define DMA_ST_SIZE                 1
#define DMA_STP_SIZE                2
#define DMA_STZ_SIZE                1
#define DMA_WFE_SIZE                2
#define DMA_WFP_SIZE                2
#define DMA_WMB_SIZE                1

#endif /* __ARCH_ARM_SRC_TSB_DMA_PL330_ASM_H */
