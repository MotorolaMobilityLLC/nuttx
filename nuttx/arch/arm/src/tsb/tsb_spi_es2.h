/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#ifndef __ARCH_ARM_SRC_TSB_TSB_SPI_H
#define __ARCH_ARM_SRC_TSB_TSB_SPI_H

/* Registers offset */
#define TSB_SPIB_FLSH_MEM_MAP_0     0x0
#define TSB_SPIB_DIR_ACC_CTRL_0     0x8
#define TSB_SPIB_DIR_RD_CTRL_0      0x10

#define TSB_SPIB_PRG_ACC_CTRL_0     0x400
#define TSB_SPIB_PRG_ACC_CTRL_1     0x404
#define TSB_SPIB_PRG_ACC_INT_EN     0x408
#define TSB_SPIB_PRG_ACC_STAT       0x40c

#define TSB_SPIB0_PRG_PRI_DAT_BASE  0x500
#define TSB_SPIB0_PRG_PRI_DAT_SIZE  8

#define TSB_SPIB0_PRG_SEC_DAT_BASE  0x600
#define TSB_SPIB0_PRG_SEC_DAT_SIZE  256

/* TSB_SPIB_FLSH_MEM_MAP_0 */
#define TSB_SPIB_FLSH_MEM_MAP_0_FBA_SHIFT  (16)
#define TSB_SPIB_FLSH_MEM_MAP_0_FBA_MASK   (0xfff)

#define TSB_SPIB_FLSH_MEM_MAP_0_FDEN_SHIFT (2)
#define TSB_SPIB_FLSH_MEM_MAP_0_FDEN_MASK  (0xf)
#define TSB_SPIB_FLSH_MEM_MAP_0_FDEN_64KB  (0x0)
#define TSB_SPIB_FLSH_MEM_MAP_0_FDEN_16MB  (0x8)
#define TSB_SPIB_FLSH_MEM_MAP_0_FDEN_32MB  (0x9)
#define TSB_SPIB_FLSH_MEM_MAP_0_FDEN_64MB  (0xa)
#define TSB_SPIB_FLSH_MEM_MAP_0_FDEN_128MB (0xb)

#define TSB_SPIB_FLSH_MEM_MAP_0_RE         (1 << 0)

/* TSB_SPIB_DIR_ACC_CTRL_0 */
#define TSB_SPIB_DIR_ACC_CTRL_0_SCSD_SHIFT (8)
#define TSB_SPIB_DIR_ACC_CTRL_0_SCSD_MASK  (0xff)

#define TSB_SPIB_DIR_ACC_CTRL_0_POLL_WIP   (1 << 6)

#define TSB_SPIB_DIR_ACC_CTRL_0_SDCE_SHIFT (2)
#define TSB_SPIB_DIR_ACC_CTRL_0_SDCD_MASK  (0xf)
/* see TSB_SPIB_SDCE below */

#define TSB_SPIB_DIR_ACC_CTRL_0_SMOD_SHIFT (0)
#define TSB_SPIB_DIR_ACC_CTRL_0_SMOD_MASK  (0x3)
/* see TSB_SPIB_SMOD below */

/* TSB_SPIB_DIR_RD_CTRL_0 */
#define TSB_SPI_CMD_OP_SHIFT          (24)
#define TSB_SPI_CMD_OP_MASK           (0xff)

#define TSB_SPI_DMY_BYTE_CNT_SHIFT    (12)
#define TSB_SPI_DMY_BYTE_CNT_MASK     (0xf)

#define TSB_SPI_ADDR_BYTE_CNT_3_BYTES (0)
#define TSB_SPI_ADDR_BYTE_CNT_4_BYTES (1)

#define TSB_SPI_DATA_IO_CTRL_SHIFT    (6)
#define TSB_SPI_DATA_IO_CTRL_MASK     (0x3)
/* see TSB_SPI_IO_CTRL below */

#define TSB_SPI_DMY_IO_CTRL_SHIFT     (4)
#define TSB_SPI_DMY_IO_CTRL_MASK      (0x3)
/* see TSB_SPI_IO_CTRL below */

#define TSB_SPI_ADR_IO_CTRL_SHIFT     (2)
#define TSB_SPI_ADR_IO_CTRL_MASK      (0x3)
/* see TSB_SPI_IO_CTRL below */

/* TSB_SPIB_PRG_ACC_CTRL_0 */
#define TSB_SPIB_PRG_ACC_CTRL_0_SPR_SHIFT  (16)
#define TSB_SPIB_PRG_ACC_CTRL_0_SPR_MASK   (0x1f)

#define TSB_SPIB_PRG_ACC_CTRL_0_SCSD_SHIFT (8)
#define TSB_SPIB_PRG_ACC_CTRL_0_SCSD_MASK  (0xff)

#define TSB_SPIB_PRG_ACC_CTRL_0_SDCE_SHIFT (2)
#define TSB_SPIB_PRG_ACC_CTRL_0_SDCE_MASK  (0xf)
/* see TSB_SPIB_SDCE below */

#define TSB_SPIB_PRG_ACC_CTRL_0_SMOD_SHIFT (0)
#define TSB_SPIB_PRG_ACC_CTRL_0_SMOD_MASK  (0x3)
/* see TSB_SPIB_SMOD below */

/* TSB_SPIB_PRG_ACC_CTRL_1 */
#define TSB_SEC_BUF_DAT_BYTE_CNT_SHIFT (24)
#define TSB_SEC_BUF_DAT_BYTE_CNT_MASK  (0xFF)

#define TSB_PRI_BUF_DAT_BYTE_CNT_SHIFT (16)
#define TSB_PRI_BUF_DAT_BYTE_CNT_MASK  (0x7)

#define TSB_SEC_BUF_EN                 (1 << 5)
#define TSB_PRI_BUF_EN                 (1 << 4)
#define TSB_SPI_CYC_GO                 (1 << 0)

/* TSB_SPIB_PRG_ACC_INT_EN */
#define TSB_SPI_INT_EN (1 << 0)

/* TSB_SPIB_PRG_ACC_STAT */
#define TSB_SPI_CYC_IN_PRGRS (1 << 1)
#define TSB_SPI_CYC_DONE     (1 << 0)


/* TSB_SPIB_SMOD */
#define TSB_SPIB_SMOD_MODE0 (0x00)

/* TSB_SPIB_SDCE */
#define TSB_SPIB_SDCE_POS  (0x00)
#define TSB_SPIB_SDCE_NEG  (0x01)

/* TSB_SPI_IO_CTRL */
#define TSB_SPI_IO_CTRL_SINGLE     (0x00)
#define TSB_SPI_IO_CTRL_DUAL       (0x01)
#define TSB_SPI_IO_CTRL_QUAD       (0x10)

#endif /* __ARCH_ARM_SRC_TSB_TSB_SPI_H */
