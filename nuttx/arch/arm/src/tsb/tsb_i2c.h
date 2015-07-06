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

#ifndef __ARCH_ARM_SRC_TSB_TSB_I2C_H
#define __ARCH_ARM_SRC_TSB_TSB_I2C_H

/* Registers offset */
#define TSB_I2C_CON               0x0
#define TSB_I2C_TAR               0x4
#define TSB_I2C_DATA_CMD          0x10
#define TSB_I2C_SS_SCL_HCNT       0x14
#define TSB_I2C_SS_SCL_LCNT       0x18
#define TSB_I2C_FS_SCL_HCNT       0x1c
#define TSB_I2C_FS_SCL_LCNT       0x20
#define TSB_I2C_INTR_STAT         0x2c
#define TSB_I2C_INTR_MASK         0x30
#define TSB_I2C_RAW_INTR_STAT     0x34
#define TSB_I2C_RX_TL             0x38
#define TSB_I2C_TX_TL             0x3c
#define TSB_I2C_CLR_INTR          0x40
#define TSB_I2C_CLR_RX_UNDER      0x44
#define TSB_I2C_CLR_RX_OVER       0x48
#define TSB_I2C_CLR_TX_OVER       0x4c
#define TSB_I2C_CLR_RD_REQ        0x50
#define TSB_I2C_CLR_TX_ABRT       0x54
#define TSB_I2C_CLR_RX_DONE       0x58
#define TSB_I2C_CLR_ACTIVITY      0x5c
#define TSB_I2C_CLR_STOP_DET      0x60
#define TSB_I2C_CLR_START_DET     0x64
#define TSB_I2C_CLR_GEN_CALL      0x68
#define TSB_I2C_ENABLE            0x6c
#define TSB_I2C_STATUS            0x70
#define TSB_I2C_TXFLR             0x74
#define TSB_I2C_RXFLR             0x78
#define TSB_I2C_SDA_HOLD          0x7c
#define TSB_I2C_TX_ABRT_SOURCE    0x80
#define TSB_I2C_ENABLE_STATUS     0x9c

/* Interrupts bits */
#define TSB_I2C_INTR_RX_UNDER     0x0001
#define TSB_I2C_INTR_RX_OVER      0x0002
#define TSB_I2C_INTR_RX_FULL      0x0004
#define TSB_I2C_INTR_TX_OVER      0x0008
#define TSB_I2C_INTR_TX_EMPTY     0x0010
#define TSB_I2C_INTR_RD_REQ       0x0020
#define TSB_I2C_INTR_TX_ABRT      0x0040
#define TSB_I2C_INTR_RX_DONE      0x0080
#define TSB_I2C_INTR_ACTIVITY     0x0100
#define TSB_I2C_INTR_STOP_DET     0x0200
#define TSB_I2C_INTR_START_DET    0x0400
#define TSB_I2C_INTR_GEN_CALL     0x0800


#define TSB_I2C_STATUS_ACTIVITY       0x1
#define TSB_I2C_ERR_TX_ABRT           0x1
#define TSB_I2C_TAR_10BITADDR_MASTER  (1 << 12)

/* I2C configuration bits */
#define TSB_I2C_CON_MASTER            0x01
#define TSB_I2C_CON_SPEED_STD         0x02
#define TSB_I2C_CON_SPEED_FAST        0x04
#define TSB_I2C_CON_10BITADDR_MASTER  0x10
#define TSB_I2C_CON_RESTART_EN        0x20
#define TSB_I2C_CON_SLAVE_DISABLE     0x40

/* status codes */
#define TSB_I2C_STATUS_IDLE                 0x0
#define TSB_I2C_STATUS_WRITE_IN_PROGRESS    0x1
#define TSB_I2C_STATUS_READ_IN_PROGRESS     0x2
#define TSB_I2C_STATUS_TIMEOUT              0x3

/* hardware abort codes from the TSB_I2C_TX_ABRT_SOURCE register */
#define ABRT_7B_ADDR_NOACK  0
#define ABRT_10ADDR1_NOACK  1
#define ABRT_10ADDR2_NOACK  2
#define ABRT_TXDATA_NOACK   3
#define ABRT_GCALL_NOACK    4
#define ABRT_GCALL_READ     5
#define ABRT_SBYTE_ACKDET   7
#define ABRT_SBYTE_NORSTRT  9
#define ABRT_10B_RD_NORSTRT 10
#define ABRT_MASTER_DIS     11
#define ARBT_LOST           12

#define TSB_I2C_TX_ABRT_7B_ADDR_NOACK     (1UL << ABRT_7B_ADDR_NOACK)
#define TSB_I2C_TX_ABRT_10ADDR1_NOACK     (1UL << ABRT_10ADDR1_NOACK)
#define TSB_I2C_TX_ABRT_10ADDR2_NOACK     (1UL << ABRT_10ADDR2_NOACK)
#define TSB_I2C_TX_ABRT_TXDATA_NOACK      (1UL << ABRT_TXDATA_NOACK)
#define TSB_I2C_TX_ABRT_GCALL_NOACK       (1UL << ABRT_GCALL_NOACK)
#define TSB_I2C_TX_ABRT_GCALL_READ        (1UL << ABRT_GCALL_READ)
#define TSB_I2C_TX_ABRT_SBYTE_ACKDET      (1UL << ABRT_SBYTE_ACKDET)
#define TSB_I2C_TX_ABRT_SBYTE_NORSTRT     (1UL << ABRT_SBYTE_NORSTRT)
#define TSB_I2C_TX_ABRT_10B_RD_NORSTRT    (1UL << ABRT_10B_RD_NORSTRT)
#define TSB_I2C_TX_ABRT_MASTER_DIS        (1UL << ABRT_MASTER_DIS)
#define TSB_I2C_TX_ARB_LOST               (1UL << ARBT_LOST)

#define TSB_I2C_TX_ABRT_NOACK (TSB_I2C_TX_ABRT_7B_ADDR_NOACK | \
                               TSB_I2C_TX_ABRT_10ADDR1_NOACK | \
                               TSB_I2C_TX_ABRT_10ADDR2_NOACK | \
                               TSB_I2C_TX_ABRT_TXDATA_NOACK | \
                               TSB_I2C_TX_ABRT_GCALL_NOACK)


#endif /* __ARCH_ARM_SRC_TSB_TSB_I2C_H */
