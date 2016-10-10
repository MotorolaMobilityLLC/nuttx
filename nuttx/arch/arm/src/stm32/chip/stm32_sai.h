/******************************************************************************
 * arch/arm/src/stm32/chip/stm32_sai.h
 *
 * Copyright (c) 2016 Motorola Mobility, LLC.
 * All rights reserved.
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
 ******************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_SAI_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_SAI_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/******************************************************************************
 * Pre-processor definitions
 ******************************************************************************/

/* Common Register Offsets ****************************************************/

#define STM32_SAI_GCR_OFFSET       0x0000 /* Global configuration register */

/* Block Specific Register Offsets ********************************************/

#define STM32_SAI_XCR1_OFFSET      0x0000 /* Configuration register 1 */
#define STM32_SAI_XCR2_OFFSET      0x0004 /* Configuration register 2 */
#define STM32_SAI_XFRCR_OFFSET     0x0008 /* Frame configuration register */
#define STM32_SAI_XSLOTR_OFFSET    0x000c /* Slot register */
#define STM32_SAI_XIM_OFFSET       0x0010 /* Interrupt mask register */
#define STM32_SAI_XSR_OFFSET       0x0014 /* Status register */
#define STM32_SAI_XCLRFR_OFFSET    0x0018 /* Clear flag register */
#define STM32_SAI_XDR_OFFSET       0x001c /* Data register */

/* Common Register Addresses **************************************************/

#define STM32_SAI1_GCR             (STM32_SAI1_BASE+STM32_SAI_GCR_OFFSET)

#define STM32_SAI2_GCR             (STM32_SAI2_BASE+STM32_SAI_GCR_OFFSET)

/* Block Specific Register Addresses ******************************************/

#define STM32_SAI1_ACR1            (STM32_SAI1_A_BASE+STM32_SAI_XCR1_OFFSET)
#define STM32_SAI1_ACR2            (STM32_SAI1_A_BASE+STM32_SAI_XCR2_OFFSET)
#define STM32_SAI1_AFRCR           (STM32_SAI1_A_BASE+STM32_SAI_XFRCR_OFFSET)
#define STM32_SAI1_ASLOTR          (STM32_SAI1_A_BASE+STM32_SAI_XSLOTR_OFFSET)
#define STM32_SAI1_AIM             (STM32_SAI1_A_BASE+STM32_SAI_XIM_OFFSET)
#define STM32_SAI1_ASR             (STM32_SAI1_A_BASE+STM32_SAI_XSR_OFFSET)
#define STM32_SAI1_ACLRFR          (STM32_SAI1_A_BASE+STM32_SAI_XCLRFR_OFFSET)
#define STM32_SAI1_ADR             (STM32_SAI1_A_BASE+STM32_SAI_XDR_OFFSET)

#define STM32_SAI1_BCR1            (STM32_SAI1_B_BASE+STM32_SAI_XCR1_OFFSET)
#define STM32_SAI1_BCR2            (STM32_SAI1_B_BASE+STM32_SAI_XCR2_OFFSET)
#define STM32_SAI1_BFRCR           (STM32_SAI1_B_BASE+STM32_SAI_XFRCR_OFFSET)
#define STM32_SAI1_BSLOTR          (STM32_SAI1_B_BASE+STM32_SAI_XSLOTR_OFFSET)
#define STM32_SAI1_BIM             (STM32_SAI1_B_BASE+STM32_SAI_XIM_OFFSET)
#define STM32_SAI1_BSR             (STM32_SAI1_B_BASE+STM32_SAI_XSR_OFFSET)
#define STM32_SAI1_BCLRFR          (STM32_SAI1_B_BASE+STM32_SAI_XCLRFR_OFFSET)
#define STM32_SAI1_BDR             (STM32_SAI1_B_BASE+STM32_SAI_XDR_OFFSET)

#define STM32_SAI2_ACR1            (STM32_SAI2_A_BASE+STM32_SAI_XCR1_OFFSET)
#define STM32_SAI2_ACR2            (STM32_SAI2_A_BASE+STM32_SAI_XCR2_OFFSET)
#define STM32_SAI2_AFRCR           (STM32_SAI2_A_BASE+STM32_SAI_XFRCR_OFFSET)
#define STM32_SAI2_ASLOTR          (STM32_SAI2_A_BASE+STM32_SAI_XSLOTR_OFFSET)
#define STM32_SAI2_AIM             (STM32_SAI2_A_BASE+STM32_SAI_XIM_OFFSET)
#define STM32_SAI2_ASR             (STM32_SAI2_A_BASE+STM32_SAI_XSR_OFFSET)
#define STM32_SAI2_ACLRFR          (STM32_SAI2_A_BASE+STM32_SAI_XCLRFR_OFFSET)
#define STM32_SAI2_ADR             (STM32_SAI2_A_BASE+STM32_SAI_XDR_OFFSET)

#define STM32_SAI2_BCR1            (STM32_SAI2_B_BASE+STM32_SAI_XCR1_OFFSET)
#define STM32_SAI2_BCR2            (STM32_SAI2_B_BASE+STM32_SAI_XCR2_OFFSET)
#define STM32_SAI2_BFRCR           (STM32_SAI2_B_BASE+STM32_SAI_XFRCR_OFFSET)
#define STM32_SAI2_BSLOTR          (STM32_SAI2_B_BASE+STM32_SAI_XSLOTR_OFFSET)
#define STM32_SAI2_BIM             (STM32_SAI2_B_BASE+STM32_SAI_XIM_OFFSET)
#define STM32_SAI2_BSR             (STM32_SAI2_B_BASE+STM32_SAI_XSR_OFFSET)
#define STM32_SAI2_BCLRFR          (STM32_SAI2_B_BASE+STM32_SAI_XCLRFR_OFFSET)
#define STM32_SAI2_BDR             (STM32_SAI2_B_BASE+STM32_SAI_XDR_OFFSET)

/* Register Bitfield Definitions **********************************************/

/* Global configuration register */

/* Configuration register 1 */

#define SAI_XCR1_MODE_SHIFT        (0)        /* Bits 0-1: SAIx audio block mode */
#define SAI_XCR1_MODE_MASK         (3 << SAI_XCR1_MODE_SHIFT)
#  define SAI_XCR1_MODE_MASTER_TX  (0 << SAI_XCR1_MODE_SHIFT) /* Master transmitter */
#  define SAI_XCR1_MODE_MASTER_RX  (1 << SAI_XCR1_MODE_SHIFT) /* Master receiver */
#  define SAI_XCR1_MODE_SLAVE_TX   (2 << SAI_XCR1_MODE_SHIFT) /* Slave transmitter */
#  define SAI_XCR1_MODE_SLAVE_RX   (3 << SAI_XCR1_MODE_SHIFT) /* Slave receiver */
#define SAI_XCR1_DS_SHIFT          (5)        /* Bits 5-7: Data size */
#define SAI_XCR1_DS_MASK           (7 << SAI_XCR1_DS_SHIFT)
#  define SAI_XCR1_DS_8_BITS       (2 << SAI_XCR1_DS_SHIFT) /* 8 bits */
#  define SAI_XCR1_DS_10_BITS      (3 << SAI_XCR1_DS_SHIFT) /* 10 bits */
#  define SAI_XCR1_DS_16_BITS      (4 << SAI_XCR1_DS_SHIFT) /* 16 bits */
#  define SAI_XCR1_DS_20_BITS      (5 << SAI_XCR1_DS_SHIFT) /* 20 bits */
#  define SAI_XCR1_DS_24_BITS      (6 << SAI_XCR1_DS_SHIFT) /* 24 bits */
#  define SAI_XCR1_DS_32_BITS      (7 << SAI_XCR1_DS_SHIFT) /* 32 bits */
#define SAI_XCR1_CKSTR             (1 << 9)   /* Bit 9:  Clock strobing edge */
#  define SAI_XCR1_CKSTR_FALLING   (0)             /* Signals received are sampled on falling edge */
#  define SAI_XCR1_CKSTR_RISING    SAI_XCR1_CKSTR  /* Signals received are sampled on rising edge */
#define SAI_XCR1_SYNCEN_SHIFT      (10)       /* Bits 10-11: Synchronization enable */
#define SAI_XCR1_SYNCEN_MASK       (3 << SAI_XCR1_SYNCEN_SHIFT)
#  define SAI_XCR1_SYNCEN_ASYNC    (0 << SAI_XCR1_SYNCEN_SHIFT) /* Asynchronous mode */
#  define SAI_XCR1_SYNCEN_SYNC_INT (1 << SAI_XCR1_SYNCEN_SHIFT) /* Synchronous with other internal sub-block */
#  define SAI_XCR1_SYNCEN_SYNC_EXT (2 << SAI_XCR1_SYNCEN_SHIFT) /* Synchronous with external SAI */
#define SAI_XCR1_SAIX              (1 << 16)  /* Bit 16: Audio block enable */
#define SAI_XCR1_DMAEN             (1 << 17)  /* Bit 17: DMA enable */
#define SAI_XCR1_MCKDIV_SHIFT      (20)       /* Bits 20-23: Master clock divider */
#define SAI_XCR1_MCKDIV_MASK       (15 << SAI_XCR1_MCKDIV_SHIFT)

/* Configuration register 2 */

#define SAI_XCR2_FTH_SHIFT         (0)        /* Bits 0-2: FIFO threshold */
#define SAI_XCR2_FTH_MASK          (7 << SAI_XCR2_FTH_SHIFT)
#  define SAI_XCR2_FTH_EMPTY       (0 << SAI_XCR2_FTH_SHIFT) /* FIFO empty */
#  define SAI_XCR2_FTH_1QF         (1 << SAI_XCR2_FTH_SHIFT) /* 1/4 FIFO */
#  define SAI_XCR2_FTH_HF          (2 << SAI_XCR2_FTH_SHIFT) /* 1/2 FIFO */
#  define SAI_XCR2_FTH_3QF         (3 << SAI_XCR2_FTH_SHIFT) /* 3/4 FIFO */
#  define SAI_XCR2_FTH_FULL        (4 << SAI_XCR2_FTH_SHIFT) /* FIFO full */

/* Frame configuration register */

#define SAI_XFRCR_FRL_SHIFT        (0)        /* Bits 0-7: Frame length */
#define SAI_XFRCR_FRL_MASK         (0xFF << SAI_XFRCR_FRL_SHIFT)
#  define SAI_XFRCR_FRL(n)         (((n) - 1) << SAI_XFRCR_FRL_SHIFT)
#define SAI_XFRCR_FSALL_SHIFT      (8)        /* Bits 8-14: Frame sync active level length */
#define SAI_XFRCR_FSALL_MASK       (0x7F << SAI_XFRCR_FSALL_SHIFT)
#  define SAI_XFRCR_FSALL(n)       (((n) - 1) << SAI_XFRCR_FSALL_SHIFT)
#define SAI_XFRCR_FSDEF            (1 << 16)  /* Bit 16: Frame synchronization definition */
#  define SAI_XFRCR_FSDEF_SF       (0)             /* FS signal is a start frame signal */
#  define SAI_XFRCR_FSDEF_CH_ID    SAI_XFRCR_FSDEF /* FS signal is a start of frame + channel side identification */
#define SAI_XFRCR_FSPOL            (1 << 17)  /* Bit 17: Frame synchronization polarity */
#  define SAI_XFRCR_FSPOL_LOW      (0)             /* FS is active low */
#  define SAI_XFRCR_FSPOL_HIGH     SAI_XFRCR_FSPOL /* FS is active high */
#define SAI_XFRCR_FSOFF            (1 << 18)  /* Bit 18: Frame synchronization offset */
#  define SAI_XFRCR_FSOFF_FB       (0)             /* FS is asserted on the first bit of the slot 0 */
#  define SAI_XFRCR_FSOFF_BFB      SAI_XFRCR_FSOFF /* FS is asserted one bit before the first bit of the slot 0 */

/* Slot register */

#define SAI_XSLOTR_NBSLOT_SHIFT    (8)        /* Bits 8-11: Number of slots in an audio frame */
#define SAI_XSLOTR_NBSLOT_MASK     (15 << SAI_XSLOTR_NBSLOT_SHIFT)
#  define SAI_XSLOTR_NBSLOT(n)     (((n) - 1) << SAI_XSLOTR_NBSLOT_SHIFT) /* n = 1..16 */
#define SAI_XSLOTR_SLOTEN_SHIFT    (1 << 16)  /* Bits 16-31: Slot enable */
#define SAI_XSLOTR_SLOTEN_MASK     (0xFFFF << SAI_XSLOTR_NBSLOT_SHIFT)
#  define SAI_XSLOTR_SLOTEN_0      (1 << 16)  /* Bit 16: Slot 0 Enabled */
#  define SAI_XSLOTR_SLOTEN_1      (1 << 17)  /* Bit 17: Slot 1 Enabled */
#  define SAI_XSLOTR_SLOTEN_2      (1 << 18)  /* Bit 18: Slot 2 Enabled */
#  define SAI_XSLOTR_SLOTEN_3      (1 << 19)  /* Bit 19: Slot 3 Enabled */
#  define SAI_XSLOTR_SLOTEN_4      (1 << 20)  /* Bit 20: Slot 4 Enabled */
#  define SAI_XSLOTR_SLOTEN_5      (1 << 21)  /* Bit 21: Slot 5 Enabled */
#  define SAI_XSLOTR_SLOTEN_6      (1 << 22)  /* Bit 22: Slot 6 Enabled */
#  define SAI_XSLOTR_SLOTEN_7      (1 << 23)  /* Bit 23: Slot 7 Enabled */
#  define SAI_XSLOTR_SLOTEN_8      (1 << 24)  /* Bit 24: Slot 8 Enabled */
#  define SAI_XSLOTR_SLOTEN_9      (1 << 25)  /* Bit 25: Slot 9 Enabled */
#  define SAI_XSLOTR_SLOTEN_10     (1 << 26)  /* Bit 26: Slot 10 Enabled */
#  define SAI_XSLOTR_SLOTEN_11     (1 << 27)  /* Bit 27: Slot 11 Enabled */
#  define SAI_XSLOTR_SLOTEN_12     (1 << 28)  /* Bit 28: Slot 12 Enabled */
#  define SAI_XSLOTR_SLOTEN_13     (1 << 29)  /* Bit 29: Slot 13 Enabled */
#  define SAI_XSLOTR_SLOTEN_14     (1 << 30)  /* Bit 30: Slot 14 Enabled */
#  define SAI_XSLOTR_SLOTEN_15     (1 << 31)  /* Bit 31: Slot 15 Enabled */

/* Interrupt mask register */

/* Status register */

/* Clear flag register */

/* Data register */

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_SAI_H */
