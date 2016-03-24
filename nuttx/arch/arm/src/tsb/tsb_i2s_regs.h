/*
 * Copyright (c) 2016 Motorola Mobility, LLC
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

/*
 * Limit register polling loops so they don't go forever.  1000 should be
 * large enough to handle any case where something isn't seriously wrong.
 */
#define TSB_I2S_POLL_LIMIT                              1000

#define TSB_I2S_PLLA_ID                                 0

#define TSB_I2S_TX_START_THRESHOLD                      0x20

/* Base addresses of the I2S register blocks. */
#define TSB_I2S_REG_SC_BASE                             0x40007000
#define TSB_I2S_REG_SO_BASE                             0x40008000
#define TSB_I2S_REG_SI_BASE                             0x40009000
#define TSB_SYSCTL_BASE                                 0x40000000

/* System Controller/Bridge Registers */
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR                0x0440

#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_DAC16BIT_SEL     BIT(2)
/*
 * ES2 uses bit 1 and ES3 uses bit 4.  Toshiba has indicated setting the ES2 bit
 * on ES3 will be acceptable. *
 */
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_LR_BCLK_SEL      (BIT(1)|BIT(4))
/* ES2 uses bit 0 and ES3 uses bit 3.  See the comment above for more details. */
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASTER_CLOCK_SEL (BIT(0)|BIT(3))
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASK                                 \
                        (TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_DAC16BIT_SEL      | \
                         TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_LR_BCLK_SEL       | \
                         TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASTER_CLOCK_SEL)

//TODO: These like the base addresses may be better in the device tree.
/* DMA Device IDs. */
#define TSB_I2S_SO_DMA_DEVID                            0
#define TSB_I2S_SI_DMA_DEVID                            1

/* I2S Registers */
#define TSB_I2S_REG_START                               0x0004
#define TSB_I2S_REG_BUSY                                0x0008
#define TSB_I2S_REG_STOP                                0x000c
#define TSB_I2S_REG_AUDIOSET                            0x0010
#define TSB_I2S_REG_INTSTAT                             0x0014
#define TSB_I2S_REG_INTMASK                             0x0018
#define TSB_I2S_REG_INTCLR                              0x001c
#define TSB_I2S_REG_MUTE                                0x0024
#define TSB_I2S_REG_EPTR                                0x0028
#define TSB_I2S_REG_TX_SSIZE                            0x0030
#define TSB_I2S_REG_REGBUSY                             0x0040
#define TSB_I2S_REG_MODESET                             0x00f8
#define TSB_I2S_REG_LMEM00                              0x0100

#define TSB_I2S_REG_START_START                         BIT(8)
#define TSB_I2S_REG_START_SPK_MIC_START                 BIT(0)

#define TSB_I2S_REG_BUSY_LRERRBUSY                      BIT(17)
#define TSB_I2S_REG_BUSY_ERRBUSY                        BIT(16)
#define TSB_I2S_REG_BUSY_BUSY                           BIT(8)
#define TSB_I2S_REG_BUSY_SERIBUSY                       BIT(1)
#define TSB_I2S_REG_BUSY_SPK_MIC_BUSY                   BIT(0)

#define TSB_I2S_REG_STOP_I2S_STOP                       BIT(0)

#define TSB_I2S_REG_AUDIOSET_DTFMT_POS                  16
#define TSB_I2S_REG_AUDIOSET_DTFMT                      BIT(TSB_I2S_REG_AUDIOSET_DTFMT_POS)
#define TSB_I2S_REG_AUDIOSET_SDEDGE_POS                 12
#define TSB_I2S_REG_AUDIOSET_SDEDGE                     BIT(TSB_I2S_REG_AUDIOSET_SDEDGE_POS)
#define TSB_I2S_REG_AUDIOSET_EDGE_POS                   11
#define TSB_I2S_REG_AUDIOSET_EDGE                       BIT(TSB_I2S_REG_AUDIOSET_EDGE_POS)
#define TSB_I2S_REG_AUDIOSET_SCLKTOWS_POS               8
#define TSB_I2S_REG_AUDIOSET_SCLKTOWS                   BIT(TSB_I2S_REG_AUDIOSET_SCLKTOWS_POS)
#define TSB_I2S_REG_AUDIOSET_WORDLEN_POS                0
#define TSB_I2S_REG_AUDIOSET_WORDLEN_MASK               (0x3f<<TSB_I2S_REG_AUDIOSET_WORDLEN_POS)
#define TSB_I2S_REG_AUDIOSET_MASK           (TSB_I2S_REG_AUDIOSET_DTFMT     | \
                                             TSB_I2S_REG_AUDIOSET_SDEDGE    | \
                                             TSB_I2S_REG_AUDIOSET_EDGE      | \
                                             TSB_I2S_REG_AUDIOSET_SCLKTOWS  | \
                                             TSB_I2S_REG_AUDIOSET_WORDLEN_MASK)
#define TSB_I2S_REG_AUDIOSET_SC_MASK        (TSB_I2S_REG_AUDIOSET_EDGE | \
                                             TSB_I2S_REG_AUDIOSET_SCLKTOWS)

#define TSB_I2S_REG_INT_DMACMSK                         BIT(16)
#define TSB_I2S_REG_INT_LRCK                            BIT(3)
#define TSB_I2S_REG_INT_UR                              BIT(2)
#define TSB_I2S_REG_INT_OR                              BIT(1)
#define TSB_I2S_REG_INT_INT                             BIT(0)
#define TSB_I2S_REG_INT_ERROR_MASK          (TSB_I2S_REG_INT_LRCK   | \
                                             TSB_I2S_REG_INT_UR     | \
                                             TSB_I2S_REG_INT_OR)

#define TSB_I2S_REG_MUTE_MUTEN                          BIT(0)

#define TSB_I2S_REG_EPTR_ERRPOINTER_GET(a)              ((a) & 0x3f)
#define TSB_I2S_REG_EPTR_ERRPOINTER_SET(a)              ((a) & 0x3f)

#define TSB_I2S_REG_TX_SSIZE_TXSTARTSIZE_GET(r)         ((r) & 0x3f)
#define TSB_I2S_REG_TX_SSIZE_TXSTARTSIZE_SET(v)         ((v) & 0x3f)

#define TSB_I2S_REG_REGBUSY_MODESETPEND                 BIT(19)
#define TSB_I2S_REG_REGBUSY_TXSSIZEPEND                 BIT(18)
#define TSB_I2S_REG_REGBUSY_MUTEPEND                    BIT(17)
#define TSB_I2S_REG_REGBUSY_AUDIOSETPEND                BIT(16)
#define TSB_I2S_REG_REGBUSY_MODESETBUSY                 BIT(3)
#define TSB_I2S_REG_REGBUSY_TXSSIZEBUSY                 BIT(2)
#define TSB_I2S_REG_REGBUSY_MUTEBUSY                    BIT(1)
#define TSB_I2S_REG_REGBUSY_AUDIOSETBUSY                BIT(0)

#define TSB_I2S_REG_MODESET_I2S_STEREO                  0x0
#define TSB_I2S_REG_MODESET_LR_STEREO                   0x2
#define TSB_I2S_REG_MODESET_LR_STEREO_REV_POL           0x3
#define TSB_I2S_REG_MODESET_PCM_MONO                    0x4
#define TSB_I2S_REG_MODESET_PCM_MONO_REV_POL            0x5
#define TSB_I2S_REG_MODESET_WS_MASK                     0x7

#define TSB_I2S_FLAG_OPEN                               BIT(0)
#define TSB_I2S_FLAG_CONFIGURED                         BIT(1)
#define TSB_I2S_FLAG_ENABLED                            BIT(2)
#define TSB_I2S_FLAG_RX_PREPARED                        BIT(3)
#define TSB_I2S_FLAG_RX_ACTIVE                          BIT(4)
#define TSB_I2S_FLAG_TX_PREPARED                        BIT(5)
#define TSB_I2S_FLAG_TX_ACTIVE                          BIT(6)
