/*
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

/**
 * @author: Jean Pihet
 */

#ifndef  _TSB_SWITCH_DRIVER_ES2_H_
#define  _TSB_SWITCH_DRIVER_ES2_H_

#define SW_SPI_PORT_1       (1)
#define SW_SPI_PORT_2       (2)
#define SWITCH_SPI_FREQUENCY 13000000

// Max NULL frames to wait for a reply from the switch
#define SWITCH_WAIT_REPLY_LEN   (16)
// Write status reply length
#define SWITCH_WRITE_STATUS_LEN (11)
// Total number of NULLs to clock out to ensure a write status is read
#define SWITCH_WRITE_STATUS_NNULL (SWITCH_WAIT_REPLY_LEN + \
                                   SWITCH_WRITE_STATUS_LEN)


/* Switch internal attributes */
#define SWINE                       (0x0015)
#define SWINS                       (0x0016)
#define SPICTLB                     (0x0041)
#define SPIINTE                     (0x0042)
#define SPICEE                      (0x0043)
#define SPICES                      (0x0044)
#define SPI3EE                      (0x0045)
#define SPI3ES                      (0x0046)
#define SPI4EE                      (0x0047)
#define SPI4ES                      (0x0048)
#define SPI5EE                      (0x0049)
#define SPI5ES                      (0x004a)

/* Switch internal attributes values */
#define SWINE_ENABLE_ALL            (0x7FFFB805)
#define SPIINTE_ENABLE_ALL          (0x3)
#define SPICEE_ENABLE_ALL           (0x3)
#define SPI3EE_ENABLE_ALL           (0x6F)
#define SPI45EE_ENABLE_ALL          (0xFE00006F)

/* TSB attributes fields values */
#define TSB_INTERRUPTENABLE_ALL     (0xFFFF)
#define TSB_L4_INTERRUPTENABLE_ALL  (0x2800)
#define TSB_INTERRUPT_SPI3ES        (1 << 11)
#define TSB_INTERRUPT_SPI4ES        (1 << 12)
#define TSB_INTERRUPT_SPI5ES        (1 << 13)
#define TSB_INTERRUPT_SPICES        (1 << 15)
#define TSB_INTERRUPT_SPIPORT4_RX   (1 << 20)
#define TSB_INTERRUPT_SPIPORT5_RX   (1 << 21)
#define TSB_INTERRUPT_SWINTERNAL    (1 << 31)

int tsb_switch_es2_fct_enable(struct tsb_switch *);

int tsb_switch_es2_init(struct tsb_switch *, unsigned int spi_bus);
void tsb_switch_es2_exit(struct tsb_switch *);

#endif
