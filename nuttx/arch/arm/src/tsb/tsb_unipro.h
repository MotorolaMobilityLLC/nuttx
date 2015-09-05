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

#ifndef __TSB_UNIPRO_H__
#define __TSB_UNIPRO_H__

#ifdef UNIPRO_DEBUG
#define DBG_UNIPRO(fmt, ...) lldbg(fmt, __VA_ARGS__)
#else
#define DBG_UNIPRO(fmt, ...) ((void)0)
#endif

#define CPORT_TX_BUF_SIZE         (0x20000U)
#define CPORT_EOM_BIT(cport)      (cport->tx_buf + (CPORT_TX_BUF_SIZE - 1))

#define TRANSFER_MODE          (2)

struct cport {
    struct unipro_driver *driver;
    uint8_t *tx_buf;                // TX region for this CPort
    uint8_t *rx_buf;                // RX region for this CPort
    unsigned int cportid;
    int connected;

    struct list_head tx_fifo;
};

struct cport *cport_handle(unsigned int cportid);
uint16_t unipro_get_tx_free_buffer_space(struct cport *cport);
int unipro_tx_init(void);

#endif /* __TSB_UNIPRO_H__ */

