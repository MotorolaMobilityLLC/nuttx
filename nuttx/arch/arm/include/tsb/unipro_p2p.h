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

#ifndef _UNIPRO_P2P_H_
#define _UNIPRO_P2P_H_

#include <stdlib.h>

struct p2p_link_err_reason {
    uint32_t link_lost_ind;
    uint32_t phy_ind;
    uint32_t pa_ind;
    uint32_t d_ind;
    uint32_t n_ind;
    uint32_t t_ind;
};

void unipro_p2p_setup_connection(unsigned int cport);
void unipro_p2p_peer_detected(void);
void unipro_p2p_peer_lost(struct p2p_link_err_reason *reason);
void unipro_p2p_setup(void);
void unipro_p2p_detect_linkloss(void);
bool unipro_p2p_is_link_up(void);

void unipro_powermode_change(uint8_t txgear, uint8_t rxgear, uint8_t pwrmode, uint8_t series, uint8_t termination);

void unipro_stop(void);
void unipro_restart(void);
void unipro_reset(void);

void unipro_dump_attributes(int peer);
void unipro_dump_cport_attributes(size_t start, size_t end, int peer);
void unipro_dump_status(void);
void unipro_dump_ints(void);
void unipro_dump_cports(void);
void unipro_dump_rx(void);
void unipro_dump_tx(void);

#endif
