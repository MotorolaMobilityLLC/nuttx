
/*
 * Copyright (c) 2016 Motorola Mobility, LLC
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
#ifndef _I2S_TUNNEL_H_
# define _I2S_TUNNEL_H_
# include <nuttx/config.h>
# include <stdbool.h>
# include <errno.h>

/* Flags used to control I2S hardware. */
/** @brief Set the TX data latch edge to falling. */
# define I2S_TUNNEL_I2S_FLAGS_TX_EDGE_FALLING  0x00
/** @brief Set the TX data latch edge to rising. */
# define I2S_TUNNEL_I2S_FLAGS_TX_EDGE_RISING   0x01
/** @brief Set the RX data latch edge to falling. */
# define I2S_TUNNEL_I2S_FLAGS_RX_EDGE_FALLING  0x00
/** @brief Set the RX data latch edge to rising. */
# define I2S_TUNNEL_I2S_FLAGS_RX_EDGE_RISING   0x02
/** @brief Set the Left Right data latch edge to falling. */
# define I2S_TUNNEL_I2S_FLAGS_LR_EDGE_FALLING  0x00
/** @brief Set the Left Right data latch edge to rising. */
# define I2S_TUNNEL_I2S_FLAGS_LR_EDGE_RISING   0x04
/** @brief Set I2S to Slave mode. */
# define I2S_TUNNEL_I2S_FLAGS_SLAVE            0x00
/** @brief Set I2S to Master mode. */
# define I2S_TUNNEL_I2S_FLAGS_MASTER           0x08

/**
 * @brief Modes supported by the i2s tunneling over uart driver.
 */
typedef enum
{
    I2S_TUNNEL_I2S_MODE_PCM_MONO,
    I2S_TUNNEL_I2S_MODE_PCM_MONO_REV,
    I2S_TUNNEL_I2S_MODE_I2S_STEREO,
    I2S_TUNNEL_I2S_MODE_LR_STEREO,
    I2S_TUNNEL_I2S_MODE_LR_STEREO_REV,
    I2S_TUNNEL_I2S_MODE_END
} I2S_TUNNEL_I2S_MODE_T;

struct i2s_tunnel_info_s
{
    bool enabled;
    bool is_master;
    unsigned int bclk_rate;
    unsigned int bytes_per_sample;
    unsigned int roundtrip_timer_ticks;
    int timer_offset;
    unsigned int i2s_tunnel_rx_packets;
    unsigned int i2s_tunnel_packet_size;
    unsigned int i2s_tx_samples_dropped;
    unsigned int i2s_tx_samples_retransmitted;
    unsigned int i2s_tx_buffers_dropped;
};

#endif
