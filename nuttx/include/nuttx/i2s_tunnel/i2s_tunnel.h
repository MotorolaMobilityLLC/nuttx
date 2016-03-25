
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

/**
 * @brief Edges supported by the I2S hardware.
 */
typedef enum
{
    I2S_TUNNEL_I2S_EDGE_RISING,
    I2S_TUNNEL_I2S_EDGE_FALLING,
    I2S_TUNNEL_I2S_EDGE_END,
} I2S_TUNNEL_I2S_EDGE_T;

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

# if defined(CONFIG_I2S_TUNNEL)
/*
 * On the APBA messages are sent to the APBE to start things. so interface
 * functions are not required.
 */
#  if defined(CONFIG_ICE_APBE)
int i2s_tunnel_enable(bool enable);
int i2s_tunnel_i2s_config(unsigned int sample_rate,
                          uint8_t sample_size_bits,
                          I2S_TUNNEL_I2S_MODE_T mode,
                          I2S_TUNNEL_I2S_EDGE_T edge,
                          bool master);
int i2s_tunnel_arm(bool enable);
void i2s_tunnel_start(bool start);
int i2s_tunnel_get_info(struct i2s_tunnel_info_s *local, struct i2s_tunnel_info_s *remote);
#  endif
int i2s_tunnel_init(void);
# else
#  define i2s_tunnel_init()            (-ENODEV)
#  define i2s_tunnel_enable(enable)    (-ENODEV)
#  define i2s_i2s_config(sample_rate, sample_size_bits, mode, edge, master) (-ENODEV)
#  define i2s_tunnel_arm(enable)       (-EMODEV)
#  define i2s_tunnel_start(enable)
#  define i2s_tunnel_get_info()        (-ENODEV)
# endif
#endif
