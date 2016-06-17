
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
#ifndef _I2S_UNIPRO_H_
# define _I2S_UNIPRO_H_
# include <nuttx/config.h>
# include <nuttx/i2s_tunnel/i2s_tunnel.h>
# include <stdbool.h>
# include <errno.h>

# if defined(CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL)
int i2s_unipro_tunnel_init(void);
void i2s_unipro_tunnel_deinit(void);
int i2s_unipro_tunnel_unipro_register(void);
int i2s_unipro_tunnel_unipro_unregister(void);
int i2s_unipro_tunnel_enable(bool enable);
int i2s_unipro_tunnel_i2s_config(unsigned int sample_rate,
                                 I2S_TUNNEL_I2S_MODE_T mode,
                                 uint8_t sample_size_bits,
                                 uint8_t flags);
int i2s_unipro_tunnel_arm(bool enable);
void i2s_unipro_tunnel_start(bool start);
int i2s_unipro_tunnel_info(struct i2s_tunnel_info_s *info);
# else
#  define i2s_unipro_tunnel_init()            (-ENODEV)
#  define i2s_unipro_tunnel_unipro_register() (-ENODEV)
#  define i2s_unipro_tunnel_enable(enable)    (-ENODEV)
#  define i2s_unipro_i2s_config(sample_rate, sample_size_bits, mode, edge, master) (-ENODEV)
#  define i2s_unipro_tunnel_arm(enable)       (-EMODEV)
#  define i2s_unipro_tunnel_start(enable)
#  define i2s_unipro_tunnel_info()            (-ENODEV)
# endif
#endif
