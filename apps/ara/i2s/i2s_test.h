/*
 * Copyright (c) 2015-2016 Google, Inc.
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

#ifndef __I2S_TEST_H__
#define __I2S_TEST_H__

struct i2s_test_stats {
    unsigned int    tx_cnt;
    unsigned int    tx_err_cnt;
    unsigned int    rx_cnt;
    unsigned int    rx_err_cnt;
    unsigned int    rx_bad_data_cnt;
};

struct i2s_test_info {
    uint8_t                 is_transmitter;
    uint8_t                 is_receiver;
    uint8_t                 is_i2s;
    uint8_t                 is_gen_audio;
    uint32_t                aud_frequency;
    uint32_t                aud_volume;
    uint8_t                 check_rx_data;
    unsigned long           rb_entries;
    unsigned long           samples_per_rb_entry;
    uint16_t                left;
    uint16_t                right;
    struct i2s_test_stats   stats;
};

extern sem_t i2s_test_done_sem;
extern struct device_i2s_pcm i2s_test_pcm;

int i2s_test_start_transmitter(struct i2s_test_info *info,
                               struct device *dev);
void i2s_test_stop_transmitter(struct device *dev);

#endif /* __I2S_TEST_H__ */
