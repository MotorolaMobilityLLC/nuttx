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

#ifndef __ARCH_ARM_INCLUDE_TSB_CDSI_CONFIG_H
#define __ARCH_ARM_INCLUDE_TSB_CDSI_CONFIG_H

#define TSB_CDSI_MODE_DSI (0)
#define TSB_CDSI_MODE_CSI (CDSI_CDSIRX_MODE_CONFIG_CSI2MODE_BIT)

struct cdsi_config {
    /* Common */
    uint32_t direction:1;  /* RX: 0 (CDSI -> UniPro), TX: 1 (UniPro -> CDSI) */
    uint32_t mode:1;       /* TSB_CDSI_MODE_DSI | TSB_CDSI_MODE_CSI */
    uint32_t rx_num_lanes:3;
    uint32_t tx_num_lanes:3;
    uint32_t rx_bits_per_lane;
    uint32_t tx_bits_per_lane;
    /* RX only */
    uint32_t hs_rx_timeout;
    /* TX only */
    uint32_t pll_frs;
    uint32_t pll_prd;
    uint32_t pll_fbd;
    uint32_t framerate;
    uint32_t width;
    uint32_t height;
    uint32_t bpp;
    uint32_t bta_enabled:1;
    uint32_t continuous_clock:1;
    uint32_t blank_packet_enabled:1;
    uint32_t video_mode:1;
    uint32_t color_bar_enabled:1;
    uint32_t vss_control_payload;
    uint8_t keep_alive;
    uint8_t clk_pre;
    uint8_t clk_post;
    /* CSI only */
    /* DSI only */
    uint8_t horizontal_front_porch;
    uint8_t horizontal_back_porch;
    uint8_t horizontal_pulse_width;
    uint8_t horizontal_sync_skew;
    uint8_t horizontal_left_border;
    uint8_t horizontal_right_border;
    uint8_t vertical_front_porch;
    uint8_t vertical_back_porch;
    uint8_t vertical_pulse_width;
    uint8_t vertical_top_border;
    uint8_t vertical_bottom_border;
    /* Video Mode only */
    /* Command Mode only */
    uint8_t vsync_mode;
};

void cdsi_modify(struct cdsi_dev *dev, uint32_t addr, uint32_t mask, uint32_t field);

int cdsi_read_until(struct cdsi_dev *dev, uint32_t addr, uint32_t mask, uint32_t desired_value, uint32_t retries);

int cdsi_initialize_rx(struct cdsi_dev *dev, const struct cdsi_config *config);
int cdsi_rx_start(struct cdsi_dev *dev);
int cdsi_uninitialize_rx(struct cdsi_dev *dev, const struct cdsi_config *config);

int cdsi_initialize_tx(struct cdsi_dev *dev, const struct cdsi_config *config);
int cdsi_tx_start(struct cdsi_dev *dev);
int cdsi_tx_stop(struct cdsi_dev *dev);
int cdsi_uninitialize_tx(struct cdsi_dev *dev, const struct cdsi_config *config);

/**
 * The following helper macros use the register definitions in cdsi_off_def.h and
 * cdsi_reg_def.h to simplify the calls and make them less prone to typos.
 *
 *
 * @param[in] name: the address offset definition without the trailing _OFFS
 * @param[in] field: the mask definition without the trailing _MASK
 * @param[in] value: the value of the field starting at bit 0 (shifting is done by the macros)
 */

#define CDSI_READ_UNTIL_CLR_RETRIES(dev, name, field, retries) cdsi_read_until(dev, name##_OFFS, name##_##field##_MASK, 0, retries)
#define CDSI_READ_UNTIL_SET_RETRIES(dev, name, field, retries) cdsi_read_until(dev, name##_OFFS, name##_##field##_MASK, name##_##field##_MASK, retries)

#define CDSI_DEFAULT_RETRIES (100)

/**
 * @brief           Read a field of a CDSI register.  Right-shift if necessary.
 * @return          The value of the field
 * @param[in]       dev: the CDSI device
 * @param[in]       name: the name of the register
 * @param[in]       field: the name of the field
 */
#define CDSI_READ(dev, name, field) ((cdsi_read(dev, name##_OFFS) & name##_##field##_MASK) >> name##_##field##_SHIFT)

/**
 * @brief           Write a value into a field of a CDSI register.  Write zeros into all other fields. Left-shift if necessary.
 * @return          none
 * @param[in]       dev: the CDSI device
 * @param[in]       name: the name of the register
 * @param[in]       field: the name of the field
 * @param[in]       value: the value of the field to write
 */
#define CDSI_WRITE(dev, name, field, value) cdsi_write(dev, name##_OFFS, ((value << name##_##field##_SHIFT) & name##_##field##_MASK));

/**
 * @brief           Read the CDSI register, set the bits field, write the result back
 * @return          none
 * @param[in]       dev: the CDSI device
 * @param[in]       name: the name of the register
 * @param[in]       field: the name of the field
 * @param[in]       value: the value of the field to write
 */
#define CDSI_MODIFY(dev, name, field, value) cdsi_modify(dev, name##_OFFS, name##_##field##_MASK, ((value << name##_##field##_SHIFT) & name##_##field##_MASK));

#endif

