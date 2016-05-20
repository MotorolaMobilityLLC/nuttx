/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/device.h>
#include <nuttx/device_display.h>

#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/mhb/mhb_dsi_display.h>

#define CONFIG_DSI_DISPLAY_TE (0)

#define DISPLAY_SUPPLIER_ID (0x6001)
#define DISPLAY_ID0 (0x05)
#define DISPLAY_ID1 (0x03)
#define DISPLAY_ID2 (0x01)

static bool _is_valid_panel_info(const struct mhb_dsi_panel_info *panel_info) {
    return panel_info->supplier_id == DISPLAY_SUPPLIER_ID &&
           panel_info->id0 == DISPLAY_ID0 &&
           panel_info->id1 == DISPLAY_ID1 &&
           panel_info->id2 == DISPLAY_ID2;
}

const static struct mhb_cdsi_config DISPLAY_CONFIG = {
    /* Common */
    .direction = 1, /* TX */
    .mode = 0, /* DSI */

    .tx_num_lanes = 4,
    .rx_num_lanes = 4,
    .tx_bits_per_lane = 998400000,
    .rx_bits_per_lane = 807540000,

    .hs_rx_timeout = 0xffffffff,

    .framerate = 60,
    .width = 1080,
    .height = 1920,
    .physical_width = 68,
    .physical_height = 121,
    .bpp = 24,

    .bta_enabled = 1,
    .continuous_clock = 1,
    .blank_packet_enabled = 1,
    .video_mode = 0,
    .color_bar_enabled = 0,

    .vss_control_payload = 0x0000,

    .horizontal_front_porch = 24,
    .horizontal_pulse_width = 4,
    .horizontal_back_porch = 40,

    .vertical_front_porch = 16,
    .vertical_pulse_width = 2,
    .vertical_back_porch = 16,

    .t_clk_pre = 0x1b,
    .t_clk_post = 0x28,
};

const static struct mhb_cdsi_cmd DISPLAY_ON_COMMANDS[] = {
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE0, .length = 2, .u = { .spdata = 0x0011 }, .delay = 120000 }, /* exit_sleep_mode */
#if CONFIG_DSI_DISPLAY_TE
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE1, .length = 2, .u = { .spdata = 0x00035 }, .delay = 120000 }, /* set_tear_on */
#else
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE1, .length = 2, .u = { .spdata = 0x0034 }, .delay = 120000 }, /* set_tear_off */
#endif
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE0, .length = 2, .u = { .spdata = 0x0029 }, .delay = 120000 }, /* set_display_on */
};

const static struct mhb_cdsi_cmd DISPLAY_OFF_COMMANDS[] = {
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE0, .length = 2, .u = { .spdata = 0x0028 }, .delay = 120000 }, /* set_display_off */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE0, .length = 2, .u = { .spdata = 0x0010 }, .delay = 120000 }, /* enter_sleep_mode */
};

int _mhb_dsi_display_get_config(uint8_t instance,
    const struct mhb_dsi_panel_info *panel_info,
    const struct mhb_cdsi_config **cfg,
    size_t *size)
{
    if (panel_info) {
        bool valid = _is_valid_panel_info(panel_info);
        if (!valid) {
            vdbg("panel info does not match\n");
        }
    }

    if (cfg) {
        *cfg = &DISPLAY_CONFIG;
    }

    if (size) {
        *size = sizeof(DISPLAY_CONFIG);
    }

    return 0;
}

int _mhb_dsi_display_get_on_commands(uint8_t instance,
    const struct mhb_dsi_panel_info *panel_info,
    const struct mhb_cdsi_cmd **cmds,
    size_t *size)
{
    if (cmds) {
        *cmds = DISPLAY_ON_COMMANDS;
    }

    if (size) {
        *size = sizeof(DISPLAY_ON_COMMANDS);
    }

    return 0;
}

int _mhb_dsi_display_get_off_commands(uint8_t instance,
    const struct mhb_dsi_panel_info *panel_info,
    const struct mhb_cdsi_cmd **cmds,
    size_t *size)
{
    if (cmds) {
        *cmds = DISPLAY_OFF_COMMANDS;
    }

    if (size) {
        *size = sizeof(DISPLAY_OFF_COMMANDS);
    }

    return 0;
}
