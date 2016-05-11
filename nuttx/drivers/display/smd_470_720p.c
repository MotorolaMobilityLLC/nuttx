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

#define CONFIG_DSI_DISPLAY_VIDEO_MODE (1)
#define CONFIG_DSI_DISPLAY_TE (0)

#define DISPLAY_SUPPLIER_ID (0x0000)
#define DISPLAY_ID0 (0x00)
#define DISPLAY_ID1 (0x00)
#define DISPLAY_ID2 (0x00)

static bool _is_valid_panel_info(const struct mhb_dsi_panel_info *panel_info) {
    return panel_info->supplier_id == DISPLAY_SUPPLIER_ID &&
           panel_info->id0 == DISPLAY_ID0 &&
           panel_info->id1 == DISPLAY_ID1 &&
           panel_info->id2 == DISPLAY_ID2;
}

const static struct mhb_cdsi_config DISPLAY_CONFIG = {
    .direction = 1, /* TX */
    .mode = 0, /* DSI */

    .tx_num_lanes = 4,
    .rx_num_lanes = 4,
    .tx_bits_per_lane = 504000000,
    .rx_bits_per_lane = 423640000,

    .hs_rx_timeout = 0xffffffff,

    .framerate = 60,
    .width = 720,
    .height = 1280,
    .physical_width = 56,
    .physical_height = 99,
    .bpp = 24,

    .bta_enabled = 1,
    .continuous_clock = 1,
    .blank_packet_enabled = 1,

#if CONFIG_DSI_DISPLAY_VIDEO_MODE
    .video_mode = 1, /* video-mode */
#endif

    .t_clk_pre = 4,
    .t_clk_post = 26,

#if CONFIG_DSI_DISPLAY_TE
    .vsync_mode = 1, /* gpio-vsync */
#endif

    .horizontal_front_porch = 144,
    .horizontal_pulse_width = 12,
    .horizontal_back_porch = 32,

    .vertical_front_porch = 9,
    .vertical_pulse_width = 4,
    .vertical_back_porch = 3,
};

const static struct mhb_cdsi_cmd DISPLAY_ON_COMMANDS[] = {
    /* Common */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE0, .length = 2, .u = { .spdata = 0x0011 }, .delay = 1000 }, /* exit_sleep_mode */

#if CONFIG_DSI_DISPLAY_VIDEO_MODE
    /* Video-mode */
    { .ctype = MHB_CTYPE_LP_LONG,  .dtype = MHB_DTYPE_DCS_LONG_WRITE, .length = 3, .u = { .lpdata = { 0x005a5af0 } }, .delay = 1000 }, /* unlock_lvl_2 */
    { .ctype = MHB_CTYPE_LP_LONG,  .dtype = MHB_DTYPE_DCS_LONG_WRITE, .length = 3, .u = { .lpdata = { 0x005a5af1 } }, .delay = 1000 }, /* unlock_lvl_mtp */
    { .ctype = MHB_CTYPE_LP_LONG,  .dtype = MHB_DTYPE_DCS_LONG_WRITE, .length = 3, .u = { .lpdata = { 0x005a5afc } }, .delay = 1000 }, /* unlock_lvl_3 */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE1, .length = 2, .u = { .spdata = 0x05f2 }, .delay = 1000 }, /* set_video_mode */
#else
    /* Command-mode */
# if CONFIG_DSI_DISPLAY_TE
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE1, .length = 2, .u = { .spdata = 0x0035 }, .delay = 1000 }, /* set_tear_on */
# else
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE1, .length = 2, .u = { .spdata = 0x0034 }, .delay = 1000 }, /* set_tear_off */
# endif
#endif

    /* Common */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE1, .length = 2, .u = { .spdata = 0x2053 }, .delay = 1000 }, /* disp_ctrl */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE1, .length = 2, .u = { .spdata = 0xff51 }, .delay = 1000 }, /* brightness_ctrl */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE1, .length = 2, .u = { .spdata = 0x0055 }, .delay = 1000 }, /* acl_enable_disable_settings */
    { .ctype = MHB_CTYPE_LP_LONG,  .dtype = MHB_DTYPE_DCS_LONG_WRITE, .length = 5, .u = { .lpdata = { 0x0002cf2a, 0x00000000 } }, .delay = 1000 }, /* set_column_address */
    { .ctype = MHB_CTYPE_LP_LONG,  .dtype = MHB_DTYPE_DCS_LONG_WRITE, .length = 5, .u = { .lpdata = { 0x0004ff2b, 0x00000000 } }, .delay = 1000 }, /* set_page_address */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE0, .length = 2, .u = { .spdata = 0x0029 }, .delay = 1000 }, /* set_display_on */
};

const static struct mhb_cdsi_cmd DISPLAY_OFF_COMMANDS[] = {
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE0, .length = 2, .u = { .spdata = 0x0028 }, .delay = 1000 }, /* set_display_off */
    { .ctype = MHB_CTYPE_LP_SHORT, .dtype = MHB_DTYPE_DCS_WRITE0, .length = 2, .u = { .spdata = 0x0010 }, .delay = 1000 }, /* enter_sleep_mode */
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
