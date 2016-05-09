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

#ifndef MHB_DSI_DISPLAY_H
#define MHB_DSI_DISPLAY_H

/* These weak_functions are used to separate generic DSI display logic
 * from panel-specific DSI logic.  Each function is used to get a block
 * of data (panel config, DCS on commands, or DCS off commands) from
 * target-specific source (e.g. ./nuttx/configs/<target>).
 *
 * Additionally, the panel_info structure allows the target-specific
 * code to return panel-specific blocks based on the panel type.  This
 * can allow support for multiple panels in one target.
 */

extern int _mhb_dsi_display_get_config(uint8_t instance,
    const struct mhb_dsi_panel_info *panel_info,
    const struct mhb_cdsi_config **cfg,
    size_t *size);

extern int _mhb_dsi_display_get_on_commands(uint8_t instance,
    const struct mhb_dsi_panel_info *panel_info,
    const struct mhb_cdsi_cmd **cmds,
    size_t *size);

extern int _mhb_dsi_display_get_off_commands(uint8_t instance,
    const struct mhb_dsi_panel_info *panel_info,
    const struct mhb_cdsi_cmd **cmds,
    size_t *size);

extern int mhb_dsi_display_set_brightness(uint8_t brightness);

#endif
