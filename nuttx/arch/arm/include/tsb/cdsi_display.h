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

#ifndef __ARCH_ARM_INCLUDE_TSB_CDSI_DISPLAY_H
#define __ARCH_ARM_INCLUDE_TSB_CDSI_DISPLAY_H

/* CDSITX_SIDEBAND_CMDIF_01_APF_CMD_TYPE */
#define CTYPE_HS_FLAG    (0x0)
#define CTYPE_LP_FLAG    (0x8)
#define CTYPE_SHORT_FLAG (0x0)
#define CTYPE_LONG_FLAG  (0x4)

#define CTYPE_HS_SHORT (CTYPE_HS_FLAG|CTYPE_SHORT_FLAG)
#define CTYPE_HS_LONG  (CTYPE_HS_FLAG|CTYPE_LONG_FLAG)
#define CTYPE_LP_SHORT (CTYPE_LP_FLAG|CTYPE_SHORT_FLAG)
#define CTYPE_LP_LONG  (CTYPE_LP_FLAG|CTYPE_LONG_FLAG)

/* Data Types for Processor-sourced Packets */
#define DT_GEN_SHORT_WRITE0 0x03
#define DT_GEN_SHORT_WRITE1 0x13
#define DT_GEN_SHORT_WRITE2 0x23
#define DT_GEN_READ0        0x04
#define DT_GEN_READ1        0x14
#define DT_GEN_READ2        0x24
#define DT_DCS_WRITE0       0x05
#define DT_DCS_WRITE1       0x15
#define DT_DCS_READ0        0x06
#define DT_MAX_RET_PKT      0x37
#define DT_GEN_LONG_WRITE   0x29
#define DT_DCS_LONG_WRITE   0x39

/* Data Types for Peripheral-sourced Packets */
#define DT_RSP_ACK_AND_ERR      0x02
#define DT_RSP_EOT              0x08
#define DT_RSP_GEN_SHORT_READ1  0x11
#define DT_RSP_GEN_SHORT_READ2  0x12
#define DT_RSP_GEN_LONG_READ    0x1a
#define DT_RSP_DCS_LONG_READ    0x1c
#define DT_RSP_DCS_SHORT_READ1  0x21
#define DT_RSP_DCS_SHORT_READ2  0x22

struct dsi_cmd {
    uint8_t ctype;  /* CTYPE_* */
    uint8_t dt;     /* DT_DCS_* */
    union {
        struct {
            uint16_t data;
        } sp;
        struct {
            uint16_t length;
            const uint32_t *data;
        } lp;
    } u;
};

int dsi_write_cmd(struct cdsi_dev *dev, const struct dsi_cmd *cmd);

size_t dsi_read_cmd(struct cdsi_dev *dev, const struct dsi_cmd *cmd, uint8_t *buffer, size_t buffer_length);

int dsi_read_panel_info(struct cdsi_dev *dev, uint16_t *supplier_id, uint8_t *id0, uint8_t *id1, uint8_t *id2);
int dsi_read_power_mode(struct cdsi_dev *dev, uint8_t *mode);

#endif

