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

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>

#include <nuttx/arch.h>

#include <arch/chip/cdsi.h>
#include <arch/chip/cdsi_config.h>
#include <arch/chip/cdsi_display.h>
#include <arch/chip/cdsi_reg_def.h>

#include "chip.h"
#include "up_arch.h"
#include "tsb_scm.h"

static void *cdsi_lookup_table(const void *table, size_t row_size, uint32_t key) {
    uint8_t *curr = (uint8_t *)table;

    while (1) {
        uint32_t table_key = (*((uint32_t *)curr));

        if (!table_key) {
            /* Out of entries */
            return NULL;
        }

        if (key < table_key) {
            return curr;
        }

        curr += row_size;
    }
}

#define CDSI_LOOKUP_TABLE(table, key) (struct table##_row *)cdsi_lookup_table(g_##table, sizeof(struct table##_row), key)

struct rx_table_row {
    uint32_t key;
    uint8_t ppi_hs_rx_clk_sel;
    uint8_t dt_hs_exit_count;
    uint8_t dt_clr_sipo_count;
    uint8_t cl_settle_count;
} __attribute__((packed));

const static struct rx_table_row g_rx_table[] = {
   {   80000000, 0, 0,  0, 0 },
   {   95500000, 2, 3,  4, 4 },
   {  116000000, 2, 3,  5, 4 },
   {  137000000, 2, 3,  6, 4 },
   {  167000000, 2, 3,  7, 4 },
   {  188000000, 2, 3,  8, 4 },
   {  209000000, 2, 3,  9, 4 },
   {  230500000, 2, 3, 10, 4 },
   {  251000000, 2, 3, 11, 4 },
   {  272000000, 2, 3, 12, 4 },
   {  293500000, 2, 3, 13, 4 },
   {  314000000, 2, 3, 14, 4 },
   {  335000000, 2, 3, 15, 4 },
   {  356500000, 2, 3, 16, 4 },
   {  373000000, 2, 3, 17, 4 },
   {  395000000, 1, 0,  7, 4 },
   {  437000000, 1, 0,  8, 4 },
   {  479000000, 1, 0,  9, 4 },
   {  521000000, 1, 0, 10, 4 },
   {  563500000, 1, 0, 11, 4 },
   {  605000000, 1, 0, 12, 4 },
   {  647500000, 1, 0, 13, 4 },
   {  682000000, 1, 0, 14, 4 },
   {  766500000, 0, 0,  6, 4 },
   {  851000000, 0, 0,  7, 4 },
   {  935000000, 0, 0,  8, 4 },
   { 1000000000, 0, 0,  9, 4 },
   {          0, 0, 0,  0, 0 },
};

struct sys_cld_tclk_table_row {
    uint32_t key;
    uint8_t sys_cld_tclk_sel;
    uint8_t sys_cld_tclk_div;
} __attribute__((packed));

const static struct sys_cld_tclk_table_row g_sys_cld_tclk_table[] = {
    {   80000000, 0, 0 },
    {  220000000, 2, 2 },
    {  400000000, 1, 4 },
    { 1000000000, 0, 8 },
    {          0, 0, 0 },
};

struct tx_table_row {
    uint32_t key;
    uint8_t lp_tx_time_cnt_bta;
    uint8_t lp_tx_time_cnt;
    uint16_t twakeup_cnt_bta;
    uint16_t twakeup_cnt;
    uint8_t rx_ta_sure_cnt;
} __attribute__((packed));

const static struct tx_table_row g_tx_table[] = {
    {   80000000,  0, 0,    0,    0,  0 },
    {   95000000,  6, 2, 3394, 7918,  7 },
    {  127000000,  7, 3, 3970, 7939,  8 },
    {  159000000,  8, 4, 4418, 7951,  9 },
    {  190000000, 10, 5, 4418, 7951, 12 },
    {  220000000, 12, 7, 4319, 7939, 14 },
    {  254000000,  8, 3, 3529, 7939,  9 },
    {  318000000,  9, 4, 3976, 7951, 10 },
    {  381000000, 10, 5, 4331, 7951, 12 },
    {  400000000, 11, 6, 4331, 7951, 13 },
    {  509000000,  8, 3, 3976, 7955,  9 },
    {  636000000,  9, 4, 3976, 7955, 10 },
    {  763000000, 10, 5, 4336, 7955, 12 },
    {  891000000, 11, 6, 4642, 7957, 13 },
    { 1000000000, 12, 7, 4895, 7958, 14 },
    {          0,  0, 0,    0,    0,  0 },
};

struct tclk_pre_zero_cnt_table_row {
    uint32_t key;
    uint8_t tclk_pre_zero_cnt;
} __attribute__((packed));

const static struct tclk_pre_zero_cnt_table_row g_tclk_pre_zero_cnt_table[] = {
    {   80000000,  0 },
    {   83000000, 13 },
    {   90000000, 14 },
    {   97000000, 15 },
    {  104000000, 16 },
    {  110000000, 17 },
    {  117000000, 18 },
    {  124000000, 19 },
    {  131000000, 20 },
    {  137000000, 21 },
    {  144000000, 22 },
    {  151000000, 23 },
    {  157000000, 24 },
    {  164000000, 25 },
    {  171000000, 26 },
    {  178000000, 27 },
    {  184000000, 28 },
    {  191000000, 29 },
    {  198000000, 30 },
    {  204000000, 31 },
    {  211000000, 32 },
    {  220000000, 33 },
    {  225000000, 16 },
    {  238000000, 17 },
    {  251000000, 18 },
    {  265000000, 19 },
    {  278000000, 20 },
    {  292000000, 21 },
    {  305000000, 22 },
    {  319000000, 23 },
    {  332000000, 24 },
    {  346000000, 25 },
    {  359000000, 26 },
    {  372000000, 27 },
    {  386000000, 28 },
    {  399000000, 29 },
    {  400000000, 30 },
    {  413000000, 14 },
    {  440000000, 15 },
    {  467000000, 16 },
    {  493000000, 17 },
    {  520000000, 18 },
    {  547000000, 19 },
    {  574000000, 20 },
    {  601000000, 21 },
    {  628000000, 22 },
    {  655000000, 23 },
    {  682000000, 24 },
    {  708000000, 25 },
    {  735000000, 26 },
    {  762000000, 27 },
    {  789000000, 28 },
    {  816000000, 29 },
    {  843000000, 30 },
    {  870000000, 31 },
    {  897000000, 32 },
    {  923000000, 33 },
    {  950000000, 34 },
    {  977000000, 35 },
    { 1000000000, 36 },
    {          0,  0 },
};

struct tclk_pre_cnt_table_row {
    uint32_t key;
    uint8_t tclk_pre_cnt;
} __attribute__((packed));

const static struct tclk_pre_cnt_table_row g_tclk_pre_cnt_table[] = {
    {   80000000, 0 },
    {  198000000, 7 },
    {  220000000, 8 },
    {  397000000, 2 },
    {  400000000, 3 },
    { 1000000000, 0 },
    {          0, 0 },
};

struct tclk_prepare_cnt_table_row {
    uint32_t key;
    uint8_t tclk_prepare_cnt;
} __attribute__((packed));

const static struct tclk_prepare_cnt_table_row g_tclk_prepare_cnt_table[] = {
    {   80000000, 0 },
    {   82000000, 2 },
    {  106000000, 3 },
    {  130000000, 4 },
    {  154000000, 5 },
    {  178000000, 6 },
    {  220000000, 7 },
    {  260000000, 4 },
    {  308000000, 5 },
    {  356000000, 6 },
    {  400000000, 7 },
    {  425000000, 3 },
    {  521000000, 4 },
    {  617000000, 5 },
    {  713000000, 6 },
    {  809000000, 7 },
    {  905000000, 8 },
    { 1000000000, 9 },
    {          0, 0 },
};

struct tclk_exit_cnt_table_row {
    uint32_t key;
    uint8_t tclk_exit_cnt;
} __attribute__((packed));

const static struct tclk_exit_cnt_table_row g_tclk_exit_cnt_table[] = {
    {   80000000,  0 },
    {   84000000,  5 },
    {   98000000,  6 },
    {  112000000,  7 },
    {  126000000,  8 },
    {  140000000,  9 },
    {  154000000, 10 },
    {  168000000, 11 },
    {  182000000, 12 },
    {  196000000, 13 },
    {  210000000, 14 },
    {  220000000, 15 },
    {  224000000,  7 },
    {  252000000,  8 },
    {  280000000,  9 },
    {  308000000, 10 },
    {  336000000, 11 },
    {  364000000, 12 },
    {  392000000, 13 },
    {  400000000, 14 },
    {  448000000,  7 },
    {  504000000,  8 },
    {  560000000,  9 },
    {  616000000, 10 },
    {  673000000, 11 },
    {  729000000, 12 },
    {  785000000, 13 },
    {  841000000, 14 },
    {  897000000, 15 },
    {  953000000, 16 },
    { 1000000000, 17 },
    {          0,  0 },
};

struct tclk_trail_cnt_table_row {
    uint32_t key;
    uint8_t tclk_trail_cnt;
} __attribute__((packed));

const static struct tclk_trail_cnt_table_row g_tclk_trail_cnt_table[] = {
    {   80000000, 0 },
    {  103000000, 4 },
    {  136000000, 5 },
    {  168000000, 6 },
    {  200000000, 7 },
    {  220000000, 8 },
    {  281000000, 3 },
    {  346000000, 4 },
    {  400000000, 5 },
    {  443000000, 1 },
    {  573000000, 2 },
    {  702000000, 3 },
    {  831000000, 4 },
    {  960000000, 5 },
    { 1090000000, 6 },
    {          0, 0 },
};

struct ths_pre_zero_cnt_table_row {
    uint32_t key;
    uint8_t ths_pre_zero_cnt;
} __attribute__((packed));

const static struct ths_pre_zero_cnt_table_row g_ths_pre_zero_cnt_table[] = {
    {   80000000,  0 },
    {  199000000,  0 },
    {  212000000,  1 },
    {  220000000,  2 },
    {  226000000,  0 },
    {  254000000,  1 },
    {  281000000,  2 },
    {  309000000,  3 },
    {  336000000,  4 },
    {  363000000,  5 },
    {  391000000,  6 },
    {  400000000,  7 },
    {  446000000,  3 },
    {  501000000,  4 },
    {  556000000,  5 },
    {  611000000,  6 },
    {  666000000,  7 },
    {  721000000,  8 },
    {  775000000,  9 },
    {  830000000, 10 },
    {  885000000, 11 },
    {  940000000, 12 },
    {  995000000, 13 },
    { 1000000000, 14 },
    {          0,  0 },
};

struct ths_prepare_cnt_table_row {
    uint32_t key;
    uint8_t ths_prepare_cnt;
} __attribute__((packed));

const static struct ths_prepare_cnt_table_row g_ths_prepare_cnt_table[] = {
    {   80000000,  0 },
    {   99000000,  5 },
    {  124000000,  6 },
    {  148000000,  7 },
    {  173000000,  8 },
    {  198000000,  9 },
    {  220000000, 10 },
    {  259000000,  5 },
    {  309000000,  6 },
    {  358000000,  7 },
    {  400000000,  8 },
    {  480000000,  4 },
    {  579000000,  5 },
    {  679000000,  6 },
    {  778000000,  7 },
    {  877000000,  8 },
    {  976000000,  9 },
    { 1075000000, 10 },
    {          0,  0 },
};

struct ths_exit_cnt_table_row {
    uint32_t key;
    uint8_t ths_exit_cnt;
} __attribute__((packed));

const static struct ths_exit_cnt_table_row g_ths_exit_cnt_table[] = {
    {   80000000,  0 },
    {   85000000,  9 },
    {   99000000, 10 },
    {  113000000, 11 },
    {  127000000, 12 },
    {  142000000, 13 },
    {  156000000, 14 },
    {  170000000, 15 },
    {  184000000, 16 },
    {  199000000, 17 },
    {  213000000, 18 },
    {  220000000, 19 },
    {  227000000,  9 },
    {  255000000, 10 },
    {  284000000, 11 },
    {  312000000, 12 },
    {  341000000, 13 },
    {  369000000, 14 },
    {  398000000, 15 },
    {  400000000, 16 },
    {  455000000,  8 },
    {  511000000,  9 },
    {  568000000, 10 },
    {  625000000, 11 },
    {  682000000, 12 },
    {  739000000, 13 },
    {  796000000, 14 },
    {  853000000, 15 },
    {  910000000, 16 },
    {  967000000, 17 },
    { 1000000000, 18 },
    {          0,  0 },
};

struct ths_trail_cnt_table_row {
    uint32_t key;
    uint8_t ths_trail_cnt;
} __attribute__((packed));

const static struct ths_trail_cnt_table_row g_ths_trail_cnt_table[] = {
    {   80000000,  0 },
    {   96000000, 10 },
    {  128000000, 11 },
    {  160000000, 12 },
    {  193000000, 13 },
    {  220000000, 14 },
    {  241000000,  6 },
    {  306000000,  7 },
    {  371000000,  8 },
    {  400000000,  9 },
    {  461000000,  4 },
    {  589000000,  5 },
    {  716000000,  6 },
    {  844000000,  7 },
    {  971000000,  8 },
    { 1000000000,  9 },
    {          0,  0 },
};

struct tclk_post_cnt_table_row {
    uint32_t key;
    uint8_t tclk_post_cnt_continuous;
    uint8_t tclk_post_cnt;
} __attribute__((packed));

const static struct tclk_post_cnt_table_row g_tclk_post_cnt_table[] = {
    {   80000000,  0,  0 },
    {  100000000, 16, 32 },
    {  120000000, 17, 33 },
    {  141000000, 18, 34 },
    {  161000000, 19, 35 },
    {  181000000, 20, 36 },
    {  201000000, 21, 37 },
    {  220000000, 23, 39 },
    {  221000000,  6, 18 },
    {  261000000,  7, 19 },
    {  302000000,  8, 20 },
    {  342000000,  9, 21 },
    {  382000000, 10, 22 },
    {  400000000, 11, 23 },
    {  463000000,  1, 11 },
    {  544000000,  2, 12 },
    {  624000000,  3, 13 },
    {  705000000,  4, 14 },
    {  785000000,  5, 15 },
    {  866000000,  6, 16 },
    {  946000000,  7, 17 },
    { 1000000000,  8, 18 },
    {          0,  0,  0 },
};

/**
 * @brief           Read the CDSI register, set the bits field, write the result back
 * @return          none
 * @param[in]       dev: the CDSI device
 * @param[in]       name: the name of the register
 * @param[in]       mask: the mask of the field to write
 * @param[in]       field: the name of the field
 */
void cdsi_modify(struct cdsi_dev *dev, uint32_t addr, uint32_t mask, uint32_t field) {
    uint32_t value = cdsi_read(dev, addr);
    value &= ~mask;
    value |= field;
    cdsi_write(dev, addr, value);
}

/**
 * @brief           Read the CDSI register until the mask bits are equal to the desired value.
 * @return          0 on success, -1 on timeout
 * @param[in]       dev: the CDSI device
 * @param[in]       addr: the offset address of the CDSI register
 * @param[in]       mask: the bits to inspect
 * @param[in]       desired_value: the value that the masked bits need to match
 * @param[in]       retries: the number of read-compare cycles to perform (0 for infinite)
 */
int cdsi_read_until(struct cdsi_dev *dev, uint32_t addr, uint32_t mask, uint32_t desired_value, uint32_t retries) {
    size_t i = 0;
    while (1) {
        uint32_t value = cdsi_read(dev, addr);
        if ((value & mask) == desired_value) {
            /* success */
            vdbg("wait: i=%d addr=%x value=%x\n", i, addr, value);
            return 0;
        }

        i++;
        if (retries && (i > retries)) {
            /* retry timeout */
            dbg("wait timeout: i=%d, addr=0x%x, value=0x%x\n", i, addr, value);
            return -ETIME;
        }
        usleep(1);
    }
}

static void cdsi_clear_rx_status(struct cdsi_dev *dev) {
    vdbg("\n");
    cdsi_write(dev, CDSI_CDSIRX_DSI_RXTRIG_INT_MASK_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI_CDSIRX_RXERR_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_TXERR_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_DSI_VC0_SH_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_DSI_VC1_SH_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_DSI_VC2_SH_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_DSI_VC3_SH_INT_MASK_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI_CDSIRX_DSI_VC0_LN_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_DSI_VC1_LN_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_DSI_VC2_LN_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_DSI_VC3_LN_INT_MASK_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC0_SH_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC1_SH_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC2_SH_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC3_SH_INT_MASK_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC0_LN0_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC0_LN1_INT_MASK_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC1_LN0_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC1_LN1_INT_MASK_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC2_LN0_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC2_LN1_INT_MASK_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC3_LN0_INT_MASK_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSIRX_CSI2_VC3_LN1_INT_MASK_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI_AL_RX_BRG_INT_MASK_OFFS, 0xffffffff);
}

int cdsi_initialize_rx(struct cdsi_dev *dev, const struct cdsi_config *config) {
    vdbg("\n");
    vdbg("mode=%d, num_lanes=%d, bits_per_lane=%d\n", config->mode, config->rx_num_lanes, config->rx_bits_per_lane);
    vdbg("framerate=%d, width=%d, height=%d, bpp=%d\n", config->framerate, config->width, config->height, config->bpp);
    vdbg("video_mode=%d\n", config->video_mode);
    vdbg("bta_enabled=%d\n", config->bta_enabled);
    vdbg("continuous_clock=%d\n", config->continuous_clock);
    vdbg("color_bar_enabled=%d\n", config->color_bar_enabled);
    vdbg("blank_packet_enabled=%d\n", config->blank_packet_enabled);

    const struct rx_table_row *rx_table = CDSI_LOOKUP_TABLE(rx_table, config->rx_bits_per_lane);
    int ret;

    cdsi_write(dev, CDSI_AL_RX_BRG_CSI_INFO_OFFS, 0);
    cdsi_write(dev, CDSI_AL_RX_BRG_CSI_DT0_OFFS, 0);
    cdsi_write(dev, CDSI_AL_RX_BRG_CSI_DT1_OFFS, 0);
    cdsi_write(dev, CDSI_AL_RX_BRG_CSI_DT2_OFFS, 0);
    cdsi_write(dev, CDSI_AL_RX_BRG_CSI_DT3_OFFS, 0);

    /* CSI/DSI mode */
    CDSI_WRITE(dev, CDSI_AL_RX_BRG_MODE, CSI2_MODE, config->mode);

    /* Enable CDSI RX bridge. */
    CDSI_MODIFY(dev, CDSI_AL_RX_BRG_MODE, BRG_EN, 1);

    /* Clocks not gated */
    CDSI_WRITE(dev, CDSI_CDSIRX_CLKEN, CDSIRXEN, 1);

    /* RX function enables */
    cdsi_write(dev, CDSI_CDSIRX_FUNC_ENABLE_OFFS, 0x00070701);

    /* Calibration control */
    cdsi_write(dev, CDSI_CDSIRX_PPI_DPHY_LPRXCALCNTRL_OFFS, 0x00190040);

    /* LP RX thresholds (clk, d0-d3) */
    cdsi_write(dev, CDSI_CDSIRX_PPI_DPHY_LPRX_THSLD_OFFS, 0x000002aa);

    /* Enable interrupts */
    cdsi_write(dev, CDSI_CDSIRX_LPRX_STATE_INT_MASK_OFFS, 0x1f1f1f1d);

    /* Start auto-calibration. */
    cdsi_write(dev, CDSI_CDSIRX_PPI_DPHY_LPRXAUTOCALST_OFFS, 1);

    /* Clock selection */
    CDSI_WRITE(dev, CDSI_CDSIRX_CLKSEL, PPIHSRXCLKEN, 1);
    CDSI_MODIFY(dev, CDSI_CDSIRX_CLKSEL, PPIHSRXCLKSEL, rx_table->ppi_hs_rx_clk_sel);

    /* DSI/CSI mode */
    CDSI_WRITE(dev, CDSI_CDSIRX_MODE_CONFIG, CSI2MODE, config->mode);

    /* Lane configuration */
    CDSI_WRITE(dev, CDSI_CDSIRX_LANE_ENABLE, CLANEEN, 1);
    CDSI_MODIFY(dev, CDSI_CDSIRX_LANE_ENABLE, DTLANEEN, config->rx_num_lanes);

    /* Enable virtual channels 0-3. */
    cdsi_write(dev, CDSI_CDSIRX_VC_ENABLE_OFFS, 0xf);

    /* Disable BTAs. */
    cdsi_write(dev, CDSI_CDSIRX_DSI_LPTX_MODE_OFFS, 1);

    /* Line initialization wait count */
    cdsi_write(dev, CDSI_CDSIRX_LINE_INIT_COUNT_OFFS, 0x12c0); /* 0x12c0 == 480ms */

    /* HS timeout */
    cdsi_write(dev, CDSI_CDSIRX_HSRXTO_COUNT_OFFS, config->hs_rx_timeout);

    /* Turnaround acknowledge timeout */
    cdsi_write(dev, CDSI_CDSIRX_DSI_TATO_COUNT_OFFS, 0xffffffff);

    /* LPTX timeout */
    cdsi_write(dev, CDSI_CDSIRX_DSI_LPTXTO_COUNT_OFFS, 0xffffffff);

    /* RX function mode enables */
    cdsi_write(dev, CDSI_CDSIRX_FUNC_MODE_OFFS, 0);

    /* HS RX control */
    CDSI_WRITE(dev, CDSI_CDSIRX_PPI_HSRX_CNTRL, CLHSRXENFCTRL, 1);

    /* DSI timings */
    CDSI_WRITE(dev, CDSI_CDSIRX_PPI_HSRX_COUNT, DTCLRSIPOCNT, rx_table->dt_clr_sipo_count);
    CDSI_MODIFY(dev, CDSI_CDSIRX_PPI_HSRX_COUNT, DTHSEXITCNT, rx_table->dt_hs_exit_count);
    CDSI_MODIFY(dev, CDSI_CDSIRX_PPI_HSRX_COUNT, CLSETTLECNT, rx_table->cl_settle_count);

    /* Disable D-Phy delay control. */
    cdsi_write(dev, CDSI_CDSIRX_PPI_DPHY_DLYCNTRL_OFFS, 0);

    /* TLPX period timeout */
    CDSI_WRITE(dev, CDSI_CDSIRX_PPI_DPHY_LPTXTIMECNT, LPTXTIMECNT, 0xfff);

    CDSI_WRITE(dev, CDSI_CDSIRX_PPI_DSI_BTA_COUNT, TXTAGO, 0x4);
    CDSI_MODIFY(dev, CDSI_CDSIRX_PPI_DSI_BTA_COUNT, RXTASURECNT, 0x7ff);

    /* LPTX output current */
    CDSI_WRITE(dev, CDSI_CDSIRX_PPI_DSI_DPHYTX_ADJUST, D0S_LPTXCURREN, 0x2);

    /* Termination trim (clk, d0-d3) */
    cdsi_write(dev, CDSI_CDSIRX_PPI_DPHY_HSRX_ADJUST_OFFS, 0x2aa);

    /* Wait for auto-calibration to finish. */
    ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSIRX_LPRX_STATE_INT_STAT,
                    AUTOCALDONE, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: auto-calibration failed\n");
        return ret;
    }

    cdsi_clear_rx_status(dev);

    cdsi_write(dev, CDSI_CDSIRX_START_OFFS, 1);

    return 0;
}

int cdsi_rx_start(struct cdsi_dev *dev) {
    int ret;

    vdbg("\n");

    /* Wait for line initialization to finish. */
    ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSIRX_LPRX_STATE_INT_STAT,
                    LINEINITDONE, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: line initialization failed\n");
        return ret;
    }

    CDSI_WRITE(dev, CDSI_CDSIRX_LPRX_STATE_INT_STAT, LINEINITDONE, 1);

    return 0;
}

int cdsi_uninitialize_rx(struct cdsi_dev *dev, const struct cdsi_config *config) {
    int result;

    vdbg("\n");

#if CONFIG_TSB_CDSI_WAIT_FOR_RXLP
    /* Wait until all lanes are in the stop state (set to 1). */
    const uint32_t mask0 = CDSI_CDSIRX_LANE_STATUS_LP_L0STOPSTATE_MASK|
                           CDSI_CDSIRX_LANE_STATUS_LP_L1STOPSTATE_MASK|
                           CDSI_CDSIRX_LANE_STATUS_LP_L2STOPSTATE_MASK|
                           CDSI_CDSIRX_LANE_STATUS_LP_L3STOPSTATE_MASK|
                           CDSI_CDSIRX_LANE_STATUS_LP_CLSTOPSTATE_MASK;
    result = cdsi_read_until(dev, CDSI_CDSIRX_LANE_STATUS_LP_OFFS, mask0, mask0,
                             CDSI_DEFAULT_RETRIES);
    if (result) {
        dbg("ERROR: Failed to stop lanes\n");
    }

    /* Wait until all lanes are in the LP state (set to 0). */
    const uint32_t mask1 = CDSI_CDSIRX_LANE_STATUS_HS_D0RXACTIVEHS_MASK|
                           CDSI_CDSIRX_LANE_STATUS_HS_D1RXACTIVEHS_MASK|
                           CDSI_CDSIRX_LANE_STATUS_HS_D2RXACTIVEHS_MASK|
                           CDSI_CDSIRX_LANE_STATUS_HS_D3RXACTIVEHS_MASK|
                           CDSI_CDSIRX_LANE_STATUS_HS_CLRXACTIVEHS_MASK;
    result = cdsi_read_until(dev, CDSI_CDSIRX_LANE_STATUS_HS_OFFS, mask1, 0,
                             CDSI_DEFAULT_RETRIES);
    if (result) {
        dbg("ERROR: Failed to go to LP\n");
    }

    /* Wait until the internal state is idle (set to 0). */
    const uint32_t mask2 = CDSI_CDSIRX_INTERNAL_STAT_CDSI_RXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_CDSI_EORRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_CL_LPRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_CL_HSRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_D0_LPRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_D0_HSRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_D1_LPRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_D1_HSRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_D2_LPRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_D2_HSRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_D3_LPRXBUSY_MASK|
                           CDSI_CDSIRX_INTERNAL_STAT_D3_HSRXBUSY_MASK;
    result = cdsi_read_until(dev, CDSI_CDSIRX_INTERNAL_STAT_OFFS, mask2, 0,
                             CDSI_DEFAULT_RETRIES);
    if (result) {
        dbg("ERROR: Failed to stop internal state\n");
    }
#endif

    /* Disable RX. */
    CDSI_WRITE(dev, CDSI_CDSIRX_START, CDSIRXSTART, 0);

    /* Clear CDSIRX internal state. */
    CDSI_WRITE(dev, CDSI_CDSIRX_SYSTEM_INIT, SYSINIT, 1);

    /* Disable CDSIRX clocks. */
    CDSI_WRITE(dev, CDSI_CDSIRX_CLKEN, CDSIRXEN, 0);

    /* Disable RX bridge. */
    CDSI_MODIFY(dev, CDSI_AL_RX_BRG_MODE, BRG_EN, 0);

    /* Wait until RX bridge is stopped. */
    result = CDSI_READ_UNTIL_CLR_RETRIES(dev, CDSI_AL_RX_BRG_MODE, BUSY,
                                         CDSI_DEFAULT_RETRIES);
    if (result) {
        dbg("ERROR: Failed to stop RX bridge\n");
    }

    return result;
}

static void cdsi_clear_tx_status(struct cdsi_dev *dev) {
    vdbg("\n");
    cdsi_write(dev, CDSI_CDSITX_INTERRUPT_STATUS_00_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_INTERRUPT_STATUS_01_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_INTERRUPT_STATUS_02_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_INTERRUPT_STATUS_03_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_INTERRUPT_STATUS_04_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_INTERRUPT_STATUS_05_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_INTERRUPT_STATUS_06_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_AL_TX_BRG_STATUS_OFFS, 0xffffffff);
}

static int cdsi_config_pll(uint32_t target, uint32_t *final_pll_frs,
                           uint32_t *final_pll_prd, uint32_t *final_pll_fbd) {
    const uint32_t MAX_PLL_FRS = (1 << 2);
    const uint32_t MAX_PLL_PRD = (1 << 2);
    const uint32_t MAX_PLL_FBD = (1 << 7);

    uint32_t pll_frs;
    uint32_t pll_prd;
    uint32_t pll_fbd;
    int32_t final_delta = LONG_MAX;

    if (!final_pll_frs || !final_pll_prd || !final_pll_fbd) {
        return -EINVAL;
    }

    if (*final_pll_frs || *final_pll_prd || *final_pll_fbd) {
        /* Already configured. */
        return 0;
    }

    /* Find best match. */
    for (pll_frs = 0; pll_frs < MAX_PLL_FRS; pll_frs++) {
        for (pll_prd = 0; pll_prd < MAX_PLL_PRD; pll_prd++) {
            for (pll_fbd = 0; pll_fbd < MAX_PLL_FBD; pll_fbd++) {
                /* Calculate */
                const uint32_t pll_vco = 2 * 192 * (pll_fbd + 1) / (pll_prd + 1) / 10;
                const uint32_t hsck = pll_vco / (1 << (pll_frs + 1));

                /* Validate */
                if (pll_vco <= 1000 || pll_vco >= 2000) {
                    continue;
                }

                if (hsck <= 80 || hsck >= 1000) {
                    continue;
                }

                /* Compare */
                const int32_t delta = abs((target / 1000000) - hsck);
                if (delta < final_delta) {
                    /* Save better match. */
                    *final_pll_frs = pll_frs;
                    *final_pll_prd = pll_prd;
                    *final_pll_fbd = pll_fbd;
                    final_delta = delta;
                }
            }
        }
    }

    return 0;
}

static int cdsi_validate_pll(uint32_t target, uint32_t pll_frs,
                           uint32_t pll_prd, uint32_t pll_fbd,
                           uint32_t *final_hsck) {
    const int32_t MAX_HSCK_DELTA = 1;

    int ret = 0;

    if (!final_hsck) {
        return -EINVAL;
    }

    vdbg("pll_frs=%d, pll_prd=%d, pll_fbd=%d\n", pll_frs, pll_prd, pll_fbd);

    const uint32_t pll_vco = 2 * 192 * (pll_fbd + 1) / (pll_prd + 1) / 10;
    vdbg("pll_vco=%d\n", pll_vco);
    if (pll_vco < 1000 || pll_vco >= 2000) {
        dbg("ERROR: pll_vco invalid: %d\n", pll_vco);
        ret = -ERANGE;
    }

    const uint32_t hsck = pll_vco / (1 << (pll_frs + 1));
    vdbg("hsck=%d\n", hsck);
    if (hsck < 80 || hsck >= 1000) {
        dbg("ERROR: hsck invalid: %d\n", hsck);
        ret = -ERANGE;
    }
    *final_hsck = hsck;

    const int32_t delta = abs((target / 1000000) - hsck);
    vdbg("delta=%d\n", delta);
    if (delta > MAX_HSCK_DELTA) {
        dbg("ERROR: delta invalid: %d\n", delta);
        ret = -ERANGE;
    }

    return ret;
}

int cdsi_initialize_tx(struct cdsi_dev *dev, const struct cdsi_config *config) {
    vdbg("\n");
    vdbg("mode=%d, num_lanes=%d, bits_per_lane=%d\n", config->mode, config->tx_num_lanes, config->tx_bits_per_lane);
    vdbg("framerate=%d, width=%d, height=%d, bpp=%d\n", config->framerate, config->width, config->height, config->bpp);
    vdbg("video_mode=%d\n", config->video_mode);
    vdbg("bta_enabled=%d\n", config->bta_enabled);
    vdbg("continuous_clock=%d\n", config->continuous_clock);
    vdbg("color_bar_enabled=%d\n", config->color_bar_enabled);
    vdbg("blank_packet_enabled=%d\n", config->blank_packet_enabled);

    uint32_t pll_frs = config->pll_frs;
    uint32_t pll_prd = config->pll_prd;
    uint32_t pll_fbd = config->pll_fbd;
    cdsi_config_pll(config->tx_bits_per_lane, &pll_frs, &pll_prd, &pll_fbd);

    uint32_t hsck = 0;
    cdsi_validate_pll(config->tx_bits_per_lane, pll_frs, pll_prd, pll_fbd, &hsck);

    const uint32_t horz_period_ns = 1000*1000*1000 / config->framerate / config->height;
    vdbg("horz_period_ns=%d\n", horz_period_ns);

#if CONFIG_DEBUG_VERBOSE
    const int32_t pixel_time = (config->bpp / config->tx_num_lanes) * 1000 / hsck;
    vdbg("pixel_time=%d\n", pixel_time);
#endif

#if CONFIG_DEBUG_VERBOSE
    const int32_t horz_pixels =
           config->horizontal_front_porch + config->horizontal_back_porch +
           config->horizontal_pulse_width + config->horizontal_sync_skew +
           config->horizontal_left_border + config->horizontal_right_border +
           config->width;
    vdbg("horz_pixels=%d\n", horz_pixels);

    const int32_t horz_period_ns_calc = horz_pixels * pixel_time;
    vdbg("horz_period_ns_calc=%d\n", horz_period_ns_calc);
#endif

    const struct sys_cld_tclk_table_row *sys_cld_tclk_table_row = CDSI_LOOKUP_TABLE(sys_cld_tclk_table, config->tx_bits_per_lane);
    const struct tx_table_row *tx_table_row = CDSI_LOOKUP_TABLE(tx_table, config->tx_bits_per_lane);
    const struct tclk_pre_zero_cnt_table_row *tclk_pre_zero_cnt_table_row = CDSI_LOOKUP_TABLE(tclk_pre_zero_cnt_table, config->tx_bits_per_lane);
    const struct tclk_pre_cnt_table_row *tclk_pre_cnt_table_row = CDSI_LOOKUP_TABLE(tclk_pre_cnt_table, config->tx_bits_per_lane);
    const struct tclk_prepare_cnt_table_row *tclk_prepare_cnt_table_row = CDSI_LOOKUP_TABLE(tclk_prepare_cnt_table, config->tx_bits_per_lane);
    const struct tclk_exit_cnt_table_row *tclk_exit_cnt_table_row = CDSI_LOOKUP_TABLE(tclk_exit_cnt_table, config->tx_bits_per_lane);
    const struct tclk_trail_cnt_table_row *tclk_trail_cnt_table_row = CDSI_LOOKUP_TABLE(tclk_trail_cnt_table, config->tx_bits_per_lane);
    const struct ths_pre_zero_cnt_table_row *ths_pre_zero_cnt_table_row = CDSI_LOOKUP_TABLE(ths_pre_zero_cnt_table, config->tx_bits_per_lane);
    const struct ths_prepare_cnt_table_row *ths_prepare_cnt_table_row = CDSI_LOOKUP_TABLE(ths_prepare_cnt_table, config->tx_bits_per_lane);
    const struct ths_exit_cnt_table_row *ths_exit_cnt_table_row = CDSI_LOOKUP_TABLE(ths_exit_cnt_table, config->tx_bits_per_lane);
    const struct ths_trail_cnt_table_row *ths_trail_cnt_table_row = CDSI_LOOKUP_TABLE(ths_trail_cnt_table, config->tx_bits_per_lane);
    const struct tclk_post_cnt_table_row *tclk_post_cnt_table_row = CDSI_LOOKUP_TABLE(tclk_post_cnt_table, config->tx_bits_per_lane);

    cdsi_write(dev, CDSI_AL_TX_BRG_SOFT_RESET_OFFS, 1);
    cdsi_write(dev, CDSI_AL_TX_BRG_SYSCLK_ENABLE_OFFS, 1);
    cdsi_write(dev, CDSI_AL_TX_BRG_CDSITX_MODE_OFFS, 1);

    uint32_t brg_mode;
    uint32_t sel1_set;
    uint32_t sel2_set;
    uint32_t mask1;
    uint32_t mask2;
    uint32_t csi2dsi_sel;
    uint32_t vdelay_start;
    uint32_t vdelay_end;
    if (config->mode == TSB_CDSI_MODE_CSI) {
        brg_mode = CDSI_AL_TX_BRG_MODE_AL_TX_BRG_CSI_MODE_MASK;

        sel1_set = 0x08fbfffc;
        sel2_set = 0xff00c0e0;

        mask1 = 0x08fb00fc;
        mask2 = 0xff00c0e0;

        csi2dsi_sel = 0x3;

        vdelay_end = vdelay_start = (config->tx_bits_per_lane / 1000 / 1000 * horz_period_ns / 1000 / 8 + 1) / 2;
    } else {
        if (config->video_mode) {
            brg_mode = 0;

            sel1_set = 0x8fff8ffd;
            sel2_set = 0x9dffadfd;

            mask1 = 0x8fd38fd1;
            mask2 = 0x9d7badf1;

            csi2dsi_sel = 0;

            vdelay_start = config->tx_bits_per_lane / 1000 / 1000 * horz_period_ns / 1000 / 8 + 1;
            vdelay_end = 0;
        } else {
            brg_mode = CDSI_AL_TX_BRG_MODE_AL_TX_BRG_COMMAND_MODE_MASK;

            sel1_set = 0;
            sel2_set = 0;

            mask1 = 0x8fd38fd3;
            mask2 = 0x9d7badf3;

            csi2dsi_sel = 0;

            vdelay_start = vdelay_end = config->tx_bits_per_lane / 1000 / 1000 * horz_period_ns / 1000 / 8 / 2 + 1;
        }
    }

    vdbg("brg_mode=0x%08x\n", brg_mode);
    vdbg("sel1_set=0x%08x\n", sel1_set);
    vdbg("sel2_set=0x%08x\n", sel2_set);
    vdbg("mask1=0x%08x\n", mask1);
    vdbg("mask2=0x%08x\n", mask2);
    vdbg("csi2dsi_sel=0x%08x\n", csi2dsi_sel);
    vdbg("vdelay_start=0x%08x\n", vdelay_start);
    vdbg("vdelay_end=0x%08x\n", vdelay_end);

    brg_mode |= CDSI_AL_TX_BRG_MODE_AL_TX_BRG_MASTER_SYNC_MODE_MASK;
    brg_mode |= CDSI_AL_TX_BRG_MODE_AL_TX_BRG_WAIT_INTERVAL_MODE_MASK;
    cdsi_write(dev, CDSI_AL_TX_BRG_MODE_OFFS, brg_mode);

    cdsi_write(dev, CDSI_AL_TX_BRG_TYPE_SEL1_SET_OFFS, sel1_set);
    cdsi_write(dev, CDSI_AL_TX_BRG_TYPE_SEL2_SET_OFFS, sel2_set);

    cdsi_write(dev, CDSI_AL_TX_BRG_TYPE_MASK1_SET_OFFS, mask1);
    cdsi_write(dev, CDSI_AL_TX_BRG_TYPE_MASK2_SET_OFFS, mask2);

    cdsi_write(dev, CDSI_AL_TX_BRG_PIC_COM_SET_OFFS, 0xf);

    cdsi_write(dev, CDSI_AL_TX_BRG_PIC_COM_VDELAYSTRCOUNT_OFFS, vdelay_start);
    cdsi_write(dev, CDSI_AL_TX_BRG_PIC_COM_VDELAYENDCOUNT_OFFS, vdelay_end);

    cdsi_write(dev, CDSI_AL_TX_BRG_PIC_COM_MAXFCNT_OFFS, 0);

    CDSI_WRITE(dev, CDSI_AL_TX_BRG_PIC_COM_3DCM_PLDT, AL_TX_BRG_REG_COM_3DCM_PLDT_A, config->vss_control_payload);
    cdsi_write(dev, CDSI_AL_TX_BRG_PIC_COM_3DCM_LINE_OFFS, 0);

    cdsi_write(dev, CDSI_AL_TX_BRG_PIC_SYN_SET_OFFS, 1);
    cdsi_write(dev, CDSI_AL_TX_BRG_PIC_SYN_LINE_OFFS, 0);

    cdsi_write(dev, CDSI_AL_TX_BRG_WAIT_CYCLE_SET_OFFS, 0x00100010);
    cdsi_write(dev, CDSI_AL_TX_BRG_VSYNC_LINE_SET_OFFS, 1);

    cdsi_write(dev, CDSI_AL_TX_BRG_STATUS_ENABLE_OFFS, 0xff);

    cdsi_write(dev, CDSI_AL_TX_BRG_UNIPRO_BYTESWAP_OFFS, 0x76543210);

    cdsi_write(dev, CDSI_AL_TX_BRG_ENABLE_OFFS, 1);

    cdsi_write(dev, CDSI_CDSITX_INTERRUPT_FUNC_ENABLE_00_OFFS, 0xf);
    if (config->mode == TSB_CDSI_MODE_DSI) {
       cdsi_write(dev, CDSI_CDSITX_INTERRUPT_FUNC_ENABLE_01_OFFS, 0x3f);
       cdsi_write(dev, CDSI_CDSITX_INTERRUPT_FUNC_ENABLE_02_OFFS, 0x007F1F1F);
    }
    cdsi_write(dev, CDSI_CDSITX_INTERRUPT_FUNC_ENABLE_03_OFFS, 0);

    cdsi_write(dev, CDSI_CDSITX_DPHY_RESET_OFFS, 1);

    CDSI_WRITE(dev, CDSI_CDSITX_PLL_CONFIG_00, SBS_DPHY_MPM_FBD, pll_fbd);
    CDSI_MODIFY(dev, CDSI_CDSITX_PLL_CONFIG_00, SBS_DPHY_CLM_LFBREN, 1);
    CDSI_MODIFY(dev, CDSI_CDSITX_PLL_CONFIG_00, SBS_DPHY_MPM_FRS, pll_frs);
    CDSI_MODIFY(dev, CDSI_CDSITX_PLL_CONFIG_00, SBS_DPHY_MPM_LBWS, 2);
    CDSI_MODIFY(dev, CDSI_CDSITX_PLL_CONFIG_00, SBS_DPHY_MPM_PRD, pll_prd);

    cdsi_write(dev, CDSI_CDSITX_PLL_CONFIG_02_OFFS, 0x00001680);

    cdsi_write(dev, CDSI_CDSITX_PLL_CONTROL_00_OFFS, 1);
    cdsi_write(dev, CDSI_CDSITX_PLL_CONTROL_01_OFFS, 1);

    /* Set the four LSBs for each lane enabled, then OR 0x10. */
    uint32_t lane_enable = 0x10 | ((1 << config->tx_num_lanes) - 1);
    cdsi_write(dev, CDSI_CDSITX_LANE_ENABLE_00_OFFS, lane_enable);
    cdsi_write(dev, CDSI_CDSITX_LANE_ENABLE_01_OFFS, lane_enable);
    cdsi_write(dev, CDSI_CDSITX_LANE_ENABLE_02_OFFS, lane_enable);

    cdsi_write(dev, CDSI_CDSITX_VREG_CONFIG_OFFS, 0x13);
    cdsi_write(dev, CDSI_CDSITX_VREG_CONTROL_OFFS, lane_enable);

    if (config->mode == TSB_CDSI_MODE_DSI) {
        cdsi_write(dev, CDSI_CDSITX_LPRX_CALIB_CONFIG_OFFS, 0xa);
        cdsi_write(dev, CDSI_CDSITX_LPRX_CALIB_CONTROL_OFFS, 1);
    }

    cdsi_write(dev, CDSI_CDSITX_CSI2DSI_SELECT_OFFS, csi2dsi_sel);

    uint32_t param2;
    uint32_t lp_tx_time_cnt;
    uint32_t twakeup_cnt;
    if (config->bta_enabled) {
        param2 = lp_tx_time_cnt = tx_table_row->lp_tx_time_cnt_bta;
        twakeup_cnt = tx_table_row->twakeup_cnt_bta;
    } else {
        param2 = lp_tx_time_cnt = tx_table_row->lp_tx_time_cnt;
        /* Mystery tweak */
        if (param2 == 2) {
            param2 = 3;
        }

        twakeup_cnt = tx_table_row->twakeup_cnt;
    }

    cdsi_write(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_00_OFFS, sys_cld_tclk_table_row->sys_cld_tclk_sel);
    cdsi_write(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_01_OFFS, 0x00022222);

    cdsi_write(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_02_OFFS, param2);

    CDSI_WRITE(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_03, SBS_DPHY_PPI_LPTXTIMECNT, lp_tx_time_cnt);

    CDSI_WRITE(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_04, SBS_DPHY_PPI_TCLK_PREPARECNT, tclk_prepare_cnt_table_row->tclk_prepare_cnt);
    CDSI_MODIFY(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_04, SBS_DPHY_PPI_TCLK_PRECNT, tclk_pre_cnt_table_row->tclk_pre_cnt);
    CDSI_MODIFY(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_04, SBS_DPHY_PPI_TCLK_PREZEROCNT, tclk_pre_zero_cnt_table_row->tclk_pre_zero_cnt);

    CDSI_WRITE(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_05, SBS_DPHY_PPI_TCLK_TRAILCNT, tclk_trail_cnt_table_row->tclk_trail_cnt);
    CDSI_MODIFY(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_05, SBS_DPHY_PPI_TCLK_EXITCNT, tclk_exit_cnt_table_row->tclk_exit_cnt);

    CDSI_WRITE(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_06, SBS_DPHY_PPI_THS_PREPARECNT, ths_prepare_cnt_table_row->ths_prepare_cnt);
    CDSI_MODIFY(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_06, SBS_DPHY_PPI_THS_PREZEROCNT, ths_pre_zero_cnt_table_row->ths_pre_zero_cnt);

    CDSI_WRITE(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_07, SBS_DPHY_PPI_THS_TRAILCNT, ths_trail_cnt_table_row->ths_trail_cnt);
    CDSI_MODIFY(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_07, SBS_DPHY_PPI_THS_EXITCNT, ths_exit_cnt_table_row->ths_exit_cnt);

    CDSI_WRITE(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_08, SBS_DPHY_PPI_TWAKEUPCNT, twakeup_cnt);

    uint32_t param9;
    if (config->continuous_clock) {
        param9 = tclk_post_cnt_table_row->tclk_post_cnt_continuous;
    } else {
        param9 = tclk_post_cnt_table_row->tclk_post_cnt;
    }
    CDSI_WRITE(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_09, SBS_DPHY_PPI_TCLK_POSTCNT, param9);

    CDSI_WRITE(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_10, SBS_DPHY_PPI_TXTAGOCNT, lp_tx_time_cnt);
    CDSI_MODIFY(dev, CDSI_CDSITX_GLOBAL_TIMING_PARAM_10, SBS_DPHY_PPI_RXTASURECNT, tx_table_row->rx_ta_sure_cnt);

    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_00_OFFS, 0x00000794);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_01_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_02_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_03_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_04_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_05_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_06_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_07_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_08_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_09_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_10_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_COUNT_CONFIG_11_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_00_OFFS, 0);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_01_OFFS, 0x0000001f);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_02_OFFS, 0x00022222);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_03_OFFS, 0x15555555);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_04_OFFS, 0x00022222);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_05_OFFS, 0x0000000f);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_06_OFFS, 0);

    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CONFIG_07, SBS_LINK_HS_CLK_MODE, config->continuous_clock);
    CDSI_MODIFY(dev, CDSI_CDSITX_SIDEBAND_CONFIG_07, SBS_LINK_DSI_BLANK_PKT_EN, config->blank_packet_enabled);

    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CONFIG_08, SBS_APF_DSI_DTVALID_POL, 1);

    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CONFIG_09, SBS_APF_DSI_SYNC_MODE, 1);
    if (config->mode == TSB_CDSI_MODE_DSI) {
        CDSI_MODIFY(dev, CDSI_CDSITX_SIDEBAND_CONFIG_09, SBS_APF_DSI_COMMAND_MODE, !config->video_mode);
    }
    CDSI_MODIFY(dev, CDSI_CDSITX_SIDEBAND_CONFIG_09, SBS_APF_DSI_BLANKPKT_EN, config->blank_packet_enabled);

    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_10_OFFS, 0);

    /* If we're in CSI mode both SIDEBAND_CONFIG_11 and
     * SIDEBAND_CONFIG_12 simply get set to 0
     */
    if (config->mode == TSB_CDSI_MODE_CSI) {
        cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_11_OFFS, 0);
        cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_12_OFFS, 0);
        vdbg("Setting SIDEBAND_CONFIG_11 and 12 both to 0\n");
    } else {

        /* Otherwise, we need to do some calculations. We start with
         * TLPX and LPS_period since these variables are used for both
         * SIDEBAND_CONFIG_11 and SIDEBAND_CONFIG_12
         */

        /* calculate TLPX */
        uint32_t tlpx;
        if (config->bta_enabled) {
            tlpx = (1 + tx_table_row->lp_tx_time_cnt_bta) *
                sys_cld_tclk_table_row->sys_cld_tclk_div;
        } else {
            tlpx = (1 + tx_table_row->lp_tx_time_cnt) *
                sys_cld_tclk_table_row->sys_cld_tclk_div;
        }
        vdbg("tlpx: %d\n", tlpx);

#if CONFIG_DEBUG_VERBOSE
        uint32_t ths_prepare = (ths_prepare_cnt_table_row->ths_prepare_cnt + 1) *
            sys_cld_tclk_table_row->sys_cld_tclk_div;
        vdbg("ths_prepare: %d\n", ths_prepare);
#endif

        uint32_t ths_pre_zero_m;
        switch (sys_cld_tclk_table_row->sys_cld_tclk_sel) {
        default:
        case 0:
            ths_pre_zero_m = 1;
            break;
        case 1:
            ths_pre_zero_m = 2;
            break;
        case 2:
            ths_pre_zero_m = 4;
            break;
        }
        vdbg("ths_pre_zero_m: %d\n", ths_pre_zero_m);
        uint32_t ths_pre_zero =
            (ths_pre_zero_cnt_table_row->ths_pre_zero_cnt + 1 + ths_pre_zero_m) *
            sys_cld_tclk_table_row->sys_cld_tclk_div + 43;
        vdbg("ths_pre_zero: %d\n", ths_pre_zero);
        uint32_t ths_trail = (ths_trail_cnt_table_row->ths_trail_cnt + 2) *
            sys_cld_tclk_table_row->sys_cld_tclk_div - 11;
        vdbg("ths_trail: %d\n", ths_trail);
        uint32_t ths_exit = (ths_exit_cnt_table_row->ths_exit_cnt + 3) *
            sys_cld_tclk_table_row->sys_cld_tclk_div;
        vdbg("ths_exit: %d\n", ths_exit);

        uint32_t tclk_post;
        if (config->continuous_clock) {
            tclk_post =
                (tclk_post_cnt_table_row->tclk_post_cnt_continuous + 2) *
                sys_cld_tclk_table_row->sys_cld_tclk_div + 2;
        } else {
            tclk_post = (tclk_post_cnt_table_row->tclk_post_cnt + 2) *
                sys_cld_tclk_table_row->sys_cld_tclk_div + 2;
        }
        vdbg("tclk_post: %d\n", tclk_post);
        uint32_t tclk_trail = (tclk_trail_cnt_table_row->tclk_trail_cnt + 3) *
            sys_cld_tclk_table_row->sys_cld_tclk_div - 3;
        vdbg("tclk_trail: %d\n", tclk_trail);
        uint32_t tclk_exit = (tclk_exit_cnt_table_row->tclk_exit_cnt + 2) *
            sys_cld_tclk_table_row->sys_cld_tclk_div;
        vdbg("tclk_exit: %d\n", tclk_exit);
#if CONFIG_DEBUG_VERBOSE
        uint32_t tclk_prepare =
            (tclk_prepare_cnt_table_row->tclk_prepare_cnt + 1) *
            sys_cld_tclk_table_row->sys_cld_tclk_div;
        vdbg("tclk_prepare: %d\n", tclk_prepare);
#endif
        uint32_t tclk_pre_zero =
            (tclk_pre_zero_cnt_table_row->tclk_pre_zero_cnt + 2) *
            sys_cld_tclk_table_row->sys_cld_tclk_div + 3;
        vdbg("tclk_pre_zero: %d\n", tclk_pre_zero);
        uint32_t tclk_pre = (tclk_pre_cnt_table_row->tclk_pre_cnt + 4) *
            sys_cld_tclk_table_row->sys_cld_tclk_div - 4;
        vdbg("tclk_pre: %d\n", tclk_pre);

        /* calculate LPS_period */
        uint32_t lps_period;
        if (config->continuous_clock) {
            lps_period = ths_trail + ths_exit + tlpx + ths_pre_zero + 8;
        } else {
            lps_period = ths_trail + tclk_post + tclk_trail + tclk_exit +
                tlpx + tclk_pre_zero + tclk_pre + tlpx + ths_pre_zero + 8;
        }
        vdbg("lps_period: %d\n", lps_period);

        /* SIDEBAND_CONFIG_11 - Horizontal blank width */
        const uint32_t hsa = (uint32_t) ((hsck + 9) / 10);
        vdbg("hsa: %d\n", hsa);

        uint32_t hsa_cnt_dsi;
        if (config->blank_packet_enabled) {
            uint32_t hsa_blapkt_size = (uint32_t) ((hsa * config->tx_num_lanes + 7) / 8);
            vdbg("hsa_blapkt_size: %d\n", hsa_blapkt_size);

            if (hsa_blapkt_size < 7) {
                hsa_blapkt_size = 7;
                vdbg("hsa_blapkt_size: %d\n", hsa_blapkt_size);
            }
            if (hsa_blapkt_size % 4) {
                hsa_blapkt_size = hsa_blapkt_size + 4 - (hsa_blapkt_size % 4);
                vdbg("hsa_blapkt_size: %d\n", hsa_blapkt_size);
            }
            if ((hsa_blapkt_size - 6) == 0) {
                hsa_cnt_dsi = 1;
            } else {
                hsa_cnt_dsi = hsa_blapkt_size - 6;
            }

#if CONFIG_DEBUG_VERBOSE
            uint32_t hsa_blk = (uint32_t) ((hsa_blapkt_size * 8 /
                config->tx_num_lanes * 10 + 9) / 10);
            vdbg("hsa_blk=%d\n", hsa_blk);
#endif
        } else {
            const int32_t hsa_cnt_hs =
                (uint32_t) ((((hsa + 32 / config->tx_num_lanes) / 8 * 10 + 9) / 10) - 3);
            vdbg("hsa_cnt_hs: %d\n", hsa_cnt_hs);

            const int32_t hsa_cnt_lp =
                (uint32_t) ((((lps_period + 32 / config->tx_num_lanes) /
                        8 * 10 + 9) / 10) - 3);
            vdbg("hsa_cnt_lp: %d\n", hsa_cnt_lp);

            int32_t hsa_cnt_max = MAX(hsa_cnt_hs, hsa_cnt_lp);
            vdbg("hsa_cnt_max: %d\n", hsa_cnt_max);

            if (hsa_cnt_max == 0) {
                hsa_cnt_dsi = 1;
            } else {
                hsa_cnt_dsi = hsa_cnt_max;
            }
        }

        vdbg("hsa_cnt_dsi: %d\n", hsa_cnt_dsi);
        CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CONFIG_11,
            SBS_APF_DSI_HSA_CNT, hsa_cnt_dsi);

        /* SIDEBAND_CONFIG_12 - Horizontal back porch */
        const uint32_t hbp = (uint32_t) ((config->horizontal_back_porch *
                (config->bpp / config->tx_num_lanes) * 10 + 9) / 10);
        vdbg("hbp: %d\n", hbp);

        const uint32_t hbp_cnt_hs =
            (uint32_t) ((((hbp + 32 / config->tx_num_lanes) / 8 * 10 + 9) / 10) - 3);
        vdbg("hbp_cnt_hs: %d\n", hbp_cnt_hs);

        const uint32_t hbp_cnt_lp =
            (uint32_t) ((((lps_period + 32 / config->tx_num_lanes) /
                    8 * 10 + 9) / 10) - 3);
        vdbg("hbp_cnt_lp: %d\n", hbp_cnt_lp);

        uint32_t hbp_cnt_max = MAX(hbp_cnt_hs, hbp_cnt_lp);
        vdbg("hbp_cnt_max: %d\n", hbp_cnt_max);

        uint32_t hbp_cnt_dsi;
        if (config->blank_packet_enabled) {
            uint32_t hbp_blapkt_size = (uint32_t) ((hbp * config->tx_num_lanes /
                    8 * 10 + 9) / 10);
            vdbg("hbp_blapkt_size: %d\n", hbp_blapkt_size);

            if (hbp_blapkt_size < 7) {
                hbp_blapkt_size = 7;
                vdbg("hbp_blapkt_size: %d\n", hbp_blapkt_size);
            }

            if (hbp_blapkt_size % 4) {
                hbp_blapkt_size = hbp_blapkt_size + 4 - (hbp_blapkt_size % 4);
                vdbg("hbp_blapkt_size: %d\n", hbp_blapkt_size);
            }

            if ((hbp_blapkt_size - 6) == 0) {
                hbp_cnt_dsi = 1;
            } else {
                hbp_cnt_dsi = hbp_blapkt_size - 6;
            }

#if CONFIG_DEBUG_VERBOSE
            uint32_t hbp_blk = (uint32_t) ((hbp_blapkt_size * 8 /
                config->tx_num_lanes * 10 + 9) / 10);
            vdbg("hbp_blk=%d\n", hbp_blk);
#endif
        } else {
            if (hbp_cnt_max == 0) {
                hbp_cnt_dsi = 1;
            } else {
                hbp_cnt_dsi = hbp_cnt_max;
            }
        }

        vdbg("hbp_cnt_dsi=%d\n", hbp_cnt_dsi);
        CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CONFIG_12,
            SBS_APF_DSI_HBP_CNT, hbp_cnt_dsi);
    }


    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_13_OFFS, 0);
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_14_OFFS, 0);

    /* Maximum line width (in bytes). */
    cdsi_write(dev, CDSI_CDSITX_SIDEBAND_CONFIG_15_OFFS, 0x1800);

    int ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_00,
                    INT_DPHY_LOCKUPDONE, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: DHPY lockup failed\n");
        return ret;
    }

    if (config->mode == TSB_CDSI_MODE_DSI) {
        ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_00,
                        INT_DPHY_AUTOCALDONE, CDSI_DEFAULT_RETRIES);
        if (ret < 0) {
            dbg("ERROR: DPHY auto-cal failed\n");
            return ret;
        }
    }
    ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_00,
                    INT_DPHY_HSTXVREGRDY, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: DHPY HSTX VREG Ready failed\n");
        return ret;
    }

    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_00, SBD_DPHY_MPM_CKEN, 1);
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_01, SBD_DPHY_MPM_PLLINTCKEN, 1);
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_02, SBD_DPHY_HSTXCLKENABLE, 1);
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_03, SBD_DPHY_STARTPPI, 1);

    ret = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_INTERRUPT_STATUS_00,
                    INT_DPHY_LINEINITDONE, CDSI_DEFAULT_RETRIES);
    if (ret < 0) {
        dbg("ERROR: DHPY line init failed\n");
        return ret;
    }

    cdsi_clear_tx_status(dev);
    cdsi_write(dev, CDSI_AL_TX_BRG_STATUS_OFFS, 0xffffffff);

    CDSI_WRITE(dev, CDSI_CDSITX_LINK_RESET, LINK_RESET_N, 1);
    CDSI_WRITE(dev, CDSI_CDSITX_APF_RESET, APF_RESET_N, 1);
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_04, SBD_APF_CDSITX_START, 1);

    /* Enable PIC_SYN_* and PIC_COM_*. */
    cdsi_write(dev, CDSI_AL_TX_BRG_VPARAM_UPDATE_OFFS, 3);

    return 0;
}

int cdsi_tx_start(struct cdsi_dev *dev) {
    vdbg("\n");

    CDSI_WRITE(dev, CDSI_AL_TX_BRG_PIC_COM_START, AL_TX_BRG_REG_COM_START_A, 1);

    return 0;
}

int cdsi_tx_stop(struct cdsi_dev *dev) {
    vdbg("\n");

    /* Stop the pixel interface. */
    CDSI_MODIFY(dev, CDSI_AL_TX_BRG_PIC_COM_START, AL_TX_BRG_REG_COM_START_A, 0);

    /* Wait until the VH (pixel) and packet interfaces are stopped. */
    int result = CDSI_READ_UNTIL_CLR_RETRIES(dev,
                    CDSI_CDSITX_SIDEBAND_STATUS_05, SBO_APF_VHIF_BUSY,
                    CDSI_DEFAULT_RETRIES);
    if (result) {
        // This failure is OK if '!config->video_mode'.
        dbg("WARNING: Failed to stop VHIF\n");
    }

    result = CDSI_READ_UNTIL_CLR_RETRIES(dev, CDSI_CDSITX_SIDEBAND_STATUS_05,
                    SBO_APF_PKTIF_BUSY, CDSI_DEFAULT_RETRIES);
    if (result) {
        dbg("ERROR: Failed to stop PKTIF\n");
    }

    /* Reset APF. */
    CDSI_MODIFY(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_04, SBD_APF_CDSITX_START, 0);
    CDSI_MODIFY(dev, CDSI_CDSITX_APF_RESET, APF_RESET_N, 0);
    CDSI_MODIFY(dev, CDSI_CDSITX_APF_RESET, APF_RESET_N, 1);
    CDSI_MODIFY(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_04, SBD_APF_CDSITX_START, 1);

    return 0;
}

int cdsi_uninitialize_tx(struct cdsi_dev *dev, const struct cdsi_config *config) {
    vdbg("\n");

    /* Stop the pixel interface. */
    CDSI_MODIFY(dev, CDSI_AL_TX_BRG_PIC_COM_START, AL_TX_BRG_REG_COM_START_A, 0);

    /* Wait until the pixel interface is stopped. */
    int result = CDSI_READ_UNTIL_CLR_RETRIES(dev,
                    CDSI_CDSITX_SIDEBAND_STATUS_05, SBO_APF_VHIF_BUSY,
                    CDSI_DEFAULT_RETRIES);
    if (result) {
        // This failure is OK if '!config->video_mode'.
        dbg("WARNING: Failed to stop VHIF\n");
    }

    /* Disable TX bridge. */
    CDSI_WRITE(dev, CDSI_AL_TX_BRG_ENABLE, AL_TX_BRG_EN, 0);

    /* Reset TX bridge. */
    CDSI_WRITE(dev, CDSI_AL_TX_BRG_SOFT_RESET, AL_TX_BRG_SOFT_RESET_N, 0);

    /* Stop the APF . */
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_04, SBD_APF_CDSITX_START, 0);

    /* Disable PPI */
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_03, SBD_DPHY_STARTPPI, 0);

    /* Disable clocks for LINK and APF. */
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_02, SBD_DPHY_HSTXCLKENABLE, 0);

    /* Reset APF. */
    CDSI_WRITE(dev, CDSI_CDSITX_APF_RESET, APF_RESET_N, 0);

    /* Reset LINK. */
    CDSI_WRITE(dev, CDSI_CDSITX_LINK_RESET, LINK_RESET_N, 0);

    /* Initialize PPI. */
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CONTROL_00, SBD_DPHY_PPI_SYSINIT, 1);

    /* Enable clocks for LINK and APF. */
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_02, SBD_DPHY_HSTXCLKENABLE, 1);

    /* Wait until D-Phy clock is stopped. */
    result = CDSI_READ_UNTIL_SET_RETRIES(dev, CDSI_CDSITX_SIDEBAND_STATUS_07,
                    SBO_PPIIF_CLM_STOPSTATE, CDSI_DEFAULT_RETRIES);

    /* Disable clocks for LINK and APF. */
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_02, SBD_DPHY_HSTXCLKENABLE, 0);
    if (result) {
        dbg("ERROR: Failed to stop LINK or APF\n");
    }

    /* Disable voltage regulator. */
    uint32_t mask0 = CDSI_CDSITX_VREG_CONTROL_SBS_DPHY_D0M_HSTXVREGEN_MASK|
                     CDSI_CDSITX_VREG_CONTROL_SBS_DPHY_D1M_HSTXVREGEN_MASK|
                     CDSI_CDSITX_VREG_CONTROL_SBS_DPHY_D2M_HSTXVREGEN_MASK|
                     CDSI_CDSITX_VREG_CONTROL_SBS_DPHY_D3M_HSTXVREGEN_MASK|
                     CDSI_CDSITX_VREG_CONTROL_SBS_DPHY_CLM_HSTXVREGEN_MASK;
    cdsi_modify(dev, CDSI_CDSITX_VREG_CONTROL_OFFS, mask0, 0);

    /* Disable D-Phy. */
    CDSI_WRITE(dev, CDSI_CDSITX_LPRX_CALIB_CONTROL, SBD_DPHY_CALIB_START, 0);

    /* Disable PLL */
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_INIT_CONTROL_00, SBD_DPHY_MPM_CKEN, 0);
    CDSI_WRITE(dev, CDSI_CDSITX_PLL_CONTROL_01, SBD_DPHY_MP_ENABLE, 0);

    /* Clear interrupts. */
    cdsi_clear_tx_status(dev);

    /* Release init for PPI. */
    CDSI_WRITE(dev, CDSI_CDSITX_SIDEBAND_CONTROL_00, SBD_DPHY_PPI_SYSINIT, 0);

    return 0;
}
