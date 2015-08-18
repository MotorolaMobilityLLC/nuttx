/*
 * Copyright (c) 2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <nuttx/pwm.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <arch/tsb/cdsi.h>
#include <arch/board/cdsi0_offs_def.h>
#include <arch/board/cdsi0_reg_def.h>
#include <arch/tsb/pwm.h>
#include <arch/tsb/gpio.h>

#include <arch/board/lg4892.h>

#include <apps/greybus-utils/utils.h>

#define AL_TX_BRG_MODE_VAL                    0x00000090
#define AL_TX_BRG_WAIT_CYCLE_SET_VAL          0x00100010
#define AL_TX_BRG_VSYNC_LINE_SET_VAL          0x00000001
#define AL_TX_BRG_TYPE_SEL1_SET_VAL           0x8FFF8FFD
#define AL_TX_BRG_TYPE_SEL2_SET_VAL           0x9FFFAFFD
#define AL_TX_BRG_TYPE_MASK1_SET_VAL          0x8FD38FD1
#define AL_TX_BRG_TYPE_MASK2_SET_VAL          0x9D7BADF1
#define AL_TX_BRG_PIC_COM_SET_VAL             0x0000000F
#define AL_TX_BRG_PIC_COM_VDELAYSTRCOUNT_VAL  0x000005A3
#define AL_TX_BRG_PIC_COM_VDELAYENDCOUNT_VAL  0x00000000
#define AL_TX_BRG_PIC_COM_MAXFCNT_VAL         0x00000000
#define AL_TX_BRG_PIC_COM_3DCM_PLDT_VAL       0x00000000
#define AL_TX_BRG_PIC_COM_3DCM_LINE_VAL       0x00000000
#define AL_TX_BRG_PIC_SYN_SET_VAL             0x00000001
#define AL_TX_BRG_PIC_SYN_LINE_VAL            0x00000000
#define AL_TX_BRG_STATUS_ENABLE_VAL           0x000000FF
#define AL_TX_BRG_CDSITX_MODE_VAL             0x00000001
#define AL_TX_BRG_UNIPRO_BYTESWAP_VAL         0x76543210
#define CDSITX_INTERRUPT_FUNC_ENABLE_00_VAL   0x0000000F
#define CDSITX_INTERRUPT_FUNC_ENABLE_01_VAL   0x0000003F
#define CDSITX_INTERRUPT_FUNC_ENABLE_02_VAL   0x007F1F1F
#define CDSITX_INTERRUPT_FUNC_ENABLE_03_VAL   0x00000000
#define CDSITX_POWER_SW_VAL                   0x00000007
#define CDSITX_DPHY_RESET_VAL                 0x00000001
#define CDSITX_PLL_CONFIG_00_VAL              0x0320107C
#define CDSITX_PLL_CONFIG_02_VAL              0x00001680
#define CDSITX_PLL_CONTROL_00_VAL             0x00000001
#define CDSITX_PLL_CONTROL_01_VAL             0x00000001
#define CDSITX_LANE_ENABLE_00_VAL             0x0000001F
#define CDSITX_LANE_ENABLE_01_VAL             0x0000001F
#define CDSITX_LANE_ENABLE_02_VAL             0x0000001F
#define CDSITX_VREG_CONFIG_VAL                0x00000013
#define CDSITX_VREG_CONTROL_VAL               0x0000001F
#define CDSITX_LPRX_CALIB_CONFIG_VAL          0x0000000A
#define CDSITX_LPRX_CALIB_CONTROL_VAL         0x00000001
#define CDSITX_CSI2DSI_SELECT_VAL             0x00000000
#define CDSITX_GLOBAL_TIMING_PARAM_00_VAL     0x00000000
#define CDSITX_GLOBAL_TIMING_PARAM_01_VAL     0x00022222
#define CDSITX_GLOBAL_TIMING_PARAM_02_VAL     0x00000000
#define CDSITX_GLOBAL_TIMING_PARAM_03_VAL     0x00000009
#define CDSITX_GLOBAL_TIMING_PARAM_04_VAL     0x00150005
#define CDSITX_GLOBAL_TIMING_PARAM_05_VAL     0x000A0003
#define CDSITX_GLOBAL_TIMING_PARAM_06_VAL     0x00060006
#define CDSITX_GLOBAL_TIMING_PARAM_07_VAL     0x000B0006
#define CDSITX_GLOBAL_TIMING_PARAM_08_VAL     0x00000F88
#define CDSITX_GLOBAL_TIMING_PARAM_09_VAL     0x00000003
#define CDSITX_GLOBAL_TIMING_PARAM_10_VAL     0x0009000A
#define CDSITX_SIDEBAND_COUNT_CONFIG_00_VAL   0x00000794
#define CDSITX_SIDEBAND_COUNT_CONFIG_01_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_02_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_03_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_04_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_05_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_06_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_07_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_08_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_09_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_10_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_COUNT_CONFIG_11_VAL   0xFFFFFFFF
#define CDSITX_SIDEBAND_CONFIG_00_VAL         0x00000000
#define CDSITX_SIDEBAND_CONFIG_01_VAL         0x0000001F
#define CDSITX_SIDEBAND_CONFIG_02_VAL         0x00022222
#define CDSITX_SIDEBAND_CONFIG_03_VAL         0x15555555
#define CDSITX_SIDEBAND_CONFIG_04_VAL         0x00022222
#define CDSITX_SIDEBAND_CONFIG_05_VAL         0x0000000F
#define CDSITX_SIDEBAND_CONFIG_06_VAL         0x00000000
#define CDSITX_SIDEBAND_CONFIG_07_VAL         0x00000082
#define CDSITX_SIDEBAND_CONFIG_08_VAL         0x00000004
#define CDSITX_SIDEBAND_CONFIG_09_VAL         0x00002001
#define CDSITX_SIDEBAND_CONFIG_10_VAL         0x00000000
#define CDSITX_SIDEBAND_CONFIG_11_VAL         0x0000001A
#define CDSITX_SIDEBAND_CONFIG_12_VAL         0x00000172
#define CDSITX_SIDEBAND_CONFIG_13_VAL         0x00000000
#define CDSITX_SIDEBAND_CONFIG_14_VAL         0x00000000
#define CDSITX_SIDEBAND_CONFIG_15_VAL         0x00001800
#define CDSITX_PISO_ENABLE_VAL                0x0000001F
#define CDSITX_SIDEBAND_INIT_CONTROL_00_VAL   0x00000001
#define CDSITX_SIDEBAND_INIT_CONTROL_01_VAL   0x00000001
#define CDSITX_SIDEBAND_INIT_CONTROL_02_VAL   0x00000001
#define CDSITX_SIDEBAND_INIT_CONTROL_03_VAL   0x00000001
#define CDSITX_SIDEBAND_INIT_CONTROL_04_VAL   0x00000001
#define CDSITX_LINK_RESET_VAL                 0x00000001
#define CDSITX_APF_RESET_VAL                  0x00000001

typedef struct {
    int size;
    int *packet;
} init_code;

static int comB0[] = { 0x0000ACB0 };
static int com11[] = { 0x00000011 };
static int comB8[] = { 0x232525B8, 0x00807FE9 };
static int com29[] = { 0x00000029 };

static init_code LG4892[] = {
    {2, comB0},
    {1, com11},
    {7, comB8},
    {1, com29}
};

static void lg4892_cdsi_init_cmd(struct cdsi_dev *dev)
{
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_00_OFFS, 0x0000000F);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_OFFS, 0x0000003F);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_OFFS, 0x007F1F1F);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_03_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_POWER_SW_OFFS, 0x00000007);
    cdsi_write(dev, CDSI0_CDSITX_DPHY_RESET_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONFIG_00_OFFS, 0x0320107C);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONFIG_02_OFFS, 0x00001680);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONTROL_00_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONTROL_01_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_00_OFFS, 0x0000001F);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_01_OFFS, 0x0000001F);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_02_OFFS, 0x0000001F);
    cdsi_write(dev, CDSI0_CDSITX_VREG_CONFIG_OFFS, 0x00000013);
    cdsi_write(dev, CDSI0_CDSITX_VREG_CONTROL_OFFS, 0x0000001F);
    cdsi_write(dev, CDSI0_CDSITX_LPRX_CALIB_CONFIG_OFFS, 0x0000000A);
    cdsi_write(dev, CDSI0_CDSITX_LPRX_CALIB_CONTROL_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_CSI2DSI_SELECT_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_00_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_01_OFFS, 0x00022222);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_02_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_03_OFFS, 0x00000009);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_04_OFFS, 0x00150005);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_05_OFFS, 0x000A0003);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_06_OFFS, 0x00060006);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_07_OFFS, 0x000B0006);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_08_OFFS, 0x00000F88);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_09_OFFS, 0x00000003);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_10_OFFS, 0x0009000A);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_00_OFFS, 0x00000794);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_01_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_02_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_03_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_04_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_05_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_06_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_07_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_08_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_09_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_10_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_11_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_00_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_01_OFFS, 0x0000001F);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_02_OFFS, 0x00022222);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_03_OFFS, 0x15555555);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_04_OFFS, 0x00022222);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_05_OFFS, 0x0000000F);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_06_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_07_OFFS, 0x00000082);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_08_OFFS, 0x00000004);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_09_OFFS, 0x00002001);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_10_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_11_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_12_OFFS, 0x0000006E);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_13_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_14_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_15_OFFS, 0x00000870);
    cdsi_write(dev, CDSI0_CDSITX_PISO_ENABLE_OFFS, 0x0000001F);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_00_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_01_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_02_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_03_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_LINK_RESET_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_APF_RESET_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_04_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_AL_TX_BRG_MODE_OFFS, 0x00000090);
    cdsi_write(dev, CDSI0_AL_TX_BRG_WAIT_CYCLE_SET_OFFS, 0x00100010);
    cdsi_write(dev, CDSI0_AL_TX_BRG_VSYNC_LINE_SET_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_SEL1_SET_OFFS, 0x8FFF8FFD);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_SEL2_SET_OFFS, 0x9FFFAFFD);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_MASK1_SET_OFFS, 0x8FD38FD1);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_MASK2_SET_OFFS, 0x9D7BADF1);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_SET_OFFS, 0x0000000F);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_VDELAYSTRCOUNT_OFFS, 0x000005A3);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_VDELAYENDCOUNT_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_MAXFCNT_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_3DCM_PLDT_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_3DCM_LINE_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_SET_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_LINE_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_AL_TX_BRG_STATUS_ENABLE_OFFS, 0x000000FF);
    cdsi_write(dev, CDSI0_AL_TX_BRG_CDSITX_MODE_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_OFFS, 0x00000000);
}

static void lg4892_cdsi_init_video(struct cdsi_dev *dev)
{
    unsigned int rdata;

    cdsi_write(dev, CDSI0_AL_TX_BRG_CDSITX_MODE_OFFS,
               AL_TX_BRG_CDSITX_MODE_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_MODE_OFFS, AL_TX_BRG_MODE_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_SEL1_SET_OFFS,
               AL_TX_BRG_TYPE_SEL1_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_SEL2_SET_OFFS,
               AL_TX_BRG_TYPE_SEL2_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_MASK1_SET_OFFS,
               AL_TX_BRG_TYPE_MASK1_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_MASK2_SET_OFFS,
               AL_TX_BRG_TYPE_MASK2_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_SET_OFFS,
               AL_TX_BRG_PIC_COM_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_VDELAYSTRCOUNT_OFFS, 0x00000874);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_VDELAYENDCOUNT_OFFS, 0x00000000);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_MAXFCNT_OFFS,
               AL_TX_BRG_PIC_COM_MAXFCNT_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_3DCM_PLDT_OFFS,
               AL_TX_BRG_PIC_COM_3DCM_PLDT_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_3DCM_LINE_OFFS,
               AL_TX_BRG_PIC_COM_3DCM_LINE_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_SET_OFFS,
               AL_TX_BRG_PIC_SYN_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_LINE_OFFS,
               AL_TX_BRG_PIC_SYN_LINE_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_WAIT_CYCLE_SET_OFFS,
               AL_TX_BRG_WAIT_CYCLE_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_VSYNC_LINE_SET_OFFS,
               AL_TX_BRG_VSYNC_LINE_SET_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_STATUS_ENABLE_OFFS,
               AL_TX_BRG_STATUS_ENABLE_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_CDSITX_MODE_OFFS,
               AL_TX_BRG_CDSITX_MODE_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_OFFS,
               AL_TX_BRG_UNIPRO_BYTESWAP_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_MODE_OFFS, 0x00000091);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_00_OFFS,
               CDSITX_INTERRUPT_FUNC_ENABLE_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_OFFS,
               CDSITX_INTERRUPT_FUNC_ENABLE_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_OFFS,
               CDSITX_INTERRUPT_FUNC_ENABLE_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_03_OFFS,
               CDSITX_INTERRUPT_FUNC_ENABLE_03_VAL);
    cdsi_write(dev, CDSI0_CDSITX_POWER_SW_OFFS, CDSITX_POWER_SW_VAL);
    cdsi_write(dev, CDSI0_CDSITX_DPHY_RESET_OFFS, CDSITX_DPHY_RESET_VAL);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONFIG_00_OFFS, 0x0120105C);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONFIG_02_OFFS, CDSITX_PLL_CONFIG_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONTROL_00_OFFS,
               CDSITX_PLL_CONTROL_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONTROL_01_OFFS,
               CDSITX_PLL_CONTROL_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_00_OFFS,
               CDSITX_LANE_ENABLE_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_01_OFFS,
               CDSITX_LANE_ENABLE_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_02_OFFS,
               CDSITX_LANE_ENABLE_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_VREG_CONFIG_OFFS, CDSITX_VREG_CONFIG_VAL);
    cdsi_write(dev, CDSI0_CDSITX_VREG_CONTROL_OFFS, CDSITX_VREG_CONTROL_VAL);
    cdsi_write(dev, CDSI0_CDSITX_CSI2DSI_SELECT_OFFS,
               CDSITX_CSI2DSI_SELECT_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_00_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_01_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_02_OFFS,
               CDSITX_GLOBAL_TIMING_PARAM_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_03_OFFS, 0x0000000C);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_04_OFFS, 0x00210008);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_05_OFFS, 0x00100005);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_06_OFFS, 0x000C0009);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_07_OFFS, 0x00100008);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_08_OFFS, 0x0000131F);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_09_OFFS, 0x00000007);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_10_OFFS, 0x000C000E);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_00_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_01_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_02_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_03_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_03_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_04_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_04_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_05_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_05_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_06_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_06_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_07_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_07_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_08_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_08_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_09_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_09_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_10_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_10_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_11_OFFS,
               CDSITX_SIDEBAND_COUNT_CONFIG_11_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_00_OFFS,
               CDSITX_SIDEBAND_CONFIG_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_01_OFFS,
               CDSITX_SIDEBAND_CONFIG_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_02_OFFS,
               CDSITX_SIDEBAND_CONFIG_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_03_OFFS,
               CDSITX_SIDEBAND_CONFIG_03_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_04_OFFS,
               CDSITX_SIDEBAND_CONFIG_04_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_05_OFFS,
               CDSITX_SIDEBAND_CONFIG_05_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_06_OFFS,
               CDSITX_SIDEBAND_CONFIG_06_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_07_OFFS,
               CDSITX_SIDEBAND_CONFIG_07_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_08_OFFS,
               CDSITX_SIDEBAND_CONFIG_08_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_09_OFFS,
               CDSITX_SIDEBAND_CONFIG_09_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_10_OFFS,
               CDSITX_SIDEBAND_CONFIG_10_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_11_OFFS, 0x00000162);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_12_OFFS, 0x0000028A);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_13_OFFS,
               CDSITX_SIDEBAND_CONFIG_13_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_14_OFFS,
               CDSITX_SIDEBAND_CONFIG_14_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_15_OFFS,
               CDSITX_SIDEBAND_CONFIG_15_VAL);
    rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_MASK_00_OFFS);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_MASK_00_OFFS, rdata & 0xFFFFFFF7);
    rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    while ((rdata & 0x8) != 0x8) {
        rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    }
    while ((rdata & 0x4) != 0x4) {
        rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    }
    while ((rdata & 0x2) != 0x2) {
        rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    }
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0x0000000E);
    cdsi_write(dev, CDSI0_CDSITX_PISO_ENABLE_OFFS, CDSITX_PISO_ENABLE_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_00_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_00_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_01_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_01_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_02_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_02_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_03_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_03_VAL);
    rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    while ((rdata & 0x1) != 0x1) {
        rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
    }

    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0x0000000E);
    cdsi_write(dev, CDSI0_AL_RX_BRG_MODE_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_10_OFFS, 0x000F0001);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_01_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_02_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_03_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_04_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_05_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_06_OFFS, 0xFFFFFFFF);
    cdsi_write(dev, CDSI0_CDSITX_LINK_RESET_OFFS, CDSITX_LINK_RESET_VAL);
    cdsi_write(dev, CDSI0_CDSITX_APF_RESET_OFFS, CDSITX_APF_RESET_VAL);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_04_OFFS,
               CDSITX_SIDEBAND_INIT_CONTROL_04_VAL);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_SET_OFFS, 0x00000001);
    cdsi_write(dev, CDSI0_AL_TX_BRG_VPARAM_UPDATE_OFFS, 0x00000003);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_START_OFFS, 0x00000001);
    lowsyslog("--- CDSITX : Initialization completed ---\n");
}

static int lg4892_init_lcd(struct cdsi_dev *dev)
{
    uint32_t rdata;
    int ret = 0;
    int i, j;

    for (i = 0; i < (sizeof(LG4892) / sizeof(LG4892[0])); i++) {
        lowsyslog("count - %d\n", i);
        while (1) {
            if (!(cdsi_read(dev, CDSI0_CDSITX_SIDEBAND_STATUS_05_OFFS) &
                  CDSI0_CDSITX_SIDEBAND_STATUS_05_SBO_APF_VHIF_BUSY_MASK)) {
                break;
            }
        }

        while (1) {
            if (!(cdsi_read(dev, CDSI0_CDSITX_SIDEBAND_STATUS_05_OFFS) &
                  CDSI0_CDSITX_SIDEBAND_STATUS_05_APF_CMD_BUSY_MASK)) {
                break;
            }
        }

        while (1) {
            if (!(cdsi_read(dev, CDSI0_CDSITX_SIDEBAND_STATUS_06_OFFS) &
                  CDSI0_CDSITX_INTERRUPT_STATUS_06_INT_APF_CMD_DONE_MASK)) {
                break;
            }
        }

        cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_01_OFFS, 0x4);

        rdata = cdsi_read(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_02_OFFS);
        rdata &= ~CDSI0_CDSITX_SIDEBAND_CMDIF_02_APF_CMD_PKT_DI_MASK;
        cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_02_OFFS, rdata);

        rdata |= 0x39 << CDSI0_CDSITX_SIDEBAND_CMDIF_02_APF_CMD_PKT_DI_SHIFT;
        cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_02_OFFS, rdata);

        rdata &= ~CDSI0_CDSITX_SIDEBAND_CMDIF_02_APF_CMD_PKT_WC_MASK;
        cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_02_OFFS, rdata);
        rdata |=
            LG4892[i].size <<
            CDSI0_CDSITX_SIDEBAND_CMDIF_02_APF_CMD_PKT_WC_SHIFT;
        cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_02_OFFS, rdata);
        rdata = cdsi_read(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_00_OFFS);
        rdata |= CDSI0_CDSITX_SIDEBAND_CMDIF_00_APF_CMD_REQ_MASK;
        cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_00_OFFS, rdata);
        if (LG4892[i].size % 4) {
            for (j = 0; j < ((LG4892[i].size / 4) + 1); j++) {
                cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_03_OFFS,
                           LG4892[i].packet[j]);
            }
        } else {
            for (j = 0; j < (LG4892[i].size / 4); j++) {
                cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CMDIF_03_OFFS,
                           LG4892[i].packet[j]);
            }
        }

        while (!(cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_06_OFFS) &
                 CDSI0_CDSITX_INTERRUPT_STATUS_06_INT_APF_CMD_DONE_MASK)) ;

        rdata = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_06_OFFS);
        rdata |= CDSI0_CDSITX_INTERRUPT_STATUS_06_INT_APF_CMD_DONE_MASK;
        cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_06_OFFS, rdata);

        if (i == 1) {
            for (j = 0; j < 0x170A390; j++) {
                __asm("nop");
            }
        }
    }

    return ret;
}

static void pwm_enable(void)
{
    struct pwm_lowerhalf_s *pwm;
    struct pwm_info_s pwm_info;
    int ret;
    int fd;

    pwm_info.frequency = 300000;
    pwm_info.duty = 32767;

    pwm = tsb_pwminitialize(0);
    if (!pwm) {
        lowsyslog("pwminitialize failed\n");
        return;
    }

    if (pwm_register("/dev/pwm0", pwm)) {
        lowsyslog("Can't register /dev/pwm0\n");
        return;
    }

    fd = open("/dev/pwm0", O_RDONLY);
    ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&pwm_info);
    if (ret < 0) {
        lowsyslog("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n",
                  errno);
    }

    ret = ioctl(fd, PWMIOC_START, 0);
    if (ret < 0) {
        lowsyslog("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
    }
}

void lg4892_dsi_init(struct cdsi_dev *dev)
{
    lg4892_cdsi_init_cmd(dev);
    lg4892_init_lcd(dev);
    lg4892_cdsi_init_video(dev);
}

struct display_panel lg4892_panel = {
    .cdsi_panel_init = lg4892_dsi_init,
};

int display_init(void)
{
    struct i2c_dev_s *dev;

    dev = up_i2cinitialize(0);
    if (!dev) {
        lowsyslog("%s(): Failed to get I/O Expander I2C bus 0\n", __func__);
        return -ENODEV;
    }
    gb_i2c_set_dev(dev);

    pwm_enable();

    lg4892_gpio_init();
    lg4892_enable(1);

    dsi_initialize(&lg4892_panel, TSB_CDSI0, TSB_CDSI_TX);

    return 0;
}
