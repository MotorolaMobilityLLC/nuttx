/*
 * Copyright (c) 2015 Google Inc.
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>

#include <nuttx/gpio.h>
#include <nuttx/wqueue.h>
#include <arch/tsb/cdsi.h>
#include <arch/tsb/gpio.h>
#include <arch/board/cdsi0_offs_def.h>
#include <arch/board/cdsi0_reg_def.h>
#ifdef CONFIG_FOR_GOOGLE_IO_DEMO
#include <arch/board/ov5645.h>
#endif

#define AL_RX_BRG_MODE_VAL                              0x00000003
#define AL_RX_BRG_CSI_INFO_VAL                          0x00000000
#define AL_RX_BRG_CSI_DT0_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT1_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT2_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT3_VAL                           0x00000000

#define CDSIRX_CLKEN_VAL                                0x00000001
#define CDSIRX_CLKSEL_VAL                               0x00000101
#define CDSIRX_MODE_CONFIG_VAL                          0x00000001
#define CDSIRX_LANE_ENABLE_VAL                          0x00000012
#define CDSIRX_VC_ENABLE_VAL                            0x0000000F
#define CDSIRX_LINE_INIT_COUNT_VAL                      0x000012C0
#define CDSIRX_HSRXTO_COUNT_VAL                         0xFFFFFFFF
#define CDSIRX_FUNC_ENABLE_VAL                          0x00070701
#define CDSIRX_DSI_LPTX_MODE_VAL                        0x00000001
#define CDSIRX_DSI_TATO_COUNT_VAL                       0xFFFFFFFF
#define CDSIRX_DSI_LPTXTO_COUNT_VAL                     0xFFFFFFFF
#define CDSIRX_FUNC_MODE_VAL                            0x00000000
#define CDSIRX_PPI_HSRX_CNTRL_VAL                       0x40000000
#define CDSIRX_PPI_HSRX_COUNT_VAL                       0x0400000A
#define CDSI0_CDSIRX_PPI_DPHY_POWERCNTRL_VAL            0x00000003
#define CDSIRX_PPI_DPHY_DLYCNTRL_VAL                    0x00000000
#define CDSIRX_PPI_DPHY_LPRX_THSLD_VAL                  0x000002AA
#define CDSI0_CDSIRX_PPI_DPHY_LPRXAUTOCALST_VAL         0x00000001
#define CDSIRX_PPI_DPHY_LPTXTIMECNT_VAL                 0x00000FFF
#define CDSIRX_PPI_DSI_BTA_COUNT_VAL                    0x000407FF
#define CDSIRX_PPI_DSI_DPHYTX_ADJUST_VAL                0x00000002
#define CDSIRX_PPI_DPHY_HSRX_ADJUST_VAL                 0x000002AA
#define CDSIRX_PPI_DPHY_LPRXCALCNTRL_VAL                0x00190040
#define CDSIRX_LPRX_STATE_INT_MASK_VAL                  0x1F1F1F1D

#define CDSI0_CDSIRX_DSI_RXTRIG_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_RXERR_INT_MASK_VAL                 0x00000000
#define CDSI0_CDSIRX_TXERR_INT_MASK_VAL                 0x00000000
#define CDSI0_CDSIRX_DSI_VC0_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC1_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC2_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC3_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC0_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC1_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC2_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC3_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_CSI2_VC0_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC1_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC2_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC3_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC0_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC0_LN1_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC1_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC1_LN1_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC2_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC2_LN1_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC3_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC3_LN1_INT_MASK_VAL          0x00000000

#define CDSI0_CDSIRX_DSI_WAITBTA_COUNT_VAL              0x10000010
#define CDSI0_CDSIRX_START_VAL                          0x00000001
#define CDSI0_CDSIRX_LPRX_STATE_INT_STAT_VAL            0x00000001
#define CDSI0_CDSIRX_ADDRESS_CONFIG_VAL                 0x00000000

#define CSI_RX_PRIORITY      (60)
#define CSI_RX_STACK_SIZE    (2048)

/**
 * @brief cdsi_sensor_init callback function of ov5645 camera_sensor
 * @param dev dev pointer to structure of cdsi_dev device data
 * @return void function without return value
 */
void ov5645_csi_init(struct cdsi_dev *dev)
{
    uint32_t rdata0;
    uint32_t rdata1;

    printf("ov5645_csi_init callback function for CSI-2 rx\n");

    /* Enable the Rx bridge and set to CSI mode */
    cdsi_write(dev, CDSI0_AL_RX_BRG_MODE_OFFS, AL_RX_BRG_MODE_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_INFO_OFFS, AL_RX_BRG_CSI_INFO_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_DT0_OFFS, AL_RX_BRG_CSI_DT0_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_DT1_OFFS, AL_RX_BRG_CSI_DT1_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_DT2_OFFS, AL_RX_BRG_CSI_DT2_VAL);
    cdsi_write(dev, CDSI0_AL_RX_BRG_CSI_DT3_OFFS, AL_RX_BRG_CSI_DT3_VAL);

    /* Enable CDSIRX */
    cdsi_write(dev, CDSI0_CDSIRX_CLKEN_OFFS, CDSIRX_CLKEN_VAL);
    /* Set CDSIRX functions enable */
    cdsi_write(dev, CDSI0_CDSIRX_FUNC_ENABLE_OFFS, CDSIRX_FUNC_ENABLE_VAL);
    /* Set LPRX calibration */
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_LPRXCALCNTRL_OFFS,
               CDSIRX_PPI_DPHY_LPRXCALCNTRL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_LPRX_THSLD_OFFS,
               CDSIRX_PPI_DPHY_LPRX_THSLD_VAL);
    /* Start LPRX calibration */
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_LPRXAUTOCALST_OFFS,
               CDSI0_CDSIRX_PPI_DPHY_LPRXAUTOCALST_VAL);
    /* CDSIRX configuration */
    cdsi_write(dev, CDSI0_CDSIRX_CLKSEL_OFFS, CDSIRX_CLKSEL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_MODE_CONFIG_OFFS, CDSIRX_MODE_CONFIG_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_LANE_ENABLE_OFFS, CDSIRX_LANE_ENABLE_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_VC_ENABLE_OFFS, CDSIRX_VC_ENABLE_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_LINE_INIT_COUNT_OFFS,
               CDSIRX_LINE_INIT_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_HSRXTO_COUNT_OFFS, CDSIRX_HSRXTO_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_FUNC_MODE_OFFS, CDSIRX_FUNC_MODE_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_LPTXTIMECNT_OFFS,
               CDSIRX_PPI_DPHY_LPTXTIMECNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_LPTX_MODE_OFFS, CDSIRX_DSI_LPTX_MODE_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DSI_BTA_COUNT_OFFS,
               CDSIRX_PPI_DSI_BTA_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_HSRX_CNTRL_OFFS,
               CDSIRX_PPI_HSRX_CNTRL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_HSRX_COUNT_OFFS,
               CDSIRX_PPI_HSRX_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_POWERCNTRL_OFFS,
               CDSI0_CDSIRX_PPI_DPHY_POWERCNTRL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DSI_DPHYTX_ADJUST_OFFS,
               CDSIRX_PPI_DSI_DPHYTX_ADJUST_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_HSRX_ADJUST_OFFS,
               CDSIRX_PPI_DPHY_HSRX_ADJUST_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_PPI_DPHY_DLYCNTRL_OFFS,
               CDSIRX_PPI_DPHY_DLYCNTRL_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_LPRX_STATE_INT_MASK_OFFS,
               CDSIRX_LPRX_STATE_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_RXTRIG_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_RXTRIG_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_RXERR_INT_MASK_OFFS,
               CDSI0_CDSIRX_RXERR_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_TXERR_INT_MASK_OFFS,
               CDSI0_CDSIRX_TXERR_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC0_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC0_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC1_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC1_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC2_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC2_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC3_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC3_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC0_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC0_LN_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC1_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC1_LN_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC2_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC2_LN_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_VC3_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC3_LN_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC0_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC0_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC1_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC1_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC2_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC2_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC3_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC3_SH_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC0_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC0_LN0_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC0_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC0_LN1_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC1_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC1_LN0_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC1_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC1_LN1_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC2_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC2_LN0_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC2_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC2_LN1_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC3_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC3_LN0_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_CSI2_VC3_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC3_LN1_INT_MASK_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_LPTXTO_COUNT_OFFS,
               CDSIRX_DSI_LPTXTO_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_TATO_COUNT_OFFS,
               CDSIRX_DSI_TATO_COUNT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_WAITBTA_COUNT_OFFS,
               CDSI0_CDSIRX_DSI_WAITBTA_COUNT_VAL);

    /* Wait LPRX calibration finish */
    rdata0 = cdsi_read(dev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
    while ((rdata0 &
            CDSI0_CDSIRX_LPRX_STATE_INT_STAT_AUTOCALDONE_MASK) == 0x0) {
        rdata0 = cdsi_read(dev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
    }
    printf("First LPRX_STATE_INT: %d\n", rdata0);
    /* Start CDSIRX */
    cdsi_write(dev, CDSI0_CDSIRX_START_OFFS, CDSI0_CDSIRX_START_VAL);

    /* Wait Line Initialization finish */
    rdata1 = cdsi_read(dev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
#ifdef CONFIG_FOR_GOOGLE_IO_DEMO
    ov5645_init(0);
#endif
    while ((rdata1 &
            CDSI0_CDSIRX_LPRX_STATE_INT_STAT_LINEINITDONE_MASK) == 0x0) {
        rdata1 = cdsi_read(dev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
    }
    printf("Second LPRX_STATE_INT: %d\n", rdata1);

    cdsi_write(dev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS,
               CDSI0_CDSIRX_LPRX_STATE_INT_STAT_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_DSI_LPTX_MODE_OFFS, CDSIRX_DSI_LPTX_MODE_VAL);
    cdsi_write(dev, CDSI0_CDSIRX_ADDRESS_CONFIG_OFFS,
               CDSI0_CDSIRX_ADDRESS_CONFIG_VAL);
}

struct camera_sensor ov5645_sensor = {
    .cdsi_sensor_init = ov5645_csi_init,
};

/**
 * @brief thread routine of camera initialization function
 * @param p_data pointer of data that pass to thread routine
 */
static void camera_fn(void *p_data)
{
#if defined(CONFIG_CAMERA_RX_CDSI0)
    csi_initialize(&ov5645_sensor, TSB_CDSI0, TSB_CDSI_RX);
#else
    csi_initialize(&ov5645_sensor, TSB_CDSI1, TSB_CDSI_RX);
#endif
}

/**
 * @brief camera initialization function
 * @return zero for success or non-zero on any faillure
 */
int camera_init(void)
{
    int taskid;

    taskid = task_create("csi_rx_worker", CSI_RX_PRIORITY, CSI_RX_STACK_SIZE,
                         (main_t)camera_fn, NULL);
    if (taskid == ERROR) {
        return ERROR;
    }

    return 0;
}
