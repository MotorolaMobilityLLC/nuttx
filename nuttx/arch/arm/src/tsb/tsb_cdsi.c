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

#include <stdlib.h>

#include <nuttx/arch.h>
#include <arch/tsb/cdsi.h>
#include <arch/tsb/vidcrypt.h>

#include "chip.h"
#include "up_arch.h"
#include "tsb_scm.h"

void cdsi_write(struct cdsi_dev *dev, uint32_t addr, uint32_t v)
{
    vdbg("addr=0x%x, value=0x%x\n", addr, v);
    putreg32(v, dev->base + addr);
}

uint32_t cdsi_read(struct cdsi_dev *dev, uint32_t addr)
{
    return getreg32(dev->base + addr);
}

struct cdsi_dev *cdsi_initialize(int cdsi, int tx)
{
    struct cdsi_dev *dev;

    dev = malloc(sizeof(struct cdsi_dev));
    if (!dev)
        return NULL;

    dev->tx = tx;
    dev->base = cdsi == TSB_CDSI0 ? CDSI0_BASE : CDSI1_BASE;
    tsb_clk_enable(cdsi == TSB_CDSI0 ? TSB_CLK_CDSI0_REF : TSB_CLK_CDSI1_REF);
#if CONFIG_ARCH_VIDCRYPT
    tsb_clk_enable(TSB_CLK_VIDENCRYPT);
    tsb_reset(TSB_RST_VIDCRYPTIF);
    tsb_reset(TSB_RST_VIDCRYPTCH0);
    tsb_reset(TSB_RST_VIDCRYPTCH1);
#endif
    if (tx) {
        if (cdsi == TSB_CDSI0) {
            tsb_clk_enable(TSB_CLK_CDSI0_TX_SYS);
            tsb_clk_enable(TSB_CLK_CDSI0_TX_APB);
            tsb_reset(TSB_RST_CDSI0_TX);
            tsb_reset(TSB_RST_CDSI0_TX_AIO);
        } else {
            tsb_clk_enable(TSB_CLK_CDSI1_TX_SYS);
            tsb_clk_enable(TSB_CLK_CDSI1_TX_APB);
            tsb_reset(TSB_RST_CDSI1_TX);
            tsb_reset(TSB_RST_CDSI1_TX_AIO);
        }
    } else {
        if (cdsi == TSB_CDSI0) {
            tsb_clk_enable(TSB_CLK_CDSI0_RX_SYS);
            tsb_clk_enable(TSB_CLK_CDSI0_RX_APB);
            tsb_reset(TSB_RST_CDSI0_RX);
            tsb_reset(TSB_RST_CDSI0_RX_AIO);
        } else {
            tsb_clk_enable(TSB_CLK_CDSI1_RX_SYS);
            tsb_clk_enable(TSB_CLK_CDSI1_RX_APB);
            tsb_reset(TSB_RST_CDSI1_RX);
            tsb_reset(TSB_RST_CDSI1_RX_AIO);
        }
    }

#if CONFIG_ARCH_VIDCRYPT
    tsb_vidcrypt_set_mode(cdsi, tx);
    tsb_vidcrypt_enable(cdsi);
#endif
    return dev;
}

void cdsi_uninitialize(struct cdsi_dev *dev)
{
    int cdsi = dev->base == CDSI0_BASE ? TSB_CDSI0 : TSB_CDSI1;

#if CONFIG_ARCH_VIDCRYPT
    tsb_vidcrypt_disable(cdsi);
    tsb_clk_disable(TSB_CLK_VIDENCRYPT);
#endif
    tsb_clk_disable(cdsi == TSB_CDSI0 ? TSB_CLK_CDSI0_REF : TSB_CLK_CDSI1_REF);
    if (dev->tx) {
        if (cdsi == TSB_CDSI0) {
            tsb_clk_disable(TSB_CLK_CDSI0_TX_SYS);
            tsb_clk_disable(TSB_CLK_CDSI0_TX_APB);
        } else {
            tsb_clk_disable(TSB_CLK_CDSI1_TX_SYS);
            tsb_clk_disable(TSB_CLK_CDSI1_TX_APB);
        }
    } else {
        if (cdsi == TSB_CDSI0) {
            tsb_clk_disable(TSB_CLK_CDSI0_RX_SYS);
            tsb_clk_disable(TSB_CLK_CDSI0_RX_APB);
        } else {
            tsb_clk_disable(TSB_CLK_CDSI1_RX_SYS);
            tsb_clk_disable(TSB_CLK_CDSI1_RX_APB);
        }
    }
    free(dev);
}

/**
 * @brief Initialize cdsi in dsi mode
 * @param panel Panel to register
 * @param cdsi   The cdsi block to initialize
 * @param tx    0 if dsi work in rx mode or 1 for tx mode
 * @return a cdsi_dev pointer or NULL on any faillure.
 */
struct cdsi_dev *dsi_initialize(struct display_panel *panel, int cdsi, int tx)
{
    struct cdsi_dev *dev;

    dev = cdsi_initialize(cdsi, tx);
    if (!dev)
        return NULL;

    panel->cdsi_panel_init(dev);

    return dev;
}

/**
 * @brief Uninitialize dsi
 * @param dev Pointer to cdsi to uninitialize
 */
void dsi_uninitialize(struct cdsi_dev *dev)
{
    cdsi_uninitialize(dev);
}

/**
 * @brief Initialize cdsi in csi mode
 * @param sensor Sensor to register
 * @param cdsi The cdsi block to initialize
 * @param tx 0 if csi work in rx mode or 1 for tx mode
 * @return a cdsi_dev pointer or NULL on any faillure.
 */
struct cdsi_dev *csi_initialize(struct camera_sensor *sensor, int cdsi, int tx)
{
    struct cdsi_dev *dev;

    dev = cdsi_initialize(cdsi, tx);
    if (!dev)
        return NULL;

    if (sensor->cdsi_sensor_init) {
        sensor->cdsi_sensor_init(dev);
    }

    return dev;
}

/**
 * @brief Uninitialize csi
 * @param dev Pointer to cdsi to uninitialize
 */
void csi_uninitialize(struct cdsi_dev *dev)
{
    cdsi_uninitialize(dev);
}
