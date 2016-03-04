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

#ifndef __ARCH_ARM_INCLUDE_TSB_DSI_H
#define __ARCH_ARM_INCLUDE_TSB_DSI_H

#define TSB_CDSI0 0
#define TSB_CDSI1 1

#define TSB_CDSI_RX 0
#define TSB_CDSI_TX 1

struct cdsi_dev
{
    uint32_t base;
    int tx;
};

struct display_panel
{
    /* platform callback to initialize cdsi */
    void (*cdsi_panel_init)(struct cdsi_dev *dev);
};

struct camera_sensor
{
    /* platform callback to initialize cdsi */
    void (*cdsi_sensor_init)(struct cdsi_dev *dev);
};

void cdsi_write(struct cdsi_dev *dev, uint32_t addr, uint32_t v);
uint32_t cdsi_read(struct cdsi_dev *dev, uint32_t addr);

struct cdsi_dev *dsi_initialize(struct display_panel *panel, int dsi, int tx);
void dsi_uninitialize(struct cdsi_dev *dev);

struct cdsi_dev *csi_initialize(struct camera_sensor *sensor, int dsi, int tx);
void csi_uninitialize(struct cdsi_dev *dev);

struct cdsi_dev *cdsi_initialize(int cdsi, int tx);
void cdsi_uninitialize(struct cdsi_dev *dev);

#endif

