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

#ifndef MHB_CSI_CAMERA_H
#define MHB_CSI_CAMERA_H

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

typedef enum {
    MHB_CAMERA_STATE_INVALID = -1,
    MHB_CAMERA_STATE_OFF = 0,
    MHB_CAMERA_STATE_WAIT_POWER_ON,
    MHB_CAMERA_STATE_ON,
    MHB_CAMERA_STATE_WAIT_STREAM,
    MHB_CAMERA_STATE_STREAMING,
    MHB_CAMERA_STATE_WAIT_STREAM_CLOSE,
} mhb_camera_sm_state_t;

typedef enum {
    MHB_CAMERA_EV_FAIL = -1,
    MHB_CAMERA_EV_NONE = 0,
    MHB_CAMERA_EV_POWER_ON_REQ,
    MHB_CAMERA_EV_POWER_OFF_REQ,
    MHB_CAMERA_EV_STREAM_ON_REQ,
    MHB_CAMERA_EV_STREAM_OFF_REQ,
    MHB_CAMERA_EV_POWERED_ON,
    MHB_CAMERA_EV_CONFIGURED,
    MHB_CAMERA_EV_DECONFIGURED,
} mhb_camera_sm_event_t;

typedef mhb_camera_sm_event_t (*mhb_camera_command_func)();

mhb_camera_sm_state_t mhb_camera_sm_get_state();
int mhb_camera_sm_init();
int mhb_camera_sm_execute(mhb_camera_sm_event_t event);

int mhb_camera_i2c_read(uint16_t i2c_addr,
                        uint8_t *addr, int addr_len,
                        uint8_t *data, int data_len);
int mhb_camera_i2c_write(uint16_t i2c_addr,
                         uint8_t *addr, int addr_len);
uint8_t mhb_camera_i2c_read_reg1(uint16_t i2c_addr, uint16_t regaddr);
uint16_t mhb_camera_i2c_read_reg2(uint16_t i2c_addr, uint16_t regaddr);
uint32_t mhb_camera_i2c_read_reg4(uint16_t i2c_addr, uint16_t regaddr);
int mhb_camera_i2c_write_reg1(uint16_t i2c_addr, uint16_t regaddr, uint8_t data);
int mhb_camera_i2c_write_reg2(uint16_t i2c_addr, uint16_t regaddr, uint16_t data);
int mhb_camera_i2c_write_reg4(uint16_t i2c_addr, uint16_t regaddr, uint32_t data);
uint16_t mhb_camera_i2c_read_reg2_16(uint16_t i2c_addr, uint16_t regaddr);
uint32_t mhb_camera_i2c_read_reg4_16(uint16_t i2c_addr, uint16_t regaddr);
int mhb_camera_i2c_write_reg2_16(uint16_t i2c_addr, uint16_t regaddr, uint16_t data);
int mhb_camera_i2c_write_reg4_16(uint16_t i2c_addr, uint16_t regaddr, uint32_t data);

mhb_camera_sm_event_t mhb_camera_power_on(void);
mhb_camera_sm_event_t mhb_camera_power_off(void);
mhb_camera_sm_event_t mhb_camera_stream_on(void);
mhb_camera_sm_event_t mhb_camera_stream_off(void);

int _mhb_camera_soc_enable();
int _mhb_camera_soc_disable();
int _mhb_camera_stream_enable();
int _mhb_camera_stream_configure();
int _mhb_camera_stream_disable();
int _mhb_camera_init(struct device *dev);
uint8_t _mhb_camera_get_csi_rx_lanes(const void *udata);

#endif
