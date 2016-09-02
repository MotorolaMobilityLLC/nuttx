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

#define DEBUG
//#define DEBUG_CTRL

#include <errno.h>
#include <debug.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/i2c.h>
#include <arch/board/mods.h>
#include <nuttx/time.h>
#include <nuttx/device.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/device_mhb_cam.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/mhb/mhb_csi_camera.h>
#include <nuttx/power/pm.h>
#include <nuttx/camera/v4l2_camera_ext_ctrls.h>
#include <nuttx/camera/camera_ext.h>

#include "mhb_csi_camera_sm.h"

/*
 * This is the reference greybus camera extension driver
 * for MHB (Motorola Mods Hi-speed Bus Archirecture)
 *
 */

#define UNIPRO_RECOVER_RETRIES   2
#define CTRL_RETRIES             5
#define MHB_CDSI_CAM_INSTANCE    0
#define MHB_PCTRL_OP_TIMEOUT_NS  3000000000LL
#define MHB_CDSI_OP_TIMEOUT_NS   1000000000LL
#define CAM_CTRL_RETRY_DELAY_US  200000

#define MHB_CAM_PWRCTL_WAIT_MASK 0xFE00
#define MHB_CAM_CDSI_WAIT_MASK   0xFC00
#define MHB_CAM_WAIT_EV_NONE     0x0000
#define MHB_CAM_MAX_CALLBACKS    4

enum apbe_config_status {
    APBE_RESET = -1,
    APBE_UNCONFIGURED = 0,
    APBE_CONFIGURED,
};

enum soc_status {
    SOC_DISABLED = 0,
    SOC_DISABLING,
    SOC_ENABLED,
};

struct cached_ctrls_node {
    struct list_head node;
    struct device *dev;
    uint32_t idx;
    uint8_t *ctrl_val;
    uint32_t ctrl_val_size;
};

struct mhb_camera_s
{
    struct device *mhb_device;
    struct device *cam_device;
    struct device *slave_pwr_ctrl;
    struct i2c_dev_s* cam_i2c;
    uint8_t apbe_enabled;
    uint8_t apbe_en_state;
    uint8_t apbe_config_state;
    uint8_t soc_status;
    uint8_t cdsi_instance;
    uint8_t bootmode;
    pthread_mutex_t mutex;
    pthread_mutex_t i2c_mutex;
    pthread_mutex_t ctrl_mutex;
    pthread_cond_t slave_cond;
    pthread_cond_t cdsi_cond;
    uint16_t mhb_wait_event;
    mhb_camera_notification_cb callbacks[MHB_CAM_MAX_CALLBACKS];
};

static struct mhb_camera_s s_mhb_camera;
static LIST_DECLARE(cached_ctrl_list);

static void mhb_cam_error_cb(int err);

#ifdef CONFIG_PM
static int mhb_camera_pm_prepare(struct pm_callback_s *cb,
                                 enum pm_state_e pm_state)
{
    if (pm_state >= PM_STANDBY)
    {
       /* Multiple situations when we dont want MUC to suspend  */
       /* Waiting on transactions                               */
       /* SM in Wait states                                     */
       /* SM in OFF && SOC is still enabled. pthread_timed_wait */
       /* timers dont seem to wake up reliably, so handle here  */

       if (!pthread_mutex_trylock(&s_mhb_camera.i2c_mutex))
           return -EBUSY;


       pthread_mutex_lock(&s_mhb_camera.mutex);
       uint8_t sm_state = mhb_camera_sm_get_state();
       /*
       CAM_DBG("pm_state=%d, sm_state=%d, mhb_wait=%04x, soc=%d\n",
               pm_state, sm_state, s_mhb_camera.mhb_wait_event,
               s_mhb_camera.soc_status);
       */
       if ((sm_state == MHB_CAMERA_STATE_WAIT_OFF ||
            sm_state == MHB_CAMERA_STATE_WAIT_POWER_ON ||
            sm_state == MHB_CAMERA_STATE_WAIT_STREAM||
            sm_state == MHB_CAMERA_STATE_WAIT_STREAM_CLOSE)
           ||
           (s_mhb_camera.mhb_wait_event != MHB_CAM_WAIT_EV_NONE)
           ||
           (sm_state == MHB_CAMERA_STATE_OFF &&
            s_mhb_camera.soc_status != SOC_DISABLED)
          ) {
            pthread_mutex_unlock(&s_mhb_camera.i2c_mutex);
            pthread_mutex_unlock(&s_mhb_camera.mutex);
            return -EBUSY;
       }
       pthread_mutex_unlock(&s_mhb_camera.i2c_mutex);
       pthread_mutex_unlock(&s_mhb_camera.mutex);
    }
    return 0;
}

static struct pm_callback_s pm_callback =
{
    .prepare = mhb_camera_pm_prepare,
};
#endif

/* Cam I2C Ops */

static inline void mhb_camera_i2c_lock(void)
{
    pthread_mutex_lock(&s_mhb_camera.i2c_mutex);
}

static inline void mhb_camera_i2c_unlock(void)
{
    pthread_mutex_unlock(&s_mhb_camera.i2c_mutex);
}

int mhb_camera_i2c_read(uint16_t i2c_addr,
                    uint8_t *addr, int addr_len,
                    uint8_t *data, int data_len)
{
    int ret = 0, retries = CONFIG_MHB_CAMERA_I2C_RETRY;
    struct i2c_msg_s msg[2];
    msg[0].addr   = i2c_addr;
    msg[0].flags  = 0;
    msg[0].buffer = addr;
    msg[0].length = addr_len;

    msg[1].addr   = i2c_addr;
    msg[1].flags  = I2C_M_READ|msg[0].flags;
    msg[1].buffer = data;
    msg[1].length = data_len;

    mhb_camera_i2c_lock();
    do {
        ret = I2C_TRANSFER(s_mhb_camera.cam_i2c, msg, 2);
        if (ret) {
            usleep(CONFIG_MHB_CAMERA_I2C_RETRY_DELAY_US);
            CAM_DBG("i2c err %d\n", ret);
        }
    } while (ret && --retries);
    mhb_camera_i2c_unlock();

    if (ret || (CONFIG_MHB_CAMERA_I2C_RETRY - retries))
        CAM_ERR("%s I2C read retried %d of %d : ret %d\n",
                ret ? "FAIL":"INFO",
                CONFIG_MHB_CAMERA_I2C_RETRY - retries,
                CONFIG_MHB_CAMERA_I2C_RETRY, ret);

    return ret;
}


int mhb_camera_i2c_write(uint16_t i2c_addr, uint8_t *addr, int addr_len)
{
    struct i2c_msg_s msg;
    int ret = 0, retries = CONFIG_MHB_CAMERA_I2C_RETRY;

    msg.addr   = i2c_addr;
    msg.flags  = I2C_M_NORESTART;
    msg.buffer = addr;
    msg.length = addr_len;

    mhb_camera_i2c_lock();
    do {
        ret = I2C_TRANSFER(s_mhb_camera.cam_i2c, &msg, 1);
        if (ret) {
            usleep(CONFIG_MHB_CAMERA_I2C_RETRY_DELAY_US);
            CAM_DBG("i2c err %d\n", ret);
        }
    } while (ret && --retries);
    mhb_camera_i2c_unlock();

    if (ret || (CONFIG_MHB_CAMERA_I2C_RETRY - retries))
        CAM_ERR("%s I2C write retried %d of %d : ret %d\n",
                ret ? "FAIL":"INFO",
                CONFIG_MHB_CAMERA_I2C_RETRY - retries,
                CONFIG_MHB_CAMERA_I2C_RETRY, ret);

    return ret;
}

int mhb_camera_i2c_read_reg(uint16_t i2c_addr, uint16_t regaddr,
                            void *value, uint8_t reg_size, uint8_t reg_width)
{
    uint8_t addr[2];
    uint8_t data[4];
    uint32_t val32 = 0;
    memset(data, 0, sizeof(data));

    if (reg_width == MHB_CAMERA_REG_WIDTH_16) {
        addr[0] = (regaddr >> 8) & 0xFF;
        addr[1] = regaddr & 0xFF;
    } else {
        addr[0] = regaddr & 0xFF;
        addr[1] = (regaddr >> 8) & 0xFF;
    }

    if (!mhb_camera_i2c_read(i2c_addr, addr, sizeof(addr), data, reg_size) == 0) {
        CAM_ERR("Failed i2c read %02x 0x%04x\n", i2c_addr, regaddr);
        return -1;
    }

    if (reg_width == MHB_CAMERA_REG_WIDTH_16) {
        switch (reg_size) {
            case MHB_CAMERA_REG_DWORD:
                val32 = (data[2] << 24) + (data[3] << 16);
            case MHB_CAMERA_REG_WORD:
                val32 += (data[0] << 8) + data[1];
                break;
            case MHB_CAMERA_REG_BYTE:
                val32 = data[0];
                break;
            default:
                CAM_ERR("Invalid register length %d\n", reg_size);
                return -1;
        }
    } else {
        switch (reg_size) {
            case MHB_CAMERA_REG_DWORD:
                val32 = (data[3] << 24) + (data[2] << 16);
            case MHB_CAMERA_REG_WORD:
                val32 += (data[1] << 8);
            case MHB_CAMERA_REG_BYTE:
                val32 += data[0];
                break;
            default:
                CAM_ERR("Invalid register length %d\n", reg_size);
                return -1;
        }
    }

    switch (reg_size) {
        case MHB_CAMERA_REG_DWORD:  *(uint32_t*)value = val32; break;
        case MHB_CAMERA_REG_WORD:   *(uint16_t*)value = val32; break;
        case MHB_CAMERA_REG_BYTE:   *(uint8_t*)value = val32; break;
        default: break;
    }
    CTRL_DBG("read: %02x 0x%04x -> 0x%08x\n", i2c_addr, regaddr, val32);
    return 0;
}

int mhb_camera_i2c_write_reg(uint16_t i2c_addr, uint16_t regaddr,
                             uint32_t data, uint8_t reg_size, uint8_t reg_width)
{
    uint8_t addr[6];
    CTRL_DBG("write 0x%08x to %02x addr 0x%04x\n", data, i2c_addr, regaddr);

    if (reg_width == MHB_CAMERA_REG_WIDTH_16) {
        addr[0] = (regaddr >> 8) & 0xFF;
        addr[1] = regaddr & 0xFF;
        switch (reg_size) {
            case MHB_CAMERA_REG_DWORD:
                addr[4] = (data >> 24) & 0xFF;
                addr[5] = (data >> 16) & 0xFF;
            case MHB_CAMERA_REG_WORD:
                addr[2] = (data >> 8) & 0xFF;
                addr[3] = data & 0xFF;
                break;
            case MHB_CAMERA_REG_BYTE:
                addr[2] = data & 0xFF;;
                break;
            default:
                CAM_ERR("Invalid register length %d\n", reg_size);
                return -1;
        }
    } else {
        addr[0] = regaddr & 0xFF;
        addr[1] = (regaddr >> 8) & 0xFF;
        switch (reg_size) {
            case MHB_CAMERA_REG_DWORD:
                addr[4] = (data >> 16) & 0xFF;
                addr[5] = (data >> 24) & 0xFF;
            case MHB_CAMERA_REG_WORD:
                addr[3] = (data >> 8) & 0xFF;
            case MHB_CAMERA_REG_BYTE:
                addr[2] = data & 0xFF;;
                break;
            default:
                CAM_ERR("Invalid register length %d\n", reg_size);
                return -1;
        }
    }

    int ret = mhb_camera_i2c_write(i2c_addr, addr, reg_size + 2);
    if (ret != 0) {
        CAM_ERR("Failed i2c write 0x%08x to %02x  addr 0x%04x err %d\n",
                data, i2c_addr, regaddr, ret);
    }

    return ret;
}

static void mhb_csi_camera_callback(uint8_t event)
{
    int i;
    for(i=0; i < MHB_CAM_MAX_CALLBACKS; ++i) {
        if (s_mhb_camera.callbacks[i])
           s_mhb_camera.callbacks[i](event);
        else break;
    }
}

int mhb_csi_camera_status_register_callback(
        mhb_camera_notification_cb callback)
{
    int i, retval = -1;
    uint8_t event = MHB_CAMERA_NOTIFY_POWERED_OFF;

    pthread_mutex_lock(&s_mhb_camera.mutex);
    for(i=0; i < MHB_CAM_MAX_CALLBACKS; ++i) {
        if (s_mhb_camera.callbacks[i] == callback) {
           retval = 0;
           break;
        }

        if (s_mhb_camera.callbacks[i] == NULL) {
            s_mhb_camera.callbacks[i] = callback;
            retval = 0;
            break;
        }
    }
    pthread_mutex_unlock(&s_mhb_camera.mutex);

    if (s_mhb_camera.soc_status == SOC_ENABLED) {
        if (s_mhb_camera.bootmode == CAMERA_EXT_BOOTMODE_DFU)
            event = MHB_CAMERA_NOTIFY_FW_UPGRADE;
        else
            event = MHB_CAMERA_NOTIFY_POWERED_ON;
    }

    callback(event);

    return retval;
}

/* MHB Ops */
static int _mhb_camera_wait_for_response(pthread_cond_t *cond,
                                         uint16_t wait_event, char *str,
                                         uint64_t timeout)
{
    int result;
    struct timespec expires;

    if (clock_gettime(CLOCK_REALTIME, &expires)) {
        return -EBADF;
    }

    uint64_t new_ns = timespec_to_nsec(&expires);
    new_ns += timeout;
    nsec_to_timespec(new_ns, &expires);

    s_mhb_camera.mhb_wait_event = wait_event;
    result = pthread_cond_timedwait(cond, &s_mhb_camera.mutex, &expires);
    if (!result)
        result = s_mhb_camera.mhb_wait_event;
    s_mhb_camera.mhb_wait_event = MHB_CAM_WAIT_EV_NONE;

    clock_gettime(CLOCK_REALTIME, &expires);
    new_ns = timespec_to_nsec(&expires) - (new_ns - timeout);
    CAM_ERR("%s: Time spent on %s %dms Result %X\n", result?"ERROR":"INFO",
            str, (uint16_t)(new_ns/NSEC_PER_MSEC), result);

    return result;
}

static int _mhb_camera_slave_status_callback(struct device *dev, uint32_t slave_status)
{
    pthread_mutex_lock(&s_mhb_camera.mutex);

    if (slave_status != MHB_PM_STATUS_PEER_CONNECTED)
        s_mhb_camera.apbe_config_state = APBE_UNCONFIGURED;

    s_mhb_camera.apbe_en_state = slave_status;

    CAM_DBG("slave status %d wait_event=0x%04x\n",
            slave_status, s_mhb_camera.mhb_wait_event);

    if (s_mhb_camera.mhb_wait_event == (slave_status|MHB_CAM_PWRCTL_WAIT_MASK)) {
        s_mhb_camera.mhb_wait_event = MHB_CAM_WAIT_EV_NONE;
         pthread_cond_signal(&s_mhb_camera.slave_cond);
    }
    pthread_mutex_unlock(&s_mhb_camera.mutex);
    return 0;
}

static int _mhb_camera_set_apbe_enable(int state)
{
    int ret;

    ret = device_slave_pwrctrl_send_slave_state(s_mhb_camera.slave_pwr_ctrl, state);
    if(ret) {
        CAM_ERR("ERROR: Failed to set slave state %d\n", ret);
        return ret;
    }

    return ret;
}

static int _mhb_csi_camera_config_req(uint8_t *cfg, size_t cfg_size)
{
    struct mhb_hdr hdr;
    int result = -1;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI1;
    hdr.type = MHB_TYPE_CDSI_CONFIG_REQ;

    result = device_mhb_send(s_mhb_camera.mhb_device, &hdr, (uint8_t *)cfg, cfg_size, 0);
    if (result) {
        CAM_ERR("ERROR: failed to send mhb config req. result %d\n", result);
        return result;
    }

    return result;
}

static int _mhb_csi_camera_unconfig_req(void)
{
    struct mhb_hdr hdr;
    int result = 0;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI1;
    hdr.type = MHB_TYPE_CDSI_UNCONFIG_REQ;

    result = device_mhb_send(s_mhb_camera.mhb_device, &hdr, NULL, 0, 0);
    if (result) {
        CAM_ERR("ERROR: failed to send mhb unconfig req. result %d\n", result);
        return result;
    }

    return result;
}


static int _mhb_csi_camera_control_req(uint8_t command)
{
    struct mhb_hdr hdr;
    struct mhb_cdsi_control_req req;
    int result = 0;

    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_CDSI1;
    hdr.type = MHB_TYPE_CDSI_CONTROL_REQ;

    req.command = command;

    result = device_mhb_send(s_mhb_camera.mhb_device, &hdr, (uint8_t *)&req, sizeof(req), 0);
    if (result) {
        CAM_ERR("ERROR: failed to send mhb ctrl cmd %d result %d\n", command, result);
        return result;
    }

    return result;
}

static int _mhb_csi_camera_handle_msg(struct device *dev,
    struct mhb_hdr *hdr, uint8_t *payload, size_t payload_length)
{
    if (hdr->addr == MHB_ADDR_CDSI1) {
        switch (hdr->type) {
        case MHB_TYPE_CDSI_WRITE_CMDS_RSP:
        case MHB_TYPE_CDSI_READ_CMDS_RSP:
        case MHB_TYPE_CDSI_STATUS_RSP:
            if (hdr->result)
                CAM_ERR("ERROR:failed cmd: addr=%02x type=%02x result=%02x\n",
                        hdr->addr, hdr->type, hdr->result);
            break;
        case MHB_TYPE_CDSI_UNCONFIG_RSP:
        case MHB_TYPE_CDSI_CONFIG_RSP:
        case MHB_TYPE_CDSI_CONTROL_RSP:
            pthread_mutex_lock(&s_mhb_camera.mutex);
            if (s_mhb_camera.mhb_wait_event == (hdr->type|MHB_CAM_CDSI_WAIT_MASK)) {
                if (hdr->result == MHB_RESULT_SUCCESS) {
                    s_mhb_camera.mhb_wait_event = MHB_CAM_WAIT_EV_NONE;
                }
                pthread_cond_signal(&s_mhb_camera.cdsi_cond);
            } else {
                 CAM_ERR("ERROR Unexpected Rsp addr=%02x type=%02x result=%02x wait=0x%04x\n",
                    hdr->addr, hdr->type,
                    hdr->result, s_mhb_camera.mhb_wait_event);
            }
            pthread_mutex_unlock(&s_mhb_camera.mutex);
            break;
        default:
            CAM_ERR("unexpected rsp: addr=%02x type=%02x result=%02x wait=0x%04x\n",
                    hdr->addr, hdr->type,
                    hdr->result, s_mhb_camera.mhb_wait_event);
        }
    }

    return 0;
}

static int _mhb_csi_camera_start_stream(uint8_t *cfg, size_t cfg_size)
{
    if(MHB_CAM_DEV_OP(s_mhb_camera.cam_device, stream_configure)) {
        return -EREMOTEIO;
    }

    pthread_mutex_lock(&s_mhb_camera.mutex);
    if (_mhb_csi_camera_config_req(cfg, cfg_size)) {
        CAM_ERR("ERROR: Send Config Failed\n");
        pthread_mutex_unlock(&s_mhb_camera.mutex);
        return -EIO;
    }

    if (_mhb_camera_wait_for_response(&s_mhb_camera.cdsi_cond,
                                  MHB_CAM_CDSI_WAIT_MASK|MHB_TYPE_CDSI_CONFIG_RSP,
                                  "CDSI CONFIG", MHB_CDSI_OP_TIMEOUT_NS)) {
        pthread_mutex_unlock(&s_mhb_camera.mutex);
        return -EIO;
    }
    pthread_mutex_unlock(&s_mhb_camera.mutex);

    if (MHB_CAM_DEV_OP(s_mhb_camera.cam_device, stream_enable)) {
        CAM_ERR("ERROR: _mhb_camera_stream_enable Failed\n");
        return -EREMOTEIO;
    }

    pthread_mutex_lock(&s_mhb_camera.mutex);
    if (_mhb_csi_camera_control_req(MHB_CDSI_COMMAND_START)) {
        CAM_ERR("ERROR: MHB_CDSI_COMMAND_START Failed\n");
        pthread_mutex_unlock(&s_mhb_camera.mutex);
        return -EIO;
    }

    if (_mhb_camera_wait_for_response(&s_mhb_camera.cdsi_cond,
                                  MHB_CAM_CDSI_WAIT_MASK|MHB_TYPE_CDSI_CONTROL_RSP,
                                  "CDSI START", MHB_CDSI_OP_TIMEOUT_NS)) {
        pthread_mutex_unlock(&s_mhb_camera.mutex);
        return -EIO;
    }
    pthread_mutex_unlock(&s_mhb_camera.mutex);
    return 0;
}

static int _mhb_csi_camera_stop_stream(void)
{
    int ret = -1;

    pthread_mutex_lock(&s_mhb_camera.mutex);
    ret = _mhb_csi_camera_control_req(MHB_CDSI_COMMAND_STOP);

    if (!ret) {
        ret = _mhb_camera_wait_for_response(&s_mhb_camera.cdsi_cond,
                              MHB_CAM_CDSI_WAIT_MASK|MHB_TYPE_CDSI_CONTROL_RSP,
                              "CDSI STOP", MHB_CDSI_OP_TIMEOUT_NS);
    }
    pthread_mutex_unlock(&s_mhb_camera.mutex);

    MHB_CAM_DEV_OP(s_mhb_camera.cam_device, stream_disable);

    if (ret) {
        CAM_ERR("ERROR: CDSI STOP Failed\n");
        return -1;
    }

    pthread_mutex_lock(&s_mhb_camera.mutex);
    if (_mhb_csi_camera_unconfig_req()) {
        CAM_ERR("ERROR: Send UnConfig Failed\n");
        pthread_mutex_unlock(&s_mhb_camera.mutex);
        return -1;
    }

    if (_mhb_camera_wait_for_response(&s_mhb_camera.cdsi_cond,
                                  MHB_CAM_CDSI_WAIT_MASK|MHB_TYPE_CDSI_UNCONFIG_RSP,
                                  "CDSI UNCONFIG", MHB_CDSI_OP_TIMEOUT_NS)) {
        pthread_mutex_unlock(&s_mhb_camera.mutex);
        return -1;
    }
    pthread_mutex_unlock(&s_mhb_camera.mutex);

    return 0;

}

void mhb_camera_process_ctrl_cache(int dump)
{
    struct cached_ctrls_node *item;
    uint8_t soc_status;

    pthread_mutex_lock(&s_mhb_camera.ctrl_mutex);
    soc_status = s_mhb_camera.soc_status;

    while (!list_is_empty(&cached_ctrl_list)) {
        item = list_entry(cached_ctrl_list.next,
                          struct cached_ctrls_node, node);

        if ((soc_status == SOC_ENABLED) && !dump) {
            int ret = 0, retries = CTRL_RETRIES;
            do {
                ret = camera_ext_ctrl_set(item->dev, item->idx,
                                          item->ctrl_val, item->ctrl_val_size);
            } while (ret == -EAGAIN && --retries);

            if (ret || (CTRL_RETRIES - retries))
                CAM_ERR("%s camera_ext_ctrl_set 0x%08x retries %d/%d\n",
                    ret ? "FAIL":"INFO", item->idx,
                    CTRL_RETRIES - retries, CTRL_RETRIES);
        }
        list_del(&item->node);
        kmm_free(item->ctrl_val);
        kmm_free(item);
    }
    pthread_mutex_unlock(&s_mhb_camera.ctrl_mutex);

}

mhb_camera_sm_event_t mhb_camera_power_on(void)
{
    uint8_t bootmode = s_mhb_camera.bootmode;

    CAM_DBG("soc_status = %d apbe_en_state = %d, bootmode = %d\n",
            s_mhb_camera.soc_status, s_mhb_camera.apbe_en_state, bootmode);

    // TODO: Hack : Should not depend on APBE_PWR_EN
    gpio_direction_out(GPIO_APBE_PWR_EN, 1);

    if (s_mhb_camera.cam_i2c == NULL) {
        s_mhb_camera.cam_i2c = up_i2cinitialize(CONFIG_MHB_CAMERA_I2C_BUS_ID);
        if (s_mhb_camera.cam_i2c == NULL) {
            CAM_ERR("Failed to initialize I2C\n");
            return MHB_CAMERA_EV_FAIL;
        }
        I2C_SETFREQUENCY(s_mhb_camera.cam_i2c, 400000);
    }

    if (s_mhb_camera.soc_status == SOC_ENABLED) {
        if (bootmode == CAMERA_EXT_BOOTMODE_PREVIEW) {
            mhb_camera_lens_extend();
        }
        if (bootmode == CAMERA_EXT_BOOTMODE_DFU) {
            mhb_csi_camera_callback(MHB_CAMERA_NOTIFY_POWERED_OFF);
            MHB_CAM_DEV_OP(s_mhb_camera.cam_device, soc_disable);
            s_mhb_camera.soc_status = SOC_DISABLED;
        } else {
            return MHB_CAMERA_EV_POWERED_ON;
        }
    }

    if(s_mhb_camera.apbe_enabled == 0 &&
       _mhb_camera_set_apbe_enable(SLAVE_STATE_ENABLED)) {
            CAM_ERR("Failed to turn ON APBE\n");
            goto failed_power_on;
    }
    s_mhb_camera.apbe_enabled = 1;

    if (MHB_CAM_DEV_OP(s_mhb_camera.cam_device, soc_enable,
                       s_mhb_camera.bootmode)) {
        CAM_ERR("Failed to turn on Camera SOC\n");
        goto failed_power_on;
    }

    s_mhb_camera.soc_status = SOC_ENABLED;
    mhb_camera_process_ctrl_cache(FALSE);

    pthread_mutex_lock(&s_mhb_camera.mutex);
    if(s_mhb_camera.apbe_en_state != MHB_PM_STATUS_PEER_CONNECTED
       &&
       _mhb_camera_wait_for_response(&s_mhb_camera.slave_cond,
                MHB_CAM_PWRCTL_WAIT_MASK|MHB_PM_STATUS_PEER_CONNECTED,
                "PEER CONNECTED", MHB_PCTRL_OP_TIMEOUT_NS)) {
        CAM_ERR("Failed waiting for PEER_CONNECTED\n");
        s_mhb_camera.apbe_config_state = APBE_RESET;
    }
    pthread_mutex_unlock(&s_mhb_camera.mutex);

    mhb_csi_camera_callback(
        (bootmode == CAMERA_EXT_BOOTMODE_DFU) ?
        MHB_CAMERA_NOTIFY_FW_UPGRADE : MHB_CAMERA_NOTIFY_POWERED_ON);

    return MHB_CAMERA_EV_POWERED_ON;

failed_power_on:
    camera_ext_send_error(CAMERA_EXT_ERROR_FATAL);
    return MHB_CAMERA_EV_FAIL;

}

mhb_camera_sm_event_t mhb_camera_power_off(void)
{
    CAM_DBG("soc_status = %d apbe_en_state = %d\n",
            s_mhb_camera.soc_status, s_mhb_camera.apbe_en_state);

    if (s_mhb_camera.apbe_enabled) {
        _mhb_camera_set_apbe_enable(SLAVE_STATE_DISABLED);
        s_mhb_camera.apbe_enabled = 0;
    }

    if (s_mhb_camera.soc_status == SOC_ENABLED) {
        s_mhb_camera.soc_status = SOC_DISABLING;
        mhb_csi_camera_callback(MHB_CAMERA_NOTIFY_POWERED_OFF);
        MHB_CAM_DEV_OP(s_mhb_camera.cam_device, soc_disable);
        s_mhb_camera.soc_status = SOC_DISABLED;
    }

    if (s_mhb_camera.cam_i2c) {
        up_i2cuninitialize(s_mhb_camera.cam_i2c);
        s_mhb_camera.cam_i2c = NULL;
    }

    return MHB_CAMERA_EV_NONE;
}

mhb_camera_sm_event_t mhb_camera_stream_on(void)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();
    const struct camera_ext_format_node *fmt;
    const struct camera_ext_frmsize_node *frmsize;
    struct mhb_cdsi_config *cdsi_config;
    int retry;

    CAM_DBG("\n");

    fmt = get_current_format_node(db, cfg);
    if (fmt == NULL) {
        CAM_ERR("Failed to get current format\n");
        goto failed_stream_on;
    }

    frmsize = get_current_frmsize_node(db, cfg);
    if (frmsize == NULL) {
        CAM_ERR("Failed to get current frame size\n");
        goto failed_stream_on;
    }

    if (MHB_CAM_DEV_OP(s_mhb_camera.cam_device, get_csi_config, (void*)&cdsi_config)) {
        CAM_ERR("Failed to get CSI Params\n");
        goto failed_stream_on;
    }

    switch(fmt->fourcc) {
        case V4L2_PIX_FMT_RGB24:
        case V4L2_PIX_FMT_BGR24:
            cdsi_config->bpp = 24;
            break;
        case V4L2_PIX_FMT_UYVY:
        case V4L2_PIX_FMT_VYUY:
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_YVYU:
            cdsi_config->bpp = 16;
            break;
        case V4L2_PIX_FMT_SBGGR10:
        case V4L2_PIX_FMT_SGBRG10:
        case V4L2_PIX_FMT_SGRBG10:
        case V4L2_PIX_FMT_SRGGB10:
            cdsi_config->bpp = 10;
            break;
        case V4L2_PIX_FMT_SBGGR8:
        case V4L2_PIX_FMT_SGBRG8:
        case V4L2_PIX_FMT_SGRBG8:
        case V4L2_PIX_FMT_SRGGB8:
            cdsi_config->bpp = 8;
        default:
            CAM_ERR("Unsupported format 0x%x\n", fmt->fourcc);
            goto failed_stream_on;
    }

    cdsi_config->width = frmsize->width;
    cdsi_config->height = frmsize->height;


    if (s_mhb_camera.apbe_config_state == APBE_CONFIGURED) {

        CAM_ERR("ERROR: Weird! APBE still Configured\n");
        if (_mhb_csi_camera_stop_stream())
            s_mhb_camera.apbe_config_state = APBE_RESET;
        else
            s_mhb_camera.apbe_config_state = APBE_UNCONFIGURED;
    }


    /* Send the configuration to the APBE. */
    retry = UNIPRO_RECOVER_RETRIES;
    do {
        if (s_mhb_camera.apbe_config_state == APBE_UNCONFIGURED) {
            int res = _mhb_csi_camera_start_stream((uint8_t*)cdsi_config,
                                                   sizeof(*cdsi_config));
            if (!res)
                break;
            else if (retry == 0 || res == -EREMOTEIO)
                goto failed_stream_on;
        }
        mhb_camera_i2c_lock();

        CAM_ERR("UNIPRO Config failed. Reset & Retry APBE Status %d\n",
                s_mhb_camera.apbe_config_state);

        if (_mhb_camera_set_apbe_enable(SLAVE_STATE_RESET)) {
             CAM_ERR("Failed waiting for PEER_CONNECTED\n");
             mhb_camera_i2c_unlock();
             goto failed_stream_on;
        }

        pthread_mutex_lock(&s_mhb_camera.mutex);
        if (_mhb_camera_wait_for_response(&s_mhb_camera.slave_cond,
                MHB_CAM_PWRCTL_WAIT_MASK|MHB_PM_STATUS_PEER_CONNECTED,
                "PEER CONNECTED", MHB_PCTRL_OP_TIMEOUT_NS)) {

             CAM_ERR("Failed waiting for PEER_CONNECTED\n");
             s_mhb_camera.apbe_config_state = APBE_RESET;
             pthread_mutex_unlock(&s_mhb_camera.mutex);
             mhb_camera_i2c_unlock();
             continue;
        }
        pthread_mutex_unlock(&s_mhb_camera.mutex);
        mhb_camera_i2c_unlock();
        MHB_CAM_DEV_OP(s_mhb_camera.cam_device, stream_reset);
    } while (retry--);

    camera_ext_send_error(CAMERA_EXT_ERROR_CSI_RESET);
    s_mhb_camera.apbe_config_state = APBE_CONFIGURED;
    mhb_csi_camera_callback(MHB_CAMERA_NOTIFY_PREVIEW_ON);

    return MHB_CAMERA_EV_CONFIGURED;

failed_stream_on:
    camera_ext_send_error(CAMERA_EXT_ERROR_STREAM_ON);
    return MHB_CAMERA_EV_FAIL;
}

mhb_camera_sm_event_t mhb_camera_stream_off(void)
{
    //struct device *dev = CONTAINER_OF(&s_mhb_camera, struct device, private);

    CAM_DBG("\n");

    mhb_csi_camera_callback(MHB_CAMERA_NOTIFY_PREVIEW_OFF);

    if (_mhb_csi_camera_stop_stream())
        s_mhb_camera.apbe_config_state = APBE_RESET;
    else
        s_mhb_camera.apbe_config_state = APBE_UNCONFIGURED;

    return MHB_CAMERA_EV_DECONFIGURED;
}

mhb_camera_sm_event_t mhb_camera_lens_extend(void)
{
    MHB_CAM_DEV_OP(s_mhb_camera.cam_device, lens_extend);
    return MHB_CAMERA_EV_NONE;
}

mhb_camera_sm_event_t mhb_camera_lens_retract(void)
{
    if (APBE_CONFIGURED == s_mhb_camera.apbe_config_state) {
        CAM_INFO("APBE Configured in lens retract.");
        mhb_camera_stream_off();
    }

    MHB_CAM_DEV_OP(s_mhb_camera.cam_device, lens_retract);
    return MHB_CAMERA_EV_NONE;
}

/* CAMERA_EXT devops */
static int _power_on(struct device *dev, uint8_t mode)
{
    CAM_INFO("mhb_camera_csi: bootmode: %d\n", mode);

    if(mhb_camera_sm_execute(MHB_CAMERA_EV_POWER_ON_REQ))
        return -1;
    s_mhb_camera.bootmode = mode;
    return 0;
}

static int _power_off(struct device *dev)
{
    CAM_INFO("mhb_camera_csi : bootmode %d\n", s_mhb_camera.bootmode);

    mhb_camera_process_ctrl_cache(TRUE);
    if (s_mhb_camera.bootmode == CAMERA_EXT_BOOTMODE_DFU) {
         /* Send 2 Off events when powering off from DFU to ensure */
         /* we dont linger in WAIT_OFF states.*/
         mhb_camera_sm_execute(MHB_CAMERA_EV_POWER_OFF_REQ);
    }

    return mhb_camera_sm_execute(MHB_CAMERA_EV_POWER_OFF_REQ);
}

static int _stream_on(struct device *dev)
{
    CAM_INFO("mhb_camera_csi : bootmode %d\n", s_mhb_camera.bootmode);
    if (s_mhb_camera.bootmode == CAMERA_EXT_BOOTMODE_DFU) {
        CAM_ERR("ERROR: mhb_camera_csi: stream_on in DFU Mode\n");
        return -EINVAL;
    }

    return mhb_camera_sm_execute(MHB_CAMERA_EV_STREAM_ON_REQ);
}

static int _stream_off(struct device *dev)
{
    CAM_INFO("mhb_camera_csi\n");
    return mhb_camera_sm_execute(MHB_CAMERA_EV_STREAM_OFF_REQ);
}

static int _camera_attach(void *arg, const void *data)
{
    if (!arg || !data) {
        return -EINVAL;
    }

    struct device *dev = (struct device *)arg;
    enum base_attached_e state = *((enum base_attached_e *)data);

    CAM_DBG("state=%d\n", state);

    switch (state) {
    case BASE_DETACHED:
    case BASE_ATTACHED_OFF:
        /* trigger power off */
        _power_off(dev);
        break;
    case BASE_ATTACHED:
    case BASE_INVALID:
        /* Ignore */
        break;
    }

    return 0;
}

static int _dev_probe(struct device *dev)
{
    struct mhb_camera_s* mhb_camera = &s_mhb_camera;
    CAM_DBG("mhb_camera_csi\n");

    device_set_private(dev, (void*)mhb_camera);

#ifdef CONFIG_PM
    pm_register(&pm_callback);
#endif

    pthread_mutex_init(&mhb_camera->mutex, NULL);
    pthread_mutex_init(&mhb_camera->i2c_mutex, NULL);
    pthread_mutex_init(&mhb_camera->ctrl_mutex, NULL);

    pthread_cond_init(&mhb_camera->slave_cond, NULL);
    pthread_cond_init(&mhb_camera->cdsi_cond, NULL);

    mhb_camera->soc_status = SOC_DISABLED;
    mhb_camera->apbe_enabled = 0;
    mhb_camera->apbe_en_state = MHB_PM_STATUS_PEER_NONE;
    mhb_camera->apbe_config_state = APBE_UNCONFIGURED;
    mhb_camera->mhb_wait_event = MHB_CAM_WAIT_EV_NONE;
    mhb_camera->bootmode = CAMERA_EXT_BOOTMODE_NORMAL;
    mhb_camera->cam_i2c = NULL;

    memset(mhb_camera->callbacks, 0x00, sizeof(mhb_camera->callbacks));

    /* Register for attach notifications. This will callback immediately. */
    if (mods_attach_register(_camera_attach, dev)) {
        dbg("ERROR: failed to register attach notifier\n");
    }

    return 0;
}

static int _dev_open(struct device *dev)
{
    struct mhb_camera_s* mhb_camera = (struct mhb_camera_s*)device_get_private(dev);
    int ret = -ENODEV;
    CAM_DBG("mhb_camera_csi\n");

    if (mhb_camera->mhb_device) {
        CAM_ERR("ERROR: already opened.\n");
        return -EBUSY;
    }

    mhb_camera->cam_device = device_open(DEVICE_TYPE_MHB_CAMERA_HW, MHB_CAM_DRIVER_ID);
    if (!mhb_camera->cam_device) {
        CAM_ERR("ERROR: Failed to open Cam Device\n");
        goto open_failed;
    }

    mhb_camera->slave_pwr_ctrl = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW, MHB_ADDR_CDSI1);
    if (!mhb_camera->slave_pwr_ctrl) {
        CAM_ERR("ERROR: Failed to open SLAVE Power Control\n");
        goto open_failed;
    }

    mhb_camera->mhb_device = device_open(DEVICE_TYPE_MHB, MHB_ADDR_CDSI1);
    if (!mhb_camera->mhb_device) {
        CAM_ERR("ERROR: failed to open MHB device.\n");
        goto open_failed;
    }
    mhb_camera_sm_init();

    ret = device_slave_pwrctrl_register_status_callback(mhb_camera->slave_pwr_ctrl,
                                                        _mhb_camera_slave_status_callback);
    if(ret) {
        CAM_ERR("ERROR: Failed to register callback %d\n", ret);
        goto open_failed;
    }

    ret = device_mhb_register_receiver(mhb_camera->mhb_device, MHB_ADDR_CDSI1,
                                       _mhb_csi_camera_handle_msg);

    if(ret) {
        CAM_ERR("ERROR: Failed to register device_mhb_register_receiver %d\n", ret);
        goto open_failed;
    }
    MHB_CAM_DEV_OP(mhb_camera->cam_device, set_err_callback, mhb_cam_error_cb);

    return 0;

open_failed:

    device_close(mhb_camera->cam_device);
    device_close(mhb_camera->slave_pwr_ctrl);
    device_close(mhb_camera->mhb_device);
    mhb_camera->cam_device = NULL;
    mhb_camera->slave_pwr_ctrl = NULL;
    mhb_camera->mhb_device = NULL;
    return ret;
}

static void _dev_close(struct device *dev)
{
    CAM_DBG("mhb_camera_csi\n");
    struct mhb_camera_s* mhb_camera = (struct mhb_camera_s*)device_get_private(dev);

    if (mhb_camera->mhb_device) {
        device_mhb_unregister_receiver(mhb_camera->mhb_device, MHB_ADDR_CDSI1,
                                       _mhb_csi_camera_handle_msg);
    }

    if (mhb_camera->slave_pwr_ctrl) {
        device_slave_pwrctrl_unregister_status_callback(mhb_camera->slave_pwr_ctrl,
                                                        _mhb_camera_slave_status_callback);
    }

    device_close(mhb_camera->mhb_device);
    device_close(mhb_camera->slave_pwr_ctrl);
    device_close(mhb_camera->cam_device);
    mhb_camera->cam_device = NULL;
    mhb_camera->mhb_device = NULL;
    mhb_camera->slave_pwr_ctrl = NULL;
}

static void mhb_cam_error_cb(int err) {
    /* Something failed on device. Pass the error up, */
    /* so that app can clean up */
    CAM_ERR("FATAL_ERROR : Camera module failed. Reset %d\n", err);
    camera_ext_send_error(CAMERA_EXT_ERROR_FATAL);
    _power_off(NULL);
}

static int _mhb_camera_ext_ctrl_cache(struct device *dev,
    uint32_t idx, uint8_t *ctrl_val, uint32_t ctrl_val_size)
{
    struct cached_ctrls_node *item;

    item = kmm_malloc(sizeof(struct cached_ctrls_node));
    if (!item) return -1;

    item->ctrl_val = kmm_malloc(ctrl_val_size);
    if (!item->ctrl_val) {
        kmm_free(item);
        return -1;
    }

    memcpy(item->ctrl_val, ctrl_val, ctrl_val_size);
    item->dev = dev;
    item->idx = idx;
    item->ctrl_val_size = ctrl_val_size;
    pthread_mutex_lock(&s_mhb_camera.ctrl_mutex);
    list_add(&cached_ctrl_list, &item->node);
    pthread_mutex_unlock(&s_mhb_camera.ctrl_mutex);

    return 0;
}

static int _mhb_camera_ext_ctrl_set(struct device *dev,
    uint32_t idx, uint8_t *ctrl_val, uint32_t ctrl_val_size)
{
    uint8_t state = mhb_camera_sm_get_state();
    if (state == MHB_CAMERA_STATE_OFF ||
        state == MHB_CAMERA_STATE_WAIT_OFF) {
        CAM_ERR("ERROR: Camera Off : State %d\n", state);
        return -ENODEV;
    }
    if (s_mhb_camera.soc_status != SOC_ENABLED ||
        state == MHB_CAMERA_STATE_WAIT_POWER_ON||
        state == MHB_CAMERA_STATE_WAIT_STREAM) {
        CTRL_DBG("Cam not ready, cache ctrl. CtrlID 0x%08x Size %d State %s\n",
                 idx, ctrl_val_size, mhb_camera_sm_state_str(state));

        _mhb_camera_ext_ctrl_cache(dev, idx, ctrl_val, ctrl_val_size);

        return 0;
    }
    return camera_ext_ctrl_set(dev, idx, ctrl_val, ctrl_val_size);
}

static int _mhb_camera_ext_ctrl_get(struct device *dev,
    uint32_t idx, uint8_t *ctrl_val, uint32_t ctrl_val_size)
{
    uint8_t state = mhb_camera_sm_get_state();
    static uint8_t cam_ready = 1;
    if (state == MHB_CAMERA_STATE_OFF ||
        state == MHB_CAMERA_STATE_WAIT_OFF) {
        CAM_ERR("ERROR: Camera Off : State %d\n", state);
        return -ENODEV;
    }

    if (s_mhb_camera.soc_status != SOC_ENABLED ||
        state == MHB_CAMERA_STATE_WAIT_POWER_ON||
        state == MHB_CAMERA_STATE_WAIT_STREAM) {
        if (cam_ready) {
            CAM_DBG("Cam not ready, try again. CtrlID 0x%08x State %s\n",
                    idx, mhb_camera_sm_state_str(state));
            cam_ready = 0;
        }
        return -EAGAIN;
    }

    if (!cam_ready) {
            CAM_DBG("Cam Ready, Process. CtrlID 0x%08x State %s\n",
                    idx, mhb_camera_sm_state_str(state));
            cam_ready = 1;
    }

    return camera_ext_ctrl_get(dev, idx, ctrl_val, ctrl_val_size);
}

static struct device_camera_ext_dev_type_ops camera_ext_type_ops = {
    .register_event_cb = camera_ext_register_event_cb,
    .power_on          = _power_on,
    .power_off         = _power_off,
    .stream_on         = _stream_on,
    .stream_off        = _stream_off,
    .input_enum        = camera_ext_input_enum,
    .input_get         = camera_ext_input_get,
    .input_set         = camera_ext_input_set,
    .format_enum       = camera_ext_format_enum,
    .format_get        = camera_ext_format_get,
    .format_set        = camera_ext_format_set,
    .frmsize_enum      = camera_ext_frmsize_enum,
    .frmival_enum      = camera_ext_frmival_enum,
    .stream_set_parm   = camera_ext_stream_set_parm,
    .stream_get_parm   = camera_ext_stream_get_parm,
    .ctrl_get_cfg      = camera_ext_ctrl_get_cfg,
    .ctrl_get          = _mhb_camera_ext_ctrl_get,
    .ctrl_set          = _mhb_camera_ext_ctrl_set,
    .ctrl_try          = camera_ext_ctrl_try,
    .set_phone_ver     = camera_ext_set_phone_ver,
};

static struct device_driver_ops camera_ext_driver_ops = {
    .probe    = _dev_probe,
    .open     = _dev_open,
    .close    = _dev_close,
    .type_ops = &camera_ext_type_ops,
};

struct device_driver cam_ext_mhb_driver = {
    .type = DEVICE_TYPE_CAMERA_EXT_HW,
    .name = "Motorola",
    .desc = "Motorola MHB Camera",
    .ops  = &camera_ext_driver_ops,
};
