/**
 * Copyright (c) 2014-2015 Google Inc.
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
 *
 * @brief LG4892 LCD Panel TouchScreen Driver
 */

#include <nuttx/i2c.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <nuttx/config.h>
#include <arch/board/lg4892.h>

/* Touchscreen controller I2C address */
#define TS_I2C_ADDR                     0x34

/* Registers */
#define TS_REGL_CMD                     0x00
#define TS_REGH_CMD                     0x10

#define TS_REGL_MODE_CONTROL            0x01
#define TS_REGL_XY_RESOLUTION_HIGH      0x02
#define TS_REGL_X_RESOLUTION_LOW        0x03
#define TS_REGL_Y_RESULUTION_LOW        0x04
#define TS_REGL_CONTACT_ON_EVENT_THRES  0x05
#define TS_REGL_OPERATION_MODE          0x09
#define TS_REGL_ROW_NUM                 0x0B
#define TS_REGL_COL_NUM                 0x0C
#define TS_REGL_RAW_TRACK               0x0E
#define TS_REGL_EVENT_PKT_SZ            0x0F
#define TS_REGL_INPUT_EVENT             0x10
#define TS_REGL_UCMD                    0xA0
#define TS_REGL_UCMD_RESULT_LENGTH      0xAE
#define TS_REGL_UCMD_RESULT             0xAF
#define TS_BOOT_MODE                    0xB0
#define TS_FW_VERSION                   0xC2

/* Universal commands */
#define TS_UNIV_ENTER_TESTMODE          0x40
#define TS_UNIV_TESTA_START             0x41
#define TS_UNIV_GET_RAWDATA             0x44
#define TS_UNIV_TESTB_START             0x48
#define TS_UNIV_GET_OPENSHORT_TEST      0x50
#define TS_UNIV_EXIT_TESTMODE           0x6F

/* Event types */
#define TS_ET_LOG                       0xD
#define TS_ET_NOTIFY                    0xE
#define TS_ET_ERROR                     0xF

/* ISP mode */
#define ISP_ENTRY1 {0xAC, 0xCE, 0xFF, 0xFD, 0xAC, 0xCE}
#define ISP_ENTRY2 {0xAC, 0xCE, 0xFF, 0xFE, 0xAC, 0xCE}
#define ISP_ENTRY3 {0xAC, 0xCE, 0xFF, 0xFF, 0xAC, 0xCE}
#define ISP_UNLOCK1 {0x00, 0x04, 0xFE, 0xDC, 0xBA, 0x98}
#define ISP_UNLOCK2 {0x00, 0x04, 0x76, 0x54, 0x32, 0x10}
#define ISP_UNLOCK3 {0x00, 0x4C, 0x00, 0x00, 0x00, 0x01}
#define ISP_LOCK {0x00, 0x04, 0x05, 0xFA, 0x00, 0x00}
#define ISP_FULL_ERASE {0x00, 0x10, 0x00, 0x00, 0x10, 0x44}
#define ISP_CLEAR_DONE {0x00, 0x0C, 0x00, 0x00, 0x00, 0x20}
#define ISP_WRITE_MODE {0x00, 0x10, 0x00, 0x00, 0x10, 0x01}
#define ISP_WRITE {0x00, 0x10, 0x00, 0x00, 0x10, 0x41}
#define ISP_READ_MODE {0x00, 0x10, 0x00, 0x00, 0x10, 0x08}
#define ISP_READ {0x00, 0x10, 0x00, 0x00, 0x10, 0x48}

#define MELFAS_TS_BIN_VERSION_OFFSET    10

extern const uint32_t MELFAS_binary_nLength;
extern const uint8_t MELFAS_binary[];

static struct i2c_dev_s *lg4892_ts_i2cdev = NULL;

/* disable the verbose debug output */
#undef lldbg
#define lldbg(x...)

int lg4892_ts_init(uint8_t i2cbus)
{
    lg4892_ts_i2cdev = up_i2cinitialize(i2cbus);
    if (!lg4892_ts_i2cdev) {
        lldbg("could not init I2C!\n", __func__);
        return -2;
    }

    return 0;
}

void lg4892_ts_close(void)
{
    up_i2cuninitialize(lg4892_ts_i2cdev);
}

int lg4892_ts_get_bin_fw_version(uint8_t *app_maj, uint8_t *app_min,
    uint8_t *core_maj, uint8_t *core_min)
{
    const uint8_t *bin_version;

    bin_version = MELFAS_binary + MELFAS_binary_nLength -
        MELFAS_TS_BIN_VERSION_OFFSET;
    *core_maj = bin_version[0];
    *core_min = bin_version[1];
    *app_maj = bin_version[4];
    *app_min = bin_version[5];

    return 0;
}

static int lg4892_ts_simple_i2c_transaction(
    uint8_t *cmd, uint8_t cmd_sz, uint8_t *buf, uint8_t buf_sz)
{
    int ret;
    #ifdef TS_SIMPLE_I2C_TRANSACTION_DBG
    uint8_t i;
    #endif
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = cmd,
            .length = cmd_sz,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = buf,
            .length = buf_sz,
        },
    };

    #ifdef TS_SIMPLE_I2C_TRANSACTION_DBG
    lldbg("Command sent to TS: ");
    for (i = 0; i < cmd_sz; i++) {
        lldbg("0x%02X ", cmd[i]);
    }
    lldbg("\n");
    #endif
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 2);
    if (ret != OK) {
        lldbg("I2C transaction failed! (%d)\n", ret);
    #ifdef TS_SIMPLE_I2C_TRANSACTION_DBG
    } else {
        lldbg("Response from TS:   ");
        for (i = 0; i < buf_sz; i++) {
            lldbg("0x%02X ", buf[i]);
        }
        lldbg("\n");
    #endif
    }

    return ret;
}

static int lg4892_ts_isp_entry(void)
{
    int ret;
    uint8_t entry_a[6] = ISP_ENTRY1;
    uint8_t entry_b[6] = ISP_ENTRY2;
    uint8_t entry_c[6] = ISP_ENTRY3;
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = entry_a,
            .length = 6,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = entry_b,
            .length = 6,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = entry_c,
            .length = 6,
        },
    };

    lldbg("Sending ENTRY sequence...\n");
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 1);
    ret += I2C_TRANSFER(lg4892_ts_i2cdev, &msg[1], 1);
    ret += I2C_TRANSFER(lg4892_ts_i2cdev, &msg[2], 1);
    if (ret != OK) {
        lldbg("could not transfer ISP_ENTRY!\n");
    }
    lldbg("ENTRY sequence sent.\n");

    return ret;
}

static int lg4892_ts_isp_unlock(void)
{
    int ret;
    uint8_t unlock_a[6] = ISP_UNLOCK1;
    uint8_t unlock_b[6] = ISP_UNLOCK2;
    uint8_t unlock_c[6] = ISP_UNLOCK3;
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = unlock_a,
            .length = 6,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = unlock_b,
            .length = 6,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = unlock_c,
            .length = 6,
        },
    };

    lldbg("Sending UNLOCK sequence...\n");
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 1);
    ret += I2C_TRANSFER(lg4892_ts_i2cdev, &msg[1], 1);
    ret += I2C_TRANSFER(lg4892_ts_i2cdev, &msg[2], 1);
    if (ret != OK) {
        lldbg("could not transfer ISP_UNLOCK!\n");
    }
    lldbg("UNLOCK sequence sent.\n");

    return ret;
}

static int lg4892_ts_isp_write_done(void)
{
    int ret;
    uint8_t cmd[2] = {0x00, 0x0C};
    uint8_t rbuf[4];
    uint16_t i = 0;
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = cmd,
            .length = 2,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = rbuf,
            .length = 4,
        }
    };

    lldbg("Proceeding with ISP Write done...\n");
    while (1) {
        ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 2);
        if (ret != OK) {
            lldbg("isp write done i2c_transfer failed!\n");
            return ret;
        }
        lldbg("rbuf={0x%02X, 0x%02X, 0x%02X, 0x%02X}\n",
            rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
        if (rbuf[3] & 0x20)
            break;
        if (i++ > 100) {
            lldbg("isp write wait-done failed!\n");
            return -4;
        }
    }
    lldbg("ISP Write done done.\n");

    return ret;
}

static int lg4892_ts_isp_clean(void)
{
    int ret;
    uint8_t clr_done[6] = ISP_CLEAR_DONE;
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = clr_done,
            .length = 6,
        },
    };

    lldbg("Sending CLEAR_DONE sequence...\n");
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 1);
    if (ret != OK) {
        lldbg("could not transfer CLEAR_DONE!\n");
        return ret;
    }
    lldbg("CLEAR_DONE sequence done.\n");

    return ret;
}

static int lg4892_ts_isp_erase(void)
{
    int ret;
    uint8_t erase[6] = ISP_FULL_ERASE;
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = erase,
            .length = 6,
        },
    };

    lldbg("Sending ISP_FULL_ERASE sequence...\n");
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 1);
    if (ret != OK) {
        lldbg("could not transfer ISP_FULL_ERASE!\n");
        return ret;
    }
    ret = lg4892_ts_isp_write_done();
    if (ret != 0) {
        return ret;
    }
    ret = lg4892_ts_isp_clean();
    if (ret != 0) {
        return ret;
    }
    lldbg("ISP_FULL_ERASE sequence done.\n");

    return ret;
}

static int lg4892_ts_isp_lock(void)
{
    int ret;
    uint8_t lock[6] = ISP_LOCK;
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = lock,
            .length = 6,
        },
    };

    lldbg("Sending ISP_LOCK sequence...\n");
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 1);
    if (ret != OK) {
        lldbg("could not transfer ISP_LOCK!\n");
        return ret;
    }
    lldbg("ISP_LOCK sequence sent.\n");

    return ret;
}


int lg4892_ts_get_fw_version(uint8_t *app_maj, uint8_t *app_min,
    uint8_t *core_maj, uint8_t *core_min)
{
    int ret;
    uint8_t cmd[2] = {TS_REGL_CMD, TS_FW_VERSION};
    uint8_t version[4];

    ret = lg4892_ts_simple_i2c_transaction(cmd, 2, version, 4);
    if (ret != OK) {
        lldbg("failed to get fw version! (%d)\n", ret);
    } else {
        *app_maj = version[0];
        *app_min = version[1];
        *core_maj = version[2];
        *core_min = version[3];
        lldbg("fw version is 0x%02X 0x%02X 0x%02X 0x%02X\n",
            *app_maj, *app_min, *core_maj, *core_min);
    }

    return ret;
}

int lg4892_ts_get_row_col(uint8_t *row, uint8_t *col)
{
    int ret;
    uint8_t cmd[2] = {TS_REGH_CMD, TS_REGL_ROW_NUM};
    uint8_t rbuf[2];

    ret = lg4892_ts_simple_i2c_transaction(cmd, 2, rbuf, 2);
    if (ret != OK) {
        lldbg("failed to get row and column number! (%d)\n", ret);
    } else {
        *row = rbuf[0];
        *col = rbuf[1];
        lldbg("row=%u col=%u\n", *row, *col);
    }

    return ret;
}

static int lg4892_ts_isp_addr(const uint32_t addr)
{
    int ret = 0;
    uint8_t wbuf_addr[6];
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = wbuf_addr,
            .length = 6,
        },
    };

    wbuf_addr[0] = 0x00;
    wbuf_addr[1] = 0x14;
    wbuf_addr[2] = (addr >> 24) & 0xFF;
    wbuf_addr[3] = (addr >> 16) & 0xFF;
    wbuf_addr[4] = (addr >> 8) & 0xFF;
    wbuf_addr[5] = (addr >> 0) & 0xFF;

    lldbg("ISP Addr procedure (0x%08X)...\n", addr);
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 1);
    if (ret != OK) {
        lldbg("isp fw-addr i2c_master_send failed\n");
    } else {
        lldbg("ISP Addr procedure done.\n");
    }

    return ret;
}


static int lg4892_ts_flash_data(const uint8_t *data, const uint32_t addr)
{
    int ret;
    uint8_t write_mode[6] = ISP_WRITE_MODE;
    uint8_t write_cmd[6] = ISP_WRITE;
    uint8_t wbuf_a[6] = {0x00, 0x28, };
    uint8_t wbuf_b[6] = {0x00, 0x2C, };
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = write_mode,
            .length = 6,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = wbuf_a,
            .length = 6,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = wbuf_b,
            .length = 6,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = write_cmd,
            .length = 6,
        },
    };

    lldbg("Flashing 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X at address 0x%08X...\n",
        data[0], data[1], data[2], data[3],
        data[4], data[5], data[6], data[7],
        addr);
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 1);
    if (ret != OK) {
        lldbg("isp write mode i2c_master_send failed\n");
        return ret;
    }

    wbuf_a[2] = data[3];
    wbuf_a[3] = data[2];
    wbuf_a[4] = data[1];
    wbuf_a[5] = data[0];

    ret = I2C_TRANSFER(lg4892_ts_i2cdev, &msg[1], 1);
    if (ret != OK) {
        lldbg("isp fw-data(a) i2c_master_send failed\n");
        return ret;
    }

    wbuf_b[2] = data[7];
    wbuf_b[3] = data[6];
    wbuf_b[4] = data[5];
    wbuf_b[5] = data[4];

    ret = I2C_TRANSFER(lg4892_ts_i2cdev, &msg[2], 1);
    if (ret != OK) {
        lldbg("isp fw-data(b) i2c_master_send failed\n");
        return ret;
    }
    ret = lg4892_ts_isp_addr(addr);
    if (ret != 0) {
        return ret;
    }
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, &msg[3], 1);
    if (ret != OK) {
        lldbg("isp write cmd i2c_master_send failed\n");
        return ret;
    }
    ret = lg4892_ts_isp_write_done();
    if (ret != 0) {
        return ret;
    }
    ret = lg4892_ts_isp_clean();
    if (ret != 0) {
        lldbg("lg4892_ts_isp_clean() failed\n");
    } else {
        lldbg("Flashing at address 0x%08X done.\n", addr);
    }

    return ret;
}

static int lg4892_ts_verify_data(uint8_t *data, const uint32_t addr)
{
    int ret;
    uint8_t read_a[2] = {0x00, 0x30};
    uint8_t read_b[2] = {0x00, 0x34};
    uint8_t rbuf[4];
    uint8_t r_cmp[8];
    uint8_t read_mode[6] = ISP_READ_MODE;
    uint8_t read_cmd[6] = ISP_READ;
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = read_mode,
            .length = 6,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = read_cmd,
            .length = 6,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = read_a,
            .length = 2,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = rbuf,
            .length = 4,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = read_b,
            .length = 2,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = rbuf,
            .length = 4,
        },
    };

    lldbg("Verifying 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X at address 0x%08X...\n",
        data[0], data[1], data[2], data[3],
        data[4], data[5], data[6], data[7],
        addr);
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 1);
    if (ret != OK) {
        lldbg("isp read mode i2c_master_send failed\n");
        return ret;
    }

    ret = lg4892_ts_isp_addr(addr);
    if (ret != 0) {
        return ret;
    }

    ret = I2C_TRANSFER(lg4892_ts_i2cdev, &msg[1], 1);
    if (ret != OK) {
        lldbg("isp read cmd i2c_master_send failed\n");
        return ret;
    }

    ret = I2C_TRANSFER(lg4892_ts_i2cdev, &msg[2], 2);
    if (ret != OK) {
        lldbg("isp read data(a) i2c_master_send failed\n");
        return ret;
    }

    r_cmp[0] = rbuf[3];
    r_cmp[1] = rbuf[2];
    r_cmp[2] = rbuf[1];
    r_cmp[3] = rbuf[0];

    ret = I2C_TRANSFER(lg4892_ts_i2cdev, &msg[4], 2);
    if (ret != OK) {
        lldbg("isp read data(b) i2c_master_send failed\n");
        return ret;
    }

    r_cmp[4] = rbuf[3];
    r_cmp[5] = rbuf[2];
    r_cmp[6] = rbuf[1];
    r_cmp[7] = rbuf[0];

    lldbg("Data to write is 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        data[0], data[1], data[2], data[3],
        data[4], data[5], data[6], data[7]);
    lldbg("Flashed data is  0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        r_cmp[0], r_cmp[1], r_cmp[2], r_cmp[3],
        r_cmp[4], r_cmp[5], r_cmp[6], r_cmp[7]);

    ret = lg4892_ts_isp_write_done();
    if (ret != 0) {
        return ret;
    }

    if (memcmp(data, r_cmp, 8)) {
        lldbg("data comparison failed!\n");
        return -10;
    }

    ret = lg4892_ts_isp_clean();
    if (ret != 0) {
        return ret;
    }
    lldbg("Address 0x%08X verified.\n");

    return ret;
}

int lg4892_ts_firmware_update(void)
{
    int ret;
    uint32_t addr = MELFAS_binary_nLength - 8;
    uint8_t retry = 0;

    lldbg("updating firmware...\n");
    for (retry = 0; retry < 20; retry++) {
        ret = lg4892_ts_isp_entry();
        if (ret != 0) {
            lldbg("lg4892_ts_isp_entry() failed! (%d)\n", ret);
            continue;
        }
        ret = lg4892_ts_isp_unlock();
        if (ret != 0) {
            lldbg("lg4892_ts_isp_unlock() failed! (%d)\n", ret);
            continue;
        }
        ret = lg4892_ts_isp_erase();
        if (ret != 0) {
            lldbg("lg4892_ts_isp_erase() failed! (%d)\n", ret);
            continue;
        }

        for (; addr < MELFAS_binary_nLength; addr -= 8) {
            lldbg("retry = %d, addr=0x%08X data=0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
                retry, addr,
                MELFAS_binary[addr], MELFAS_binary[addr + 1],
                MELFAS_binary[addr + 2],
                MELFAS_binary[addr + 3],
                MELFAS_binary[addr + 4],
                MELFAS_binary[addr + 5],
                MELFAS_binary[addr + 6],
                MELFAS_binary[addr + 7]);
            if (lg4892_ts_flash_data((uint8_t *) &MELFAS_binary[addr],
                addr)) {
                lldbg("isp flash failed!\n");
                break;
            }
            if (lg4892_ts_verify_data((uint8_t *) &MELFAS_binary[addr],
                addr)) {
                lldbg("isp verify failed!\n");
                break;
            }
            retry = 0;
        }
        if (addr > MELFAS_binary_nLength) {
            break;
        }
    }
    if (retry == 20) {
        lldbg("max number of retry reached, fw update aborted!\n");
        return -10;
    }
    ret = lg4892_ts_isp_lock();
    if (ret != 0) {
        lldbg("firmware updated but lg4892_ts_isp_lock() failed! (%d)\n",
            ret);
    } else {
        lldbg("firmware update completed.\n");
    }

    return ret;
}

int lg4892_ts_get_boot_mode(uint8_t *mode)
{
    int ret;
    uint8_t cmd[2] = {TS_REGL_CMD, TS_BOOT_MODE};

    lldbg("Retrieving TS contact on threshold...\n");
    ret = lg4892_ts_simple_i2c_transaction(cmd, 2, mode, 1);
    if (ret != OK) {
        lldbg("failed to get boot mode! (%d)\n", ret);
    #ifdef TS_GET_BOOT_MODE_DBG
    } else {
        lldbg("boot mode: 0x%02X\n", *mode);
    #endif
    }

    return ret;
}

int lg4892_ts_get_events(uint8_t *buf, uint8_t *sz)
{
    int ret;
    uint8_t reg[2] = {TS_REGH_CMD, TS_REGL_INPUT_EVENT};
    uint8_t cmd[2] = {TS_REGH_CMD, TS_REGL_EVENT_PKT_SZ};
    struct i2c_msg_s msg[] = {
        {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = cmd,
            .length = 2,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = sz,
            .length = 1,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = 0,
            .buffer = reg,
            .length = 2,
        }, {
            .addr = TS_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = buf,
            .length = *sz,
        },
    };

    lldbg("Retrieving TS events...\n");
    ret = I2C_TRANSFER(lg4892_ts_i2cdev, msg, 2);
    if (ret != OK) {
        lldbg("failed to retrieve number of events\n");
        return ret;
    }
    lldbg("Number of TS events to get: %u\n", *sz);
    if (*sz == 0) {
        return 0;
    }
    buf = malloc(*sz * sizeof(uint8_t));
    if (buf == NULL) {
        lldbg("failed to allocate memory for buf!\n");
        return -ENOMEM;
    }
    msg[3].buffer = buf;

    ret = I2C_TRANSFER(lg4892_ts_i2cdev, &msg[2], 2);
    if (ret != OK) {
        lldbg("failed to retrieve events\n");
    } else {
        lldbg("events retrieved.\n");
    }

    return *sz;
}


int lg4892_ts_get_ctrl_state(uint8_t *ctrl_state)
{
    int ret;
    uint8_t cmd[2] = {TS_REGH_CMD, TS_REGL_MODE_CONTROL};

    lldbg("Retrieving TS control state...\n");
    ret = lg4892_ts_simple_i2c_transaction(cmd, 2, ctrl_state, 1);
    if (ret != OK) {
        lldbg("failed to retrieve control state! (%d)\n", ret);
    #ifdef TS_GET_CTRL_STATE_DBG
    } else {
        lldbg("TS control state: %u\n", *ctrl_state);
    #endif
    }

    return ret;
}


int lg4892_ts_get_operation_mode(uint8_t *mode)
{
    int ret;
    uint8_t cmd[2] = {TS_REGH_CMD, TS_REGL_OPERATION_MODE};

    lldbg("Retrieving TS operation mode...\n");
    ret = lg4892_ts_simple_i2c_transaction(cmd, 2, mode, 1);
    if (ret != OK) {
        lldbg("failed to retrieve operation mode! (%d)\n", ret);
    #ifdef TS_GET_OPERATION_MODE_DBG
    } else {
        lldbg("TS operation mode: %u\n", *mode);
    #endif
    }

    return ret;
}

int lg4892_ts_get_resolution(uint16_t *x, uint16_t *y)
{
    int ret;
    uint8_t cmd[2] = {TS_REGH_CMD, TS_REGL_XY_RESOLUTION_HIGH};
    uint8_t buf[3];

    lldbg("Retrieving TS resolution...\n");
    ret = lg4892_ts_simple_i2c_transaction(cmd, 2, buf, 3);
    if (ret != OK) {
        lldbg("failed to retrieve TS resolution! (%d)\n", ret);
    } else {
        *x = buf[1] | ((buf[0] & 0x0F) << 8);
        *y = buf[2] | ((buf[0] & 0xF0) << 4);
        lldbg("TS resolution: X=0x%03X Y=0x%03X\n", *x, *y);
    }

    return ret;
}

int lg4892_ts_get_contact_threshold(uint8_t *thres)
{
    int ret;
    uint8_t cmd[2] = {TS_REGH_CMD, TS_REGL_CONTACT_ON_EVENT_THRES};

    lldbg("Retrieving TS contact on threshold...\n");
    ret = lg4892_ts_simple_i2c_transaction(cmd, 2, thres, 1);
    if (ret != OK) {
        lldbg("failed to retrieve TS contact on threshold! (%d)\n",
            ret);
    #ifdef TS_GET_CONTACT_THRESHOLD_DBG
    } else {
        lldbg("TS contact on threshold: %u\n", *thres);
    #endif
    }

    return ret;
}
