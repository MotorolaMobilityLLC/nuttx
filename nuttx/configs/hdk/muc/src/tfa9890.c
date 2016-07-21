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
#include <ctype.h>
#include <errno.h>
#include <debug.h>
#include <semaphore.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <arch/byteorder.h>
#include <nuttx/audio/audio.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <arch/board/tfa9890_firmware.h>
#include <arch/board/tfa9890_registers.h>
#include <nuttx/tfa9890.h>

#define CLRBIT(val, mask)   (val & ~(mask))
#define SETBIT(val, mask)   (val | mask)

#define CLRBITS(val, mask)  CLRBIT(val, mask)
#define SETBITS(val, mask, s) (CLRBITS(val, mask) | s)

#define TFA9890_STATUS_UP_MASK  (TFA9890_STATUS_PLLS | \
                    TFA9890_STATUS_CLKS | \
                    TFA9890_STATUS_VDDS | \
                    TFA9890_STATUS_ARFS | TFA9890_STATUS_MTPB)
#define TFA9890_STATUS_UP  (TFA9890_STATUS_PLLS | \
                    TFA9890_STATUS_CLKS | \
                    TFA9890_STATUS_VDDS | \
                    TFA9890_STATUS_ARFS )

struct tfa9890_dev_s {
    struct tfa9890_lowerhalf_s dev;
    bool is_dsp_cfg_done;
    FAR struct i2c_dev_s *i2c;
    uint32_t i2c_addr;
    int current_eq;
    int vol_step;
    int speaker_imp;
    int type;
    int32_t sys_vol_db;
    int preset_update_pending;
    sem_t preset_lock;
};

static int tfa9890_reg_read(FAR struct tfa9890_dev_s *priv, uint8_t reg);
static int tfa9890_reg_write(FAR struct tfa9890_dev_s *priv, uint8_t reg,
                uint16_t val);
static int tfa9890_modify(FAR struct tfa9890_dev_s *priv, uint16_t reg,
                uint16_t mask, uint16_t s, uint16_t offset);
static int tfa9890_wait_pll_sync(struct tfa9890_dev_s *priv);

/* lower half ops */
static int      tfa9890_configure(FAR struct tfa9890_lowerhalf_s *dev,
                  FAR const struct tfa9890_caps_s *caps);
static int      tfa9890_stop(FAR struct tfa9890_lowerhalf_s *dev);
static int      tfa9890_start(FAR struct tfa9890_lowerhalf_s *dev);

static const struct tfa9890_ops_s g_audioops =
{
    .configure = tfa9890_configure,     /* configure      */
    .start = tfa9890_start,         /* start          */
    .stop = tfa9890_stop,          /* stop           */
};

static uint8_t chunk_buf[TFA9890_MAX_I2C_SIZE + 1];
static sem_t bulk_write_lock = SEM_INITIALIZER(1);

#ifdef DEBUG
static void tfa9890_dump_reg(FAR struct tfa9890_dev_s *priv, uint16_t reg)
{
    int ret = tfa9890_reg_read(priv, reg);
    if (ret < 0)
    {
         lldbg("failed to read 0x%04x\n", reg);
         return;
    }
    lldbg("tfa9890 reg 0x%04x value 0x%04x\n", reg, ret);
}

static void tfa9890_dump_regs(FAR struct tfa9890_dev_s *priv)
{
     uint16_t i;

     for (i = 0; i <= 0xB; i++)
     {
          tfa9890_dump_reg(priv, i);
     }
     tfa9890_dump_reg(priv, 0x80);
}
#endif

int tfa9890_reg_read(FAR struct tfa9890_dev_s *priv, uint8_t reg)
{
     uint8_t reg_val[2];
     int ret;

     I2C_SETADDRESS(priv->i2c, priv->i2c_addr, 7);

     ret = I2C_WRITEREAD(priv->i2c, (uint8_t *)&reg, sizeof(reg),
              reg_val, sizeof(reg_val));
     if (ret)
         return ret;

#ifdef CONFIG_ENDIAN_BIG
     ret = *((uint16_t *)reg_val);
#else
     ret = be16_to_cpu(*((uint16_t *)reg_val));
#endif
     return ret;
}

int tfa9890_reg_write(FAR struct tfa9890_dev_s *priv, uint8_t reg, uint16_t val)
{
    uint8_t buf[3];

    I2C_SETADDRESS(priv->i2c, priv->i2c_addr, 7);

    buf[0] = reg;
#ifdef CONFIG_ENDIAN_BIG
    buf[1] = val & 0x00ff;
    buf[2] = (val & 0xff00) >> 8;
#else
    buf[2] = val & 0x00ff;
    buf[1] = (val & 0xff00) >> 8;
#endif
    return I2C_WRITE(priv->i2c, (uint8_t *)buf, 4);
}

static int tfa9890_modify(FAR struct tfa9890_dev_s *priv, uint16_t reg,
                                    uint16_t mask, uint16_t s, uint16_t offset)
{
    int val = tfa9890_reg_read(priv, reg);

    if (val < 0)
    {
        lldbg("reg_read failed for 0x%02x rv = %d\n", reg, val);
        return val;
    }

    val = SETBITS(val, mask << offset, s << offset);
    return tfa9890_reg_write(priv, reg, val);
}

static int tfa9890_bulk_read(FAR struct i2c_dev_s *i2c_dev, uint32_t i2c_addr,
                                           uint8_t reg, uint8_t *data, int len)
{
    int ret = 0;
    int offset = 0;
    int remaining_bytes = len;
    int chunk_size = TFA9890_MAX_I2C_SIZE;

    struct i2c_msg_s msgs[] =
    {
        {
            .addr = i2c_addr,
            .flags = 0,
            .buffer = &reg,
            .length = 1,
        },
        {
            .addr = i2c_addr,
            .flags = I2C_M_READ,
            .buffer = NULL,
            .length = len,
        },
    };

    while (remaining_bytes > 0)
    {
        if (remaining_bytes < chunk_size)
            chunk_size = remaining_bytes;
        msgs[1].buffer = data + offset;
        msgs[1].length = chunk_size;
        I2C_TRANSFER(i2c_dev, msgs, 2);
        offset = offset + chunk_size;
        remaining_bytes = remaining_bytes - chunk_size;
    }

    return ret;
}

static int tfa9890_bulk_write(FAR struct i2c_dev_s *i2c_dev, uint32_t i2c_addr,
                uint8_t reg, const void *data, size_t len)
{
    int offset = 0;
    int ret = 0;

    /* first byte is mem address */
    int remaining_bytes = len - 1;
    int chunk_size = TFA9890_MAX_I2C_SIZE;

    struct i2c_msg_s msg[] =
    {
        {
            .addr = i2c_addr,
            .flags = 0,
            .buffer = chunk_buf,
            .length = len,
        },
    };
    sem_wait(&bulk_write_lock);
    chunk_buf[0] = reg & 0xff;
    while ((remaining_bytes > 0))
    {
        if (remaining_bytes <= chunk_size)
           chunk_size = remaining_bytes;

        memcpy(chunk_buf + 1, data + 1 + offset, chunk_size);
        msg[0].length = chunk_size +1;
        ret = I2C_TRANSFER(i2c_dev, msg, 1);

        if (ret)
        {
            lldbg("I2C_TRANSFER error = %d\n", ret);
            goto out;
        }

        offset += chunk_size;
        remaining_bytes -= chunk_size;

    }
out:
    sem_post(&bulk_write_lock);
    return ret;
}

/* RPC Protocol to Access DSP Memory is implemented in tfa9890_dsp_transfer func
 *- The host writes into a certain DSP memory location the data for the RPC
 *- Module and Id of the function
 *- Parameters for the function (in case of SetParam), maximum 145 parameters
 *- The host triggers the DSP that there is valid RPC data
 *  - The DSP executes the RPC and stores the result
 *      -An error code
 *      -Return parameters (in case of GetParam)
 *  - The host reads the result
 */

static int tfa9890_dsp_transfer(FAR struct tfa9890_dev_s *priv, int module_id,
                int param_id, const uint8_t *bytes,
                int len, int type, uint8_t *read)
{
    uint8_t buffer[8];
    /* DSP mem access control */
    uint16_t cf_ctrl = TFA9890_CF_CTL_MEM_REQ;
    /* memory address to be accessed (0 : Status, 1 : ID, 2 : parameters) */
    uint16_t cf_mad = TFA9890_CF_MAD_ID;
    int err;
    int rpc_status;
    int cf_status;

    /* first the data for CF_CONTROLS */
    buffer[0] = TFA9890_CF_CONTROLS;
    buffer[1] = ((cf_ctrl >> 8) & 0xFF);
    buffer[2] = (cf_ctrl & 0xFF);
    /* write the contents of CF_MAD which is the subaddress
     * following CF_CONTROLS.
     */
    buffer[3] = ((cf_mad >> 8) & 0xFF);
    buffer[4] = (cf_mad & 0xFF);
    /* write the module and RPC id into CF_MEM, which follows CF_MAD */
    buffer[5] = 0;
    buffer[6] = module_id + 128;
    buffer[7] = param_id;

    err = tfa9890_bulk_write(priv->i2c, priv->i2c_addr, TFA9890_CF_CONTROLS,
                         buffer, sizeof(buffer));
    if (err < 0)
    {
        lldbg("Failed to Write DSP controls req %d\n", err);
        return err;
    }
    /* check transfer type */
    if (type == TFA9890_DSP_WRITE)
    {
        /* write data to DSP memory */
        err = tfa9890_bulk_write(priv->i2c, priv->i2c_addr, TFA9890_CF_MEM,
                         bytes, len);
        if (err < 0)
          return err;
    }

    /* wake up the DSP to process the data */
    cf_ctrl |= TFA9890_CF_ACK_REQ | TFA9890_CF_INT_REQ;
    err = tfa9890_reg_write(priv, TFA9890_CF_CONTROLS, cf_ctrl);
    if (err)
    {
        lldbg("failed to wake up DSP to process the data err = %d\n", err);
        return err;
    }
    /* wait for ~1 msec (max per spec) and check for DSP status */
    usleep(1000);
    cf_status = tfa9890_reg_read(priv, TFA9890_CF_STATUS);
    if ((cf_status & TFA9890_STATUS_CF) == 0)
    {
        lldbg("Failed to ack DSP CTL req 0x%04x\n", cf_status);
        return -EIO;
    }

    /* read the RPC Status */
    cf_ctrl = TFA9890_CF_CTL_MEM_REQ;
    buffer[0] = TFA9890_CF_CONTROLS;
    buffer[1] = (unsigned char)((cf_ctrl >> 8) & 0xFF);
    buffer[2] = (unsigned char)(cf_ctrl & 0xFF);
    cf_mad = TFA9890_CF_MAD_STATUS;
    buffer[3] = (unsigned char)((cf_mad >> 8) & 0xFF);
    buffer[4] = (unsigned char)(cf_mad & 0xFF);
    err = tfa9890_bulk_write(priv->i2c, priv->i2c_addr, TFA9890_CF_CONTROLS,
                        buffer, 5);
    if (err < 0)
    {
        lldbg("Failed to Write NXP DSP CTL reg for rpc check %d\n", err);
        return err;
    }

    rpc_status = tfa9890_reg_read(priv, TFA9890_CF_MEM);
    if (rpc_status != TFA9890_STATUS_OK)
    {
        lldbg("RPC status check failed %x\n", rpc_status);
        return -EIO;
    }
    if (type == TFA9890_DSP_READ)
    {
        cf_mad = TFA9890_CF_MAD_PARAM;
        tfa9890_reg_write(priv, TFA9890_CF_MAD, cf_mad);
        tfa9890_bulk_read(priv->i2c, priv->i2c_addr, TFA9890_CF_MEM, read, len);
    }

    return 0;
}

static int tfa9887_load_dsp_patch(FAR struct tfa9890_dev_s *priv,
                                            const uint8_t *fw, int fw_len)
{
    int index = 0;
    int length;
    int size = 0;
    const uint8_t *fw_data;
    int err = -EINVAL;

    length = fw_len - TFA9890_PATCH_HEADER;
    fw_data = fw + TFA9890_PATCH_HEADER;

    /* process the firmware */
    while (index < length)
    {
        /* extract length from first two bytes*/
        size = *(fw_data + index) + (*(fw_data + index + 1)) * 256;
        index += 2;
        if ((index + size) > length)
        {
            /* outside the buffer, error in the input data */
            lldbg("invalid length\n");
            goto out;
        }
        if ((size) > TFA9890_MAX_I2C_SIZE)
        {
            /* too big */
            lldbg("invalid dsp patch\n");
            goto out;
        }
        err = tfa9890_bulk_write(priv->i2c, priv->i2c_addr, *(fw_data + index),
                     (fw_data + index), size);
        if (err < 0)
        {
            lldbg("writing dsp patch failed\n");
            goto out;
        }
        index += size;
    }
    err = 0;

out:
    return err;
}

static int tfa9890_get_preset_index(const int presets[],
                           uint32_t steps, int32_t sys_vol_db)
{
    int index = steps - 1;
    int i;

    /* find preset index */
    for (i = 0; i < steps - 1; i++) {
        if ((sys_vol_db <= presets[i]) && (sys_vol_db >= presets[i+1])) {
            index = i;
            break;
        }
    }
    return index;
}

static int tfa9890_update_preset(FAR struct tfa9890_dev_s *priv)
{
    int32_t sys_vol_db;
    int32_t use_case;
    int preset_index;
    const uint8_t *pst;
    int ret;

    ret = tfa9890_wait_pll_sync(priv);
    if (ret) {
        goto err;
    }
    sys_vol_db = priv->sys_vol_db;
    use_case = priv->current_eq;

    switch (use_case) {
    case TFA9890_MUSIC:
        pst = tfa9890_music_table_preset;
        preset_index = tfa9890_get_preset_index(tfa9890_presets_music,
                                                TFA9890_MUSIC_VOL_STEP, sys_vol_db);
        break;
    case TFA9890_VOICE:
        pst = tfa9890_voice_table_preset;
        preset_index = tfa9890_get_preset_index(tfa9890_presets_voice,
                                                TFA9890_VOICE_VOL_STEP, sys_vol_db);
        break;
    case TFA9890_RINGTONE:
        pst = tfa9890_ringtone_table_preset;
        preset_index = tfa9890_get_preset_index(tfa9890_presets_ringtone,
                                                TFA9890_RINGTONE_VOL_STEP, sys_vol_db);
        break;
    default:
        preset_index = -1;
        break;
    }
    if (preset_index < 0) {
        ret = -EINVAL;
        goto err;
    }
    ret = tfa9890_dsp_transfer(priv, TFA9890_DSP_MOD_SPEAKERBOOST,
                               TFA9890_PARAM_SET_PRESET, (pst + TFA9890_PST_FW_SIZE * preset_index),
                               TFA9890_PST_FW_SIZE,
                               TFA9890_DSP_WRITE, 0);
    priv->preset_update_pending = 0;
    return 0;
err:
    priv->preset_update_pending = 1;
    return ret;
}

void tfa9890_powerup(FAR struct tfa9890_dev_s *priv, int on)
{
    int tries = 0;

    if (on)
    {
        /* power on */
        tfa9890_modify(priv, TFA9890_REG_SYSTEM_CTRL_1, 1, 0, TFA9890_PWDN_OFFSET);
        /* amp on */
        tfa9890_modify(priv, TFA9890_REG_SYSTEM_CTRL_1, 1, 1, TFA9890_AMP_OFFSET);
    }
    else
    {
        /* amp off */
        tfa9890_modify(priv, TFA9890_REG_SYSTEM_CTRL_1, 1, 0, TFA9890_AMP_OFFSET);
        do
        {
            /* need to wait for amp to stop switching, to minimize
             * pop, before turning OFF IC.
             */
            if (tfa9890_reg_read(priv, TFA9890_REG_SYSTEM_STATUS)
                                   & TFA9890_STATUS_AMP_SWS)
                break;
            usleep(1000);
        } while (++tries < 20);

        tfa9890_modify(priv, TFA9890_REG_SYSTEM_CTRL_1, 1, 1, TFA9890_PWDN_OFFSET);
     }
}

int tfa9890_load_config(FAR struct tfa9890_dev_s *priv)
{
    int ret = -EIO;

    ret = tfa9887_load_dsp_patch(priv, tfa9890_n1c2_patch,
                     tfa9890_n1c2_patch_len);
    if (ret)
    {
        lldbg("Failed to load dsp patch!\n");
        goto out;
    }

    ret = tfa9890_dsp_transfer(priv, TFA9890_DSP_MOD_SPEAKERBOOST,
                   TFA9890_PARAM_SET_LSMODEL, tfa9890_speaker,
                   tfa9890_speaker_len, TFA9890_DSP_WRITE, 0);
    if (ret)
    {
        lldbg("Failed to load speaker!\n");
        goto out;
    }

    ret = tfa9890_dsp_transfer(priv, TFA9890_DSP_MOD_SPEAKERBOOST,
                   TFA9890_PARAM_SET_CONFIG, tfa9890_config,
                   tfa9890_config_len, TFA9890_DSP_WRITE, 0);
    if (ret)
    {
         lldbg("Failed to load config!\n");
         goto out;
    }

    ret = tfa9890_dsp_transfer(priv, TFA9890_DSP_MOD_BIQUADFILTERBANK,
                   0, tfa9890_eq, tfa9890_eq_len,
                   TFA9890_DSP_WRITE, 0);
    if (ret)
    {
         lldbg("Failed to load eq!\n");
         goto out;
    }

    ret = tfa9890_update_preset(priv);
    if (ret)
    {
        lldbg("Failed to load preset!\n");
        goto out;
    }

    /* set all dsp config loaded */
    tfa9890_modify(priv, TFA9890_REG_SYSTEM_CTRL_1, 1, 1, TFA9890_SBSL_OFFSET);


    lldbg("dsp loaded successfully\n");

    ret = 0;

out:
    return ret;
}

static int tfa9890_read_spkr_imp(struct tfa9890_dev_s *priv)
{
    int imp;
    uint8_t buf[3] = {0, 0, 0};
    int ret;

    ret = tfa9890_dsp_transfer(priv,
            TFA9890_DSP_MOD_SPEAKERBOOST,
            TFA9890_PARAM_GET_RE0, 0, 3,
            TFA9890_DSP_READ, buf);
    if (ret == 0)
    {
        imp = (buf[0] << 16 | buf[1] << 8 | buf[2]);
        imp = imp/(1 << (23 - TFA9890_SPKR_IMP_EXP));
    } else
        imp = 0;

    return imp;
}

static void tfa9890_calibaration(struct tfa9890_dev_s *priv)
{
    uint16_t val;

    /* Ensure no audio playback while calibarating, mute but leave
     * amp enabled
     */
    val = tfa9890_reg_read(priv, TFA9890_REG_VOLUME_CTRL);
    val = val | (TFA9890_STATUS_MUTE);
    tfa9890_reg_write(priv, TFA9890_REG_VOLUME_CTRL, val);

    /* unlock write access MTP memory, no need to relock
     * it will lock after copy from MTP memory is done.
     */
    tfa9890_reg_write(priv, TFA9890_REG_MTP_KEY, TFA9890_MTK_KEY);

    val = tfa9890_reg_read(priv, TFA9890_MTP_REG);
    /* set MTPOTC = 1 & MTPEX = 0 */
    val = val | TFA9890_MTPOTC;
    val = val & (~(TFA9890_STATUS_MTPEX));
    tfa9890_reg_write(priv, TFA9890_MTP_REG, val);

    /* set CIMTB to initiate copy of calib values */
    val = tfa9890_reg_read(priv, TFA9890_REG_I2C_TO_MTP);
    val = val | TFA9890_STATUS_CIMTP;
    tfa9890_reg_write(priv, TFA9890_REG_I2C_TO_MTP, val);
}

static int tfa9890_wait_pll_sync(struct tfa9890_dev_s *priv)
{
    int ret = 0;
    int tries = 0;
    uint16_t val;

    /* check if DSP pll is synced, should not take longer than 10msec */
    do
    {
        val = tfa9890_reg_read(priv, TFA9890_REG_SYSTEM_STATUS);
        if ((val & TFA9890_STATUS_UP_MASK) == TFA9890_STATUS_UP)
            break;
         usleep(1000);
    } while ((++tries < 10));

    if (tries >= 10)
    {
        lldbg("0X%x:DSP pll sync failed!\n", priv->i2c_addr);
        ret = -EIO;
    }

    return ret;
}

int tfa9890_start(FAR struct tfa9890_lowerhalf_s *dev)
{
    int ret = 0;
    uint16_t val;

    struct tfa9890_dev_s *priv= dev->priv;

    lldbg("activating 0X%x\n", priv->i2c_addr);
    tfa9890_powerup(priv, 1);
    tfa9890_wait_pll_sync(priv);
    while (sem_wait(&priv->preset_lock) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }
    if (!priv->is_dsp_cfg_done)
    {
       ret = tfa9890_load_config(priv);
       if (ret == 0)
       {
           priv->is_dsp_cfg_done = 1;
           val =  tfa9890_reg_read(priv, TFA9890_MTP_REG);
            /* check if calibaration completed, Calibaration is one time event.
            * Will be done only once when device boots up for the first time.Once
            * calibarated info is stored in non-volatile memory of the device.
            * It will be part of the factory test to validate spkr imp.
            * The MTPEX will be set to 1 always after calibration, on subsequent
            * power down/up as well.
            */
            if (!(val & TFA9890_STATUS_MTPEX))
            {
                lldbg("Calibration not completed initiating seq\n");
                tfa9890_calibaration(priv);
                val = tfa9890_reg_read(priv, TFA9890_REG_VOLUME_CTRL);
                val = val & ~(TFA9890_STATUS_MUTE);
                tfa9890_reg_write(priv, TFA9890_REG_VOLUME_CTRL, val);
            }
        }
    } else if (priv->preset_update_pending) {
            tfa9890_update_preset(priv);
    }

    /* read speaker impedence*/
    priv->speaker_imp = tfa9890_read_spkr_imp(priv);
    lldbg("Calibration imp %d\n", priv->speaker_imp);
#ifdef DEBUG
    tfa9890_dump_regs(priv);
#endif
    sem_post(&priv->preset_lock);

    return ret;
}

static int tfa9890_stop(FAR struct tfa9890_lowerhalf_s *dev)
{
    struct tfa9890_dev_s *priv = dev->priv;

    lldbg("deactivating %x\n", priv->i2c_addr);

    tfa9890_powerup(priv, 0);

    return 0;
}

static int tfa9890_configure(FAR struct tfa9890_lowerhalf_s *dev,
                            FAR const struct tfa9890_caps_s *caps)
{
    struct tfa9890_dev_s *priv = dev->priv;

    if(!priv || !caps)
        return -EINVAL;

    while (sem_wait(&priv->preset_lock) != OK)
    {
        if (errno == EINVAL)
        {
            return -EINVAL;
        }
    }
    switch (caps->type)
    {
        case TFA9890_VOLUME:
             /* vol step range 0-255
              * 0 -0db attenuation, 1 -0.5 attenuation
              * need to invert the volume value here as
              * greybus protocol treats 0 as mute and FF
              * as max volume/no attenuation.
              */
              priv->vol_step = ~(caps->param & 0xFF);
              tfa9890_modify(priv, TFA9890_REG_VOLUME_CTRL,
                                0xff, priv->vol_step,
                                TFA9890_VOLUME_CTRL_OFFSET);
              break;
        case TFA9890_USECASE:
             if (priv->current_eq == caps->param) {
                 break;
             }
             priv->current_eq = caps->param;
             if (priv->is_dsp_cfg_done) {
                 tfa9890_update_preset(priv);
             }
             break;
        case TFA9890_SYS_VOL:
             if (priv->sys_vol_db == (int) caps->param) {
                 break;
             }
             priv->sys_vol_db = (int) caps->param;
             if (priv->is_dsp_cfg_done) {
                 tfa9890_update_preset(priv);
             }
            break;
         default:
             return -EINVAL;;
     }

    sem_post(&priv->preset_lock);
    return 0;
}

static void tfa9890_init_registers(FAR struct tfa9890_dev_s *priv)
{
    int i;

    /* set up initial register values*/
    for (i = 0; i < ARRAY_SIZE(tfa9890_registers_init_list); i++)
    {
         tfa9890_reg_write(priv, tfa9890_registers_init_list[i].reg,
                     tfa9890_registers_init_list[i].val);
    }
}

#if defined (CONFIG_MODS_AUDIO_TFA9890_STEREO)
static int tfa9890_driver_stereo_setup(FAR struct tfa9890_dev_s *priv, int type)
{
    int i;

     /* set up right/left channel and Gain sharing channels
      * for stereo config.
      */
    if (type == TFA9890_LEFT)
    {

        for (i = 0; i < ARRAY_SIZE(tfa9890_registers_left_init); i++)
        {
             tfa9890_reg_write(priv, tfa9890_registers_left_init[i].reg,
                     tfa9890_registers_left_init[i].val);
        }
        priv->type = TFA9890_LEFT;
    }
    else if (type == TFA9890_RIGHT)
    {
        for (i = 0; i < ARRAY_SIZE(tfa9890_registers_right_init); i++)
        {
             tfa9890_reg_write(priv, tfa9890_registers_right_init[i].reg,
                     tfa9890_registers_right_init[i].val);
        }
        priv->type = TFA9890_RIGHT;
    }
    else if (type == TFA9890_MONO)
         /* default set up is mono, nothing to do */
         priv->type = TFA9890_MONO;
    else
         return -EINVAL;

    return 0;
}
#endif

FAR struct tfa9890_lowerhalf_s *tfa9890_driver_init(FAR struct i2c_dev_s *i2c,
                                                   uint32_t i2c_addr,
                                                   uint32_t frequency,
                                                   int type)
{
    int ret;
    FAR struct tfa9890_dev_s *priv;

    lldbg("0x%x\n", i2c_addr);

    priv = (FAR struct tfa9890_dev_s  *)kmm_zalloc(sizeof(FAR struct tfa9890_dev_s));
    if (!priv)
        return NULL;

    I2C_SETFREQUENCY(i2c, frequency);

    priv->i2c = i2c;
    priv->i2c_addr = i2c_addr;

    priv->dev.ops = &g_audioops;
    priv->dev.priv = (void*)priv;

    /* verify the lowest 7 bits of the ID are 80h */
    ret = tfa9890_reg_read(priv, TFA9890_REG_ID);
    if (ret != 0x80)
    {
        lldbg("0X%x: failed rev check\n", i2c_addr);
        goto err;
    }
    tfa9890_modify(priv, TFA9890_REG_SYSTEM_CTRL_1, 1, 1, TFA9890_RESET_OFFSET);

    tfa9890_init_registers(priv);
    sem_init(&priv->preset_lock, 0, 1);
#if defined (CONFIG_MODS_AUDIO_TFA9890_STEREO)
    tfa9890_driver_stereo_setup(priv, type);
#endif

#ifdef DEBUG
       tfa9890_dump_regs(priv);
#endif
    return &priv->dev;

err:
    kmm_free(priv);
    return NULL;
}

void tfa9890_deinit(FAR struct tfa9890_lowerhalf_s *dev)
{
    struct tfa9890_dev_s *priv = dev->priv;

    if(priv)
        kmm_free(priv);
}