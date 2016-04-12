/*
 * Copyright (c) 2016 Motorola, LLC.
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
#include <debug.h>
#include <stdint.h>

#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/util.h>
#include <nuttx/wqueue.h>

#include <nuttx/device_ext_power.h>

#define FUSB302_I2C_ADDR 0x22

#define FUSB302_DEVICE_ID_REG                      0x01
# define FUSB302_DEVICE_ID_VERSION_ID_MASK     (0x0f << 4)
# define FUSB302_DEVICE_ID_REVISION_ID_MASK    (0x0f << 0)

#define FUSB302_SWITCHES0_REG                      0x02
# define FUSB302_SWITCHES0_PU_EN2_MASK         (0x01 << 7)
# define FUSB302_SWITCHES0_PU_EN1_MASK         (0x01 << 6)
# define FUSB302_SWITCHES0_VCONN_CC2_MASK      (0x01 << 5)
# define FUSB302_SWITCHES0_VCONN_CC1_MASK      (0x01 << 4)
# define FUSB302_SWITCHES0_MEAS_CC2_MASK       (0x01 << 3)
# define FUSB302_SWITCHES0_MEAS_CC1_MASK       (0x01 << 2)
# define FUSB302_SWITCHES0_PDWN2_MASK          (0x01 << 1)
# define FUSB302_SWITCHES0_PDWN1_MASK          (0x01 << 0)
# define FUSB302_SWITCHES0_ZERO_MASK           (0x00 << 0)

#define FUSB302_SWITCHES1_REG                      0x03
# define FUSB302_SWITCHES1_POWERROLE_MASK      (0x01 << 7)
# define FUSB302_SWITCHES1_SPECREV_MASK        (0x03 << 5)
# define FUSB302_SWITCHES1_DATAROLE_MASK       (0x01 << 4)
# define FUSB302_SWITCHES1_AUTO_CRC_MASK       (0x01 << 2)
# define FUSB302_SWITCHES1_TXCC2_MASK          (0x01 << 1)
# define FUSB302_SWITCHES1_TXCC1_MASK          (0x01 << 0)

#define FUSB302_MEASURE_REG                        0x04
# define FUSB302_MEASURE_MEAS_VBUS_MASK        (0x01 << 7)
# define FUSB302_MEASURE_MDAC_MASK             (0x3f << 0)
#  define FUSB302_MEASURE_MDAC_0P042_0P420     (0x00 << 0)
#  define FUSB302_MEASURE_MDAC_0P084_0P840     (0x01 << 0)
#  define FUSB302_MEASURE_MDAC_2P058_20P58     (0x30 << 0)
#  define FUSB302_MEASURE_MDAC_XPXXX_XPXXX     (0x31 << 0)
#  define FUSB302_MEASURE_MDAC_2P184_21P84     (0x33 << 0)
#  define FUSB302_MEASURE_MDAC_2P646_26P46     (0x3e << 0)
#  define FUSB302_MEASURE_MDAC_2P688_26P88     (0x3f << 0)

#define FUSB302_SLICE_REG                          0x05
# define FUSB302_SLICE_SDAC_HYS_MASK           (0x03 << 6)
# define FUSB302_SLICE_SDAC_MASK               (0x3f << 0)
#  define FUSB302_SLICE_SDAC_HYS_255MV         (0x03 << 6)
#  define FUSB302_SLICE_SDAC_HYS_170MV         (0x02 << 6)
#  define FUSB302_SLICE_SDAC_HYS_085MV         (0x01 << 6)
#  define FUSB302_SLICE_SDAC_HYS_NONE          (0x00 << 6)

#define FUSB302_CONTROL0_REG   0x06
# define FUSB302_CONTROL0_TX_FLUSH_MASK        (0x01 << 6)
# define FUSB302_CONTROL0_INT_MASK_MASK        (0x01 << 5)
# define FUSB302_CONTROL0_HOST_CUR_MASK        (0x03 << 2)
# define FUSB302_CONTROL0_AUTO_PRE_MASK        (0x01 << 1)
# define FUSB302_CONTROL0_TX_START_MASK        (0x01 << 0)
#  define FUSB302_CONTROL0_HOST_CUR_NO         (0x00 << 2)
#  define FUSB302_CONTROL0_HOST_CUR_USB        (0x01 << 2)
#  define FUSB302_CONTROL0_HOST_CUR_1P5A       (0x02 << 2)
#  define FUSB302_CONTROL0_HOST_CUR_3P0A       (0x03 << 2)

#define FUSB302_CONTROL1_REG   0x07
# define FUSB302_CONTROL1_ENSOP2DB_MASK        (0x01 << 6)
# define FUSB302_CONTROL1_ENSOP1DB_MASK        (0x01 << 5)
# define FUSB302_CONTROL1_BIST_MODE2_MASK      (0x01 << 4)
# define FUSB302_CONTROL1_RESERVED_MASK        (0x01 << 3)
# define FUSB302_CONTROL1_RX_FLUSH_MASK        (0x01 << 2)
# define FUSB302_CONTROL1_ENSOP2_MASK          (0x01 << 1)
# define FUSB302_CONTROL1_ENSOP1_MASK          (0x01 << 0)

#define FUSB302_CONTROL2_REG   0x08
# define FUSB302_CONTROL2_TOG_SAVE_PWR_MASK    (0x03 << 6)
# define FUSB302_CONTROL2_TOG_RD_ONLY_MASK     (0x01 << 5)
# define FUSB302_CONTROL2_RESERVED_MASK        (0x01 << 4)
# define FUSB302_CONTROL2_WAKE_EN_MASK         (0x01 << 3)
# define FUSB302_CONTROL2_MODE_MASK            (0x03 << 2)
# define FUSB302_CONTROL2_TOGGLE_MASK          (0x01 << 0)
#  define FUSB302_CONTROL2_MODE_SRC_POLLING    (0x03 << 2)
#  define FUSB302_CONTROL2_MODE_SNK_POLLING    (0x02 << 2)
#  define FUSB302_CONTROL2_MODE_DRP_POLLING    (0x01 << 2)

#define FUSB302_CONTROL3_REG   0x09
# define FUSB302_CONTROL3_RESERVED1_MASK       (0x01 << 7)
# define FUSB302_CONTROL3_SEND_HARD_RESET_MASK (0x01 << 6)
# define FUSB302_CONTROL3_RESERVED2_MASK       (0x01 << 5)
# define FUSB302_CONTROL3_AUTO_HARD_RESET_MASK (0x01 << 4)
# define FUSB302_CONTROL3_AUTO_SOFT_RESET_MASK (0x01 << 3)
# define FUSB302_CONTROL3_N_RETRIES_MASK       (0x03 << 1)
# define FUSB302_CONTROL3_AUTO_RETRY_MASK      (0x01 << 0)
#  define FUSB302_CONTROL3_N_RETRIES_THREE     (0x03 << 1)
#  define FUSB302_CONTROL3_N_RETRIES_TWO       (0x02 << 1)
#  define FUSB302_CONTROL3_N_RETRIES_ONE       (0x01 << 1)
#  define FUSB302_CONTROL3_N_RETRIES_NONE      (0x00 << 1)

#define FUSB302_MASK_REG       0x0a
# define FUSB302_MASK_M_VBUSOK_MASK            (0x01 << 7)
# define FUSB302_MASK_M_ACTIVITY_MASK          (0x01 << 6)
# define FUSB302_MASK_M_COMP_CHNG_MASK         (0x01 << 5)
# define FUSB302_MASK_M_CRC_CHK_MASK           (0x01 << 4)
# define FUSB302_MASK_M_ALERT_MASK             (0x01 << 3)
# define FUSB302_MASK_M_WAKE_MASK              (0x01 << 2)
# define FUSB302_MASK_M_COLLISION_MASK         (0x01 << 1)
# define FUSB302_MASK_M_BC_LVL_MASK            (0x01 << 0)

#define FUSB302_POWER_REG      0x0b
# define FUSB302_POWER_RESERVED_MASK           (0x0f << 4)
# define FUSB302_POWER_PWR_MASK                (0x0f << 0)
# define FUSB302_POWER_PWR_INIT_MASK           (0x07 << 0)

#define FUSB302_RESET_REG      0x0c
# define FUSB302_RESET_RESERVED_MASK           (0x3f << 4)
# define FUSB302_RESET_PD_RESET_MASK           (0x01 << 1)
# define FUSB302_RESET_SW_RES_MASK             (0x01 << 0)

#define FUSB302_OCPREG_REG     0x0d
# define FUSB302_OCPREG_RESERVED_MASK          (0x0f << 4)
# define FUSB302_OCPREG_OCP_RANGE_MASK         (0x01 << 3)
# define FUSB302_OCPREG_OCP_CUR_MASK           (0x07 << 0)
#  define FUSB302_OCPREG_OCP_RANGE_800MA       (0x01 << 3)
#  define FUSB302_OCPREG_OCP_CUR_MAX           (0x07 << 0)
#  define FUSB302_OCPREG_OCP_CUR_7X            (0x06 << 0)
#  define FUSB302_OCPREG_OCP_CUR_6X            (0x05 << 0)
#  define FUSB302_OCPREG_OCP_CUR_5X            (0x04 << 0)
#  define FUSB302_OCPREG_OCP_CUR_4X            (0x03 << 0)
#  define FUSB302_OCPREG_OCP_CUR_3X            (0x02 << 0)
#  define FUSB302_OCPREG_OCP_CUR_2X            (0x01 << 0)
#  define FUSB302_OCPREG_OCP_CUR_1X            (0x00 << 0)

#define FUSB302_MASKA_REG      0x0e
# define FUSB302_MASKA_M_OCP_TEMP_MASK         (0x01 << 7)
# define FUSB302_MASKA_M_TOGDONE_MASK          (0x01 << 6)
# define FUSB302_MASKA_M_SOFTFAIL_MASK         (0x01 << 5)
# define FUSB302_MASKA_M_RETRY_FAIL_MASK       (0x01 << 4)
# define FUSB302_MASKA_M_HARDSENT_MASK         (0x01 << 3)
# define FUSB302_MASKA_M_TXSENT_MASK           (0x01 << 2)
# define FUSB302_MASKA_M_SOFTRST_MASK          (0x01 << 1)
# define FUSB302_MASKA_M_HARDRST_MASK          (0x01 << 0)

#define FUSB302_MASKB_REG      0x0f
# define FUSB302_MASKB_RESERVED_MASK           (0x7f << 1)
# define FUSB302_MASKB_M_GCRCSENT_MASK         (0x01 << 0)

#define FUSB302_CONTROL4_REG   0x10
# define FUSB302_CONTROL4_RESERVED_MASK        (0x7f << 1)
# define FUSB302_CONTROL4_TOG_USRC_EXIT_MASK   (0x01 << 0)

#define FUSB302_STATUS0A_REG   0x3c
# define FUSB302_STATUS0A_RESERVED_MASK        (0x03 << 1)
# define FUSB302_STATUS0A_SOFTFAIL_MASK        (0x01 << 5)
# define FUSB302_STATUS0A_RETRYFAIL_MASK       (0x01 << 4)
# define FUSB302_STATUS0A_POWER_MASK           (0x03 << 2)
# define FUSB302_STATUS0A_SOFTRST_MASK         (0x01 << 1)
# define FUSB302_STATUS0A_HARDRST_MASK         (0x01 << 0)

#define FUSB302_STATUS1A_REG   0x3d
# define FUSB302_STATUS1A_RESERVED_MASK        (0x03 << 1)
# define FUSB302_STATUS1A_TOGSS_SHIFT                  (3)
# define FUSB302_STATUS1A_TOGSS_MASK           (0x07 << FUSB302_STATUS1A_TOGSS_SHIFT)
# define FUSB302_STATUS1A_RXSOP2DB_MASK        (0x01 << 2)
# define FUSB302_STATUS1A_RXSOP1DB_MASK        (0x01 << 1)
# define FUSB302_STATUS1A_RXSOP_MASK           (0x01 << 0)

# define FUSB302_STATUS1A_TOGSS_TOGGLE_RUNNING      (0x00)
# define FUSB302_STATUS1A_TOGSS_SRC_ON_CC1          (0x01)
# define FUSB302_STATUS1A_TOGSS_SRC_ON_CC2          (0x02)
# define FUSB302_STATUS1A_TOGSS_SNK_ON_CC1          (0x05)
# define FUSB302_STATUS1A_TOGSS_SNK_ON_CC2          (0x06)
# define FUSB302_STATUS1A_TOGSS_AUTO_ACCESSORY      (0x07)


#define FUSB302_INTERRUPTA_REG 0x3e
# define FUSB302_INTERRUPTA_I_OCP_TEMP_MASK    (0x01 << 7)
# define FUSB302_INTERRUPTA_I_TOGDONE_MASK     (0x01 << 6)
# define FUSB302_INTERRUPTA_I_SOFTFAIL_MASK    (0x01 << 5)
# define FUSB302_INTERRUPTA_I_RETRYFAIL_MASK   (0x01 << 4)
# define FUSB302_INTERRUPTA_I_HARDSENT_MASK    (0x01 << 3)
# define FUSB302_INTERRUPTA_I_TXSENT_MASK      (0x01 << 2)
# define FUSB302_INTERRUPTA_I_SOFTRST_MASK     (0x01 << 1)
# define FUSB302_INTERRUPTA_I_HARDRST_MASK     (0x01 << 0)

#define FUSB302_INTERRUPTB_REG 0x3f
# define FUSB302_INTERRUPTB_RESERVED_MASK      (0x7f << 1)
# define FUSB302_INTERRUPTB_I_GCRCSENT_MASK    (0x01 << 0)

#define FUSB302_STATUS0_REG                 0x40
# define FUSB302_STATUS0_VBUSOK_MASK           (0x01 << 7)
# define FUSB302_STATUS0_ACTIVITY_MASK         (0x01 << 6)
# define FUSB302_STATUS0_COMP_MASK             (0x01 << 5)
# define FUSB302_STATUS0_CRC_CHK_MASK          (0x01 << 4)
# define FUSB302_STATUS0_ALERT_CHK_MASK        (0x01 << 3)
# define FUSB302_STATUS0_WAKE_MASK             (0x01 << 2)
# define FUSB302_STATUS0_BC_LVL_MASK           (0x03 << 0)

#define FUSB302_STATUS1_REG    0x41
# define FUSB302_STATUS1_RXSOP2_MASK           (0x01 << 7)
# define FUSB302_STATUS1_RXSOP1_MASK           (0x01 << 6)
# define FUSB302_STATUS1_RX_EMPTY_MASK         (0x01 << 5)
# define FUSB302_STATUS1_RX_FULL_MASK          (0x01 << 4)
# define FUSB302_STATUS1_TX_EMPTY_MASK         (0x01 << 3)
# define FUSB302_STATUS1_TX_FULL_MASK          (0x01 << 2)
# define FUSB302_STATUS1_OVRTEMP_MASK          (0x01 << 1)
# define FUSB302_STATUS1_OCP_MASK              (0x01 << 0)

#define FUSB302_INTERRUPT_REG  0x42
# define FUSB302_INTERRUPT_I_VBUSOK_MASK       (0x01 << 7)
# define FUSB302_INTERRUPT_I_ACTIVITY_MASK     (0x01 << 6)
# define FUSB302_INTERRUPT_I_COMP_CHNG_MASK    (0x01 << 5)
# define FUSB302_INTERRUPT_I_CRC_CHK_MASK      (0x01 << 4)
# define FUSB302_INTERRUPT_I_ALERT_MASK        (0x01 << 3)
# define FUSB302_INTERRUPT_I_WAKE_MASK         (0x01 << 2)
# define FUSB302_INTERRUPT_I_COLLISION_MASK    (0x01 << 1)
# define FUSB302_INTERRUPT_I_BC_LVL_MASK       (0x01 << 0)

#define FUSB302_FIFOS_REG      0x43
# define FUSB302_FIFOS_RX_TX_TOKEN_MASK        (0xff << 0)


struct fusb302_regs_s {
    uint8_t device_id;
    uint8_t switches0;
    uint8_t switches1;
    uint8_t measure;
    uint8_t slice;
    uint8_t control0;
    uint8_t control1;
    uint8_t control2;
    uint8_t control3;
    uint8_t mask;
    uint8_t power;
    uint8_t reset;
    uint8_t ocpreg;
    uint8_t maska;
    uint8_t maskb;
    uint8_t control4;

    uint8_t status0a;
    uint8_t status1a;
    uint8_t interrupta;
    uint8_t interruptb;

    uint8_t status0;
    uint8_t status1;
    uint8_t interrupt;
    uint8_t fifos;
};

static struct fusb302_regs_s fusb302_regs;

struct fusb302_dev_s {
    FAR struct i2c_dev_s *i2c;
    int int_n;
    struct work_s work;
    sem_t fusb302_sem;
    ext_power_changed power_changed_cb;
    void *fusb302_available_arg;
};

static struct fusb302_dev_s *fusb302_info;

static int decode_cc_termination(void)
{
    int termination = 0;   /*  By default set it to nothing */

    if (fusb302_regs.status0 & FUSB302_STATUS0_VBUSOK_MASK) {   /* VBUS Valid  */
       switch (fusb302_regs.status0 & FUSB302_STATUS0_BC_LVL_MASK) {   /* Determine which level */
            case 0b00:   /* If BC_LVL is lowest... it's an vRa */
                 termination = 0;;
                 break;
            case 0b01:   /* If BC_LVL is 1, it's default */
                 termination = 500;
                 break;
            case 0b10:   /* If BC_LVL is 2, it's vRd1p5 */
                 termination = 1500;
                 break;
            case 0b11:   /* If BC_LVL is 2 and COMP is 0, it's vRd3p0 */
                 if ((fusb302_regs.status0 & FUSB302_STATUS0_COMP_MASK) == 0)
                     termination = 3000;
                 break;
        }
    }
    return termination; /* Return the termination type */
}

static inline int fusb302_reg_read(struct i2c_dev_s *i2c, uint8_t reg)
{
    uint8_t val;
    int ret;

    ret = I2C_WRITEREAD(i2c, &reg, sizeof(reg), &val, sizeof(val));
    return ret ? ret : val;
}

static inline int fusb302_reg_write(struct i2c_dev_s *i2c, uint8_t reg, uint8_t val)
{
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = val;

    return I2C_WRITE(i2c, buf, sizeof(buf));
}

static void fusb302_init(void)
{
    fusb302_reg_write(fusb302_info->i2c, FUSB302_RESET_REG, FUSB302_RESET_SW_RES_MASK);                /* [0x0C]:0x01 */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL2_REG, FUSB302_CONTROL2_TOGGLE_MASK);          /* [0x08]:0x01 */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_POWER_REG, FUSB302_POWER_PWR_INIT_MASK);              /* [0x0B]:0x07 */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL0_REG, FUSB302_CONTROL0_TX_START_MASK);        /* [0x06]:0x01 */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MEASURE_REG, FUSB302_MEASURE_MDAC_0P042_0P420);       /* [0x04]:0x00 */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES0_REG, FUSB302_SWITCHES0_ZERO_MASK);          /* [0x02]:0x00 */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASK_REG, FUSB302_MASK_M_VBUSOK_MASK|                 /* [0x0A]:0xFE */
                                         FUSB302_MASK_M_ACTIVITY_MASK|FUSB302_MASK_M_COMP_CHNG_MASK|
                                         FUSB302_MASK_M_CRC_CHK_MASK|FUSB302_MASK_M_ALERT_MASK|
                                         FUSB302_MASK_M_WAKE_MASK|FUSB302_MASK_M_COLLISION_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASKA_REG, FUSB302_MASKA_M_OCP_TEMP_MASK|             /* [0x0E]:0xBF */
                                         FUSB302_MASKA_M_SOFTFAIL_MASK|FUSB302_MASKA_M_RETRY_FAIL_MASK|
                                         FUSB302_MASKA_M_HARDSENT_MASK|FUSB302_MASKA_M_TXSENT_MASK|
                                         FUSB302_MASKA_M_SOFTRST_MASK|FUSB302_MASKA_M_HARDRST_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASKB_REG, FUSB302_MASKB_M_GCRCSENT_MASK);            /* [0x0F]:0x01 */
}

static void fusb302_read_all_regs(void)
{
    fusb302_regs.device_id  = fusb302_reg_read(fusb302_info->i2c, FUSB302_DEVICE_ID_REG);
    fusb302_regs.switches0  = fusb302_reg_read(fusb302_info->i2c, FUSB302_SWITCHES0_REG);
    fusb302_regs.switches1  = fusb302_reg_read(fusb302_info->i2c, FUSB302_SWITCHES1_REG);
    fusb302_regs.measure    = fusb302_reg_read(fusb302_info->i2c, FUSB302_MEASURE_REG);
    fusb302_regs.slice      = fusb302_reg_read(fusb302_info->i2c, FUSB302_SLICE_REG);
    fusb302_regs.control0   = fusb302_reg_read(fusb302_info->i2c, FUSB302_CONTROL0_REG);
    fusb302_regs.control1   = fusb302_reg_read(fusb302_info->i2c, FUSB302_CONTROL1_REG);
    fusb302_regs.control2   = fusb302_reg_read(fusb302_info->i2c, FUSB302_CONTROL2_REG);
    fusb302_regs.control3   = fusb302_reg_read(fusb302_info->i2c, FUSB302_CONTROL3_REG);
    fusb302_regs.mask       = fusb302_reg_read(fusb302_info->i2c, FUSB302_MASK_REG);
    fusb302_regs.power      = fusb302_reg_read(fusb302_info->i2c, FUSB302_POWER_REG);
    fusb302_regs.reset      = fusb302_reg_read(fusb302_info->i2c, FUSB302_RESET_REG);
    fusb302_regs.ocpreg     = fusb302_reg_read(fusb302_info->i2c, FUSB302_OCPREG_REG);
    fusb302_regs.maska      = fusb302_reg_read(fusb302_info->i2c, FUSB302_MASKA_REG);
    fusb302_regs.maskb      = fusb302_reg_read(fusb302_info->i2c, FUSB302_MASKB_REG);
    fusb302_regs.control4   = fusb302_reg_read(fusb302_info->i2c, FUSB302_CONTROL4_REG);

    fusb302_regs.status0a   = fusb302_reg_read(fusb302_info->i2c, FUSB302_STATUS0A_REG);
    fusb302_regs.status1a   = fusb302_reg_read(fusb302_info->i2c, FUSB302_STATUS1A_REG);
    fusb302_regs.interrupta = fusb302_reg_read(fusb302_info->i2c, FUSB302_INTERRUPTA_REG);
    fusb302_regs.interruptb = fusb302_reg_read(fusb302_info->i2c, FUSB302_INTERRUPTB_REG);

    fusb302_regs.status0    = fusb302_reg_read(fusb302_info->i2c, FUSB302_STATUS0_REG);
    fusb302_regs.status1    = fusb302_reg_read(fusb302_info->i2c, FUSB302_STATUS1_REG);
    fusb302_regs.interrupt  = fusb302_reg_read(fusb302_info->i2c, FUSB302_INTERRUPT_REG);
    fusb302_regs.fifos      = fusb302_reg_read(fusb302_info->i2c, FUSB302_FIFOS_REG);
};

static void fusb302_write_connected(uint8_t sw0, uint8_t sw1)
{
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES0_REG, sw0);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES1_REG, sw1);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MEASURE_REG, /* 0x31 */
            FUSB302_MEASURE_MDAC_XPXXX_XPXXX);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SLICE_REG, /* 0x60 */
            FUSB302_SLICE_SDAC_HYS_085MV |
            0x20); /* magic */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL0_REG, 0x00);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL1_REG, 0x00);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL2_REG, /* 0x25 */
            FUSB302_CONTROL2_MODE_SNK_POLLING |
            FUSB302_CONTROL2_TOGGLE_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL3_REG, /* 0x05 */
            FUSB302_CONTROL3_N_RETRIES_ONE |
            FUSB302_CONTROL3_AUTO_RETRY_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASK_REG, /* 0x44 */
            FUSB302_MASK_M_ACTIVITY_MASK |
            FUSB302_MASK_M_COMP_CHNG_MASK |
            FUSB302_MASK_M_WAKE_MASK |
            FUSB302_MASK_M_WAKE_MASK |
            FUSB302_MASK_M_COLLISION_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_POWER_REG, 0x07); /* magic */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_OCPREG_REG, /* 0x0f */
            FUSB302_OCPREG_OCP_RANGE_800MA |
            FUSB302_OCPREG_OCP_CUR_MAX);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASKA_REG, 0x00);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASKB_REG, 0x00);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL4_REG,  /* 0x01 */
            FUSB302_CONTROL4_TOG_USRC_EXIT_MASK);
}

static void fusb302_write_disconnected(void)
{
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES0_REG, /* 0x03 */
            FUSB302_SWITCHES0_MEAS_CC1_MASK |
            FUSB302_SWITCHES0_PDWN2_MASK |
            FUSB302_SWITCHES0_PDWN1_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES1_REG, 0x00);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MEASURE_REG, /* 0x31 */
            FUSB302_MEASURE_MDAC_XPXXX_XPXXX);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SLICE_REG, /* 0x60 */
            FUSB302_SLICE_SDAC_HYS_085MV |
            0x20); /* magic */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL0_REG, /* 0x04 */
            FUSB302_CONTROL0_HOST_CUR_USB |
            FUSB302_CONTROL0_AUTO_PRE_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL1_REG,  0x00);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL2_REG, /* 0x25 */
            FUSB302_CONTROL2_MODE_SNK_POLLING |
            FUSB302_CONTROL2_TOGGLE_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL3_REG, /* 0x06 */
            FUSB302_CONTROL3_N_RETRIES_THREE);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASK_REG,      /* 0xff */
            FUSB302_MASK_M_VBUSOK_MASK |
            FUSB302_MASK_M_ACTIVITY_MASK |
            FUSB302_MASK_M_COMP_CHNG_MASK |
            FUSB302_MASK_M_CRC_CHK_MASK |
            FUSB302_MASK_M_ALERT_MASK |
            FUSB302_MASK_M_WAKE_MASK |
            FUSB302_MASK_M_COLLISION_MASK |
            FUSB302_MASK_M_BC_LVL_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_POWER_REG,     0x07);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_OCPREG_REG,    /* 0x0f */
            FUSB302_OCPREG_OCP_RANGE_800MA |
            FUSB302_OCPREG_OCP_CUR_MAX);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASKA_REG,     /* 0xbf */
            FUSB302_MASKA_M_OCP_TEMP_MASK |
            FUSB302_MASKA_M_SOFTFAIL_MASK |
            FUSB302_MASKA_M_RETRY_FAIL_MASK |
            FUSB302_MASKA_M_HARDSENT_MASK |
            FUSB302_MASKA_M_TXSENT_MASK |
            FUSB302_MASKA_M_SOFTRST_MASK |
            FUSB302_MASKA_M_HARDRST_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASKB_REG,     0x00);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL4_REG,  /* 0x01 */
            FUSB302_CONTROL4_TOG_USRC_EXIT_MASK);
}

static bool fusb302_get_cc1(void)
{
    uint8_t togss = (fusb302_regs.status1a & FUSB302_STATUS1A_TOGSS_MASK) >> FUSB302_STATUS1A_TOGSS_SHIFT;

    switch (togss) {
    case FUSB302_STATUS1A_TOGSS_SNK_ON_CC1:
    case FUSB302_STATUS1A_TOGSS_SRC_ON_CC1:
    case FUSB302_STATUS1A_TOGSS_AUTO_ACCESSORY:
        return true;
    default:
        return false;
    }
}

static bool fusb302_get_cc2(void)
{
    uint8_t togss = (fusb302_regs.status1a & FUSB302_STATUS1A_TOGSS_MASK)
            >> FUSB302_STATUS1A_TOGSS_SHIFT;

    switch (togss) {
    case FUSB302_STATUS1A_TOGSS_SNK_ON_CC2:
    case FUSB302_STATUS1A_TOGSS_SRC_ON_CC2:
    case FUSB302_STATUS1A_TOGSS_AUTO_ACCESSORY:
        return true;
    default:
        return false;
    }
}

static void fusb302_toggle_measure(void)
{
    fusb302_read_all_regs();

    if (fusb302_regs.status0 & FUSB302_STATUS0_VBUSOK_MASK) {
        bool cc1 = fusb302_get_cc1();
        bool cc2 = fusb302_get_cc2();

        if (cc1) {
            fusb302_write_connected(0x07, 0x05);
        } else if (cc2) {
            fusb302_write_connected(0x0b, 0x06);
        } else {
            fusb302_write_connected(0x03, 0x00);
        }
    } else {
        if (fusb302_regs.interrupta & FUSB302_INTERRUPTA_I_TOGDONE_MASK) {
            /* Disable the toggle in order to clear */
            fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL2_REG, 0x22);
        } else {
            fusb302_reg_write(fusb302_info->i2c, FUSB302_RESET_REG,
                    FUSB302_RESET_SW_RES_MASK);
        }
        fusb302_write_disconnected();
    }
}

static void fusb302_worker(FAR void *arg)
{
    fusb302_toggle_measure();
    if (fusb302_info->power_changed_cb)
        fusb302_info->power_changed_cb(fusb302_info->fusb302_available_arg, decode_cc_termination());
}

static int fusb302_isr(int irq, void *context)
{
    if (fusb302_info && work_available(&fusb302_info->work))
        work_queue(LPWORK, &fusb302_info->work, fusb302_worker, NULL, 0);

    return OK;
}

static int fusb302_register_callback(struct device *dev, ext_power_changed cb, void *arg)
{
    struct fusb302_dev_s *info = device_get_private(dev);

    if (!info) {
        lldbg("invalid\n");
        return 0;
    }

    if (info->power_changed_cb) {
        dbg("already registered\n");
        return -EINVAL;
    }

    if (!cb) {
        dbg("invalid parameter\n");
        return -EINVAL;
    }
    info->fusb302_available_arg = arg;
    info->power_changed_cb = cb;
    /* first call to initialize client */
    cb(arg, 0);
    /* reset and initialize chipset */
    fusb302_init();
    return 0;
}

struct reg_value {
    uint8_t reg;
    uint8_t val;
};

static int fusb302_open(struct device *dev)
{
    return 0;
}

static void fusb302_close(struct device *dev)
{
}

static int fusb302_probe(struct device *dev)
{
    int ret;
    struct device_resource *res;
    struct fusb302_dev_s *info;

    if (!dev) {
        dbg("no device present\n");
        ret = -EINVAL;
        goto init_done;
    }

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_IRQ, "int_n");
    if (!res) {
        dbg("failed to allocate info\n");
        ret = -EINVAL;
        goto init_done;
    }

    info = kmm_zalloc(sizeof(*info));
    if (!info) {
        dbg("failed to allocate info\n");
        ret = -ENOMEM;
        goto init_done;
    }

    info->int_n = res->start;

    info->i2c = up_i2cinitialize(CONFIG_FUSB302_I2C_BUS);
    if (!info->i2c) {
        dbg("failed to init i2c\n");
        ret = -ENODEV;
        goto init_free_info;
    }

    ret = I2C_SETADDRESS(info->i2c, FUSB302_I2C_ADDR, 7);
    if (ret) {
        dbg("failed to set i2c address\n");
        goto init_free_info;
    }

    gpio_direction_in(info->int_n);


    I2C_SETFREQUENCY(info->i2c, CONFIG_FUSB302_I2C_BUS_SPEED);

    fusb302_info = info;

    ret = gpio_irqattach(info->int_n, fusb302_isr);
    if (ret) {
        dbg("failed to register for irq\n");
        goto init_free_info;
    }

    ret = set_gpio_triggering(info->int_n, IRQ_TYPE_EDGE_FALLING);
    if (ret) {
        dbg("failed to set irq edge\n");
        goto init_free_info;
    }

    dev->private = info;

    return 0;

init_free_info:
    fusb302_info = NULL;
    free(info);
init_done:
    return ret;
}

void fusb302_remove(struct device *dev)
{
    kmm_free(fusb302_info);
    fusb302_info = NULL;
}

static struct device_ext_power_type_ops fusb302_type_ops  = {
    .register_callback = fusb302_register_callback,
};

static struct device_driver_ops fusb302_driver_ops = {
    .probe = fusb302_probe,
    .remove = fusb302_remove,
    .open = fusb302_open,
    .close = fusb302_close,
    .type_ops = &fusb302_type_ops,
};

struct device_driver fusb302_driver = {
    .type = DEVICE_TYPE_EXT_POWER_HW,
    .name = "fusb302",
    .desc = "FUSB 302 USBC",
    .ops = &fusb302_driver_ops,
};
