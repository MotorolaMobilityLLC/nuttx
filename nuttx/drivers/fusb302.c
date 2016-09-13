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
#include <debug.h>
#include <semaphore.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/pm.h>
#include <nuttx/util.h>
#include <nuttx/wqueue.h>

#include <nuttx/device_ext_power.h>
#include <nuttx/device_usb_ext.h>
#include <nuttx/fusb302.h>
#include <nuttx/util.h>

#define PM_USBC_ACTIVITY 10
#define USB_DEBOUNCE_MS 150

/* TODO: Add support for higher voltages */
#define OUTPUT_VOLTAGE  (5000)  /* mV */

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
# define FUSB302_MEASURE_MEAS_VBUS_MASK        (0x01 << 6)
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
# define FUSB302_CONTROL2_MODE_MASK            (0x03 << 1)
# define FUSB302_CONTROL2_TOGGLE_MASK          (0x01 << 0)
#  define FUSB302_CONTROL2_MODE_SRC_POLLING    (0x03 << 1)
#  define FUSB302_CONTROL2_MODE_SNK_POLLING    (0x02 << 1)
#  define FUSB302_CONTROL2_MODE_DRP_POLLING    (0x01 << 1)

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
    uint8_t status0a;
    uint8_t status1a;
    uint8_t interrupta;
    uint8_t interruptb;

    uint8_t status0;
    uint8_t status1;
    uint8_t interrupt;
};

static struct fusb302_regs_s fusb302_regs;

struct fusb302_usb_ext_s {
    struct device *dev;
    usb_ext_event_callback usb_ext_cb;
    int current;
    uint8_t path;
    uint8_t type;
};

struct fusb302_power_s {
    struct device *dev;
    device_ext_power_notification power_changed_cb;
    void *arg;
    int current;
};

enum usb_attach_state_e {
    USB_STATE_UNATTACHED,  /* this combines Unattached.SNK and Unattached.SRC */
    USB_STATE_ATTACHED_WAIT_SNK,
    USB_STATE_ATTACHED_WAIT_SRC,
    USB_STATE_ATTACHED_SNK,
    USB_STATE_ATTACHED_SRC
};

struct fusb302_dev_s {
    FAR struct i2c_dev_s *i2c;
    int int_n;
    int gpio_vbus_en;
    struct work_s isr_work;
    struct work_s dly_work;
    sem_t fusb302_sem;
    void *fusb302_available_arg;
    enum usb_attach_state_e state;

    struct fusb302_usb_ext_s usb_ext;
    struct fusb302_power_s ext_power;
};

static struct fusb302_dev_s *fusb302_info;

static enum usb_attach_state_e fusb302_get_state(void)
{
    if (fusb302_info)
      return fusb302_info->state;
    else
      return USB_STATE_UNATTACHED;
}

int fusb302_get_current(void)
{
  if (fusb302_info)
    return fusb302_info->ext_power.current;
  else
    return 0;
}

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

/* Number of retries for I2C reads/writes */
#define RETRIES_I2C 3
static inline int fusb302_reg_read(struct i2c_dev_s *i2c, uint8_t reg)
{
    uint8_t val;
    int ret;
    int retries = RETRIES_I2C;
    do {
       ret = I2C_WRITEREAD(i2c, &reg, sizeof(reg), &val, sizeof(val));
    } while (ret && --retries);
    if (ret)
        dbg("I2C_WRITEREAD fail ret=%d reg=0x%02x\n", ret, reg);
    return ret ? ret : val;
}

static inline int fusb302_reg_write(struct i2c_dev_s *i2c, uint8_t reg, uint8_t val)
{
    uint8_t buf[2];
    int ret;
    int retries = RETRIES_I2C;

    buf[0] = reg;
    buf[1] = val;

    do {
        ret = I2C_WRITE(i2c, buf, sizeof(buf));
    } while (ret && --retries);
    if (ret)
        dbg("I2C_WRITE fail ret=%d reg=0x%02x val=0x%02x\n", ret, reg, val);
    return ret;
}

static void fusb302_init(void)
{
    fusb302_reg_write(fusb302_info->i2c, FUSB302_RESET_REG, FUSB302_RESET_SW_RES_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_POWER_REG, FUSB302_POWER_PWR_INIT_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES0_REG, FUSB302_SWITCHES0_ZERO_MASK);
    /* listen to I_BC_LVL and I_TOGDONE only */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASK_REG, FUSB302_MASK_M_VBUSOK_MASK|
                                         FUSB302_MASK_M_ACTIVITY_MASK|
                                         FUSB302_MASK_M_CRC_CHK_MASK|FUSB302_MASK_M_ALERT_MASK|
                                         FUSB302_MASK_M_COLLISION_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASKA_REG, FUSB302_MASKA_M_OCP_TEMP_MASK|
                                         FUSB302_MASKA_M_SOFTFAIL_MASK|FUSB302_MASKA_M_RETRY_FAIL_MASK|
                                         FUSB302_MASKA_M_HARDSENT_MASK|FUSB302_MASKA_M_TXSENT_MASK|
                                         FUSB302_MASKA_M_SOFTRST_MASK|FUSB302_MASKA_M_HARDRST_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASKB_REG, FUSB302_MASKB_M_GCRCSENT_MASK);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SLICE_REG, FUSB302_SLICE_SDAC_HYS_085MV|
                                         0x20); /* magic */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL0_REG, FUSB302_CONTROL0_TX_START_MASK|
                                         FUSB302_CONTROL0_HOST_CUR_USB);
    /* enable toggle */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL2_REG, FUSB302_CONTROL2_WAKE_EN_MASK|
                                         FUSB302_CONTROL2_MODE_DRP_POLLING|FUSB302_CONTROL2_TOGGLE_MASK);
}

static void fusb302_read_status_regs(void)
{
    fusb302_regs.status0a   = fusb302_reg_read(fusb302_info->i2c, FUSB302_STATUS0A_REG);
    fusb302_regs.status1a   = fusb302_reg_read(fusb302_info->i2c, FUSB302_STATUS1A_REG);
    fusb302_regs.interrupta = fusb302_reg_read(fusb302_info->i2c, FUSB302_INTERRUPTA_REG);
    fusb302_regs.interruptb = fusb302_reg_read(fusb302_info->i2c, FUSB302_INTERRUPTB_REG);

    fusb302_regs.status0    = fusb302_reg_read(fusb302_info->i2c, FUSB302_STATUS0_REG);
    fusb302_regs.status1    = fusb302_reg_read(fusb302_info->i2c, FUSB302_STATUS1_REG);
    fusb302_regs.interrupt  = fusb302_reg_read(fusb302_info->i2c, FUSB302_INTERRUPT_REG);
};

static void fusb302_write_connected(uint8_t sw0, uint8_t sw1)
{
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES0_REG, sw0);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES1_REG, sw1);

    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASK_REG,
                                         FUSB302_MASK_M_ACTIVITY_MASK|
                                         FUSB302_MASK_M_CRC_CHK_MASK|FUSB302_MASK_M_ALERT_MASK|
                                         FUSB302_MASK_M_COLLISION_MASK|
                                         FUSB302_MASK_M_BC_LVL_MASK);
}

static void fusb302_write_connected_src(uint8_t cc)
{
    if (cc == 1) {
        fusb302_write_connected(
            (FUSB302_SWITCHES0_MEAS_CC1_MASK |
             FUSB302_SWITCHES0_PDWN2_MASK |
             FUSB302_SWITCHES0_PDWN1_MASK),
            (FUSB302_SWITCHES1_AUTO_CRC_MASK |
             FUSB302_SWITCHES1_TXCC1_MASK));
    } else if (cc == 2) {
        fusb302_write_connected(
            (FUSB302_SWITCHES0_PU_EN2_MASK |
             FUSB302_SWITCHES0_MEAS_CC2_MASK),
            (FUSB302_SWITCHES1_AUTO_CRC_MASK |
             FUSB302_SWITCHES1_TXCC2_MASK));
    } else {
        fusb302_write_connected(
            (FUSB302_SWITCHES0_PDWN2_MASK |
             FUSB302_SWITCHES0_PDWN1_MASK),
            0);
    }
}

static void fusb302_write_connected_snk(uint8_t cc)
{
    if (cc == 1) {
        fusb302_write_connected(
            (FUSB302_SWITCHES0_PU_EN1_MASK |
             FUSB302_SWITCHES0_MEAS_CC1_MASK),
            (FUSB302_SWITCHES1_AUTO_CRC_MASK |
             FUSB302_SWITCHES1_TXCC1_MASK));
    } else if (cc == 2) {
        fusb302_write_connected(
            (FUSB302_SWITCHES0_PU_EN2_MASK |
             FUSB302_SWITCHES0_MEAS_CC2_MASK),
            (FUSB302_SWITCHES1_AUTO_CRC_MASK |
             FUSB302_SWITCHES1_TXCC2_MASK));
    } else {
        fusb302_write_connected(
            (FUSB302_SWITCHES0_PU_EN2_MASK |
             FUSB302_SWITCHES0_PU_EN1_MASK |
             FUSB302_SWITCHES0_VCONN_CC2_MASK),
            0);
    }
}

static void fusb302_write_disconnected(void)
{
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES0_REG, 0x00);
    fusb302_reg_write(fusb302_info->i2c, FUSB302_SWITCHES1_REG, 0x00);
    /* mask all interrupts */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_MASK_REG,
                                         FUSB302_MASK_M_VBUSOK_MASK|
                                         FUSB302_MASK_M_ACTIVITY_MASK|FUSB302_MASK_M_COMP_CHNG_MASK|
                                         FUSB302_MASK_M_CRC_CHK_MASK|FUSB302_MASK_M_ALERT_MASK|
                                         FUSB302_MASK_M_WAKE_MASK|FUSB302_MASK_M_COLLISION_MASK|
                                         FUSB302_MASK_M_BC_LVL_MASK);
    /* clear toggle state */
    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL2_REG, 0x00);

    fusb302_reg_write(fusb302_info->i2c, FUSB302_CONTROL2_REG, FUSB302_CONTROL2_TOG_RD_ONLY_MASK|
                                         FUSB302_CONTROL2_WAKE_EN_MASK|FUSB302_CONTROL2_MODE_DRP_POLLING|
                                         FUSB302_CONTROL2_TOGGLE_MASK);
}

enum attach_type_e {
    ATTACH_TYPE_NONE,
    ATTACH_TYPE_SNK,
    ATTACH_TYPE_SRC,
    ATTACH_TYPE_ACC,
};

#define ATTACH_TYPE_VBUS_OK_BIT                       (0x01 << 3)
#define ATTACH_TYPE_COMP_BIT                          (0x01 << 4)

#define ATTACH_TYPE_SNK_ON_CC1  ATTACH_TYPE_VBUS_OK_BIT | FUSB302_STATUS1A_TOGSS_SNK_ON_CC1
#define ATTACH_TYPE_SNK_ON_CC2  ATTACH_TYPE_VBUS_OK_BIT | FUSB302_STATUS1A_TOGSS_SNK_ON_CC2
#define ATTACH_TYPE_SRC_ON_CC1A FUSB302_STATUS1A_TOGSS_SRC_ON_CC1
#define ATTACH_TYPE_SRC_ON_CC1B ATTACH_TYPE_VBUS_OK_BIT | FUSB302_STATUS1A_TOGSS_SRC_ON_CC1
#define ATTACH_TYPE_SRC_ON_CC2A FUSB302_STATUS1A_TOGSS_SRC_ON_CC2
#define ATTACH_TYPE_SRC_ON_CC2B ATTACH_TYPE_VBUS_OK_BIT | FUSB302_STATUS1A_TOGSS_SRC_ON_CC2

static enum attach_type_e fusb302_togss(int *cc)
{
    enum attach_type_e attach_type = ATTACH_TYPE_NONE;
    uint8_t togss = (fusb302_regs.status1a & FUSB302_STATUS1A_TOGSS_MASK)
        >> FUSB302_STATUS1A_TOGSS_SHIFT;

    togss |= (!!(fusb302_regs.status0 & FUSB302_STATUS0_VBUSOK_MASK)) << 3;
    togss |= (!!(fusb302_regs.status0 & FUSB302_STATUS0_COMP_MASK)) << 4;

    switch (togss) {
    case ATTACH_TYPE_SNK_ON_CC1:
        *cc = 1;
        attach_type = ATTACH_TYPE_SNK;
        break;
    case ATTACH_TYPE_SNK_ON_CC2:
        *cc = 2;
        attach_type = ATTACH_TYPE_SNK;
        break;
    case ATTACH_TYPE_SRC_ON_CC1A:
    case ATTACH_TYPE_SRC_ON_CC1B:
        *cc = 1;
        attach_type = ATTACH_TYPE_SRC;
        break;
    case ATTACH_TYPE_SRC_ON_CC2A:
    case ATTACH_TYPE_SRC_ON_CC2B:
        *cc = 2;
        attach_type = ATTACH_TYPE_SRC;
        break;
    case FUSB302_STATUS1A_TOGSS_AUTO_ACCESSORY:
        *cc = 0;
        attach_type = ATTACH_TYPE_ACC;
        break;
    default:
        *cc = 0;
        attach_type = ATTACH_TYPE_NONE;
        break;
    }

    return attach_type;
}

static void fusb302_notify_device_usb_ext(struct fusb302_dev_s *info)
{
    if (info->usb_ext.usb_ext_cb) {
        bool attached = (fusb302_get_state() == USB_STATE_ATTACHED_SRC ||
                         fusb302_get_state() == USB_STATE_ATTACHED_SNK);
        info->usb_ext.usb_ext_cb(info->usb_ext.dev,  attached);
    }
}

static void fusb302_notify_device_ext_power(struct fusb302_dev_s *info)
{
    int current = decode_cc_termination();

    if (current != info->ext_power.current) {
        info->ext_power.current = current;
        if (info->ext_power.power_changed_cb)
            info->ext_power.power_changed_cb(info->ext_power.arg);
    }
}

static void fusb302_dly_worker(FAR void *arg);

static void fusb302_update_state(struct fusb302_dev_s *info)
{
    int cc = 0;
    enum attach_type_e new_attach;

    sem_wait(&info->fusb302_sem);

    fusb302_read_status_regs();
    new_attach = fusb302_togss(&cc);

    switch(info->state) {

    case USB_STATE_UNATTACHED:
        switch (new_attach) {
            case ATTACH_TYPE_NONE:
                vdbg("unattached -> unattached\n");
                break;
            case ATTACH_TYPE_SNK:
                vdbg("unattached -> wait_snk\n");
                info->state = USB_STATE_ATTACHED_WAIT_SNK;
                fusb302_write_connected_snk(cc);

                if (info && work_available(&info->dly_work)) {
                    work_queue(LPWORK, &info->dly_work, fusb302_dly_worker,
                            info, MSEC2TICK(USB_DEBOUNCE_MS));
                }
                break;
            case ATTACH_TYPE_SRC:
                vdbg("unattached -> wait_src\n");
                info->state = USB_STATE_ATTACHED_WAIT_SRC;
                fusb302_write_connected_src(cc);

                if (info && work_available(&info->dly_work)) {
                    work_queue(LPWORK, &info->dly_work, fusb302_dly_worker,
                            info, MSEC2TICK(USB_DEBOUNCE_MS));
                }
                break;
            case ATTACH_TYPE_ACC:
                vdbg("unattached -> acc\n");
            default:
                ASSERT(FALSE);
                break;
        }
        break;

    case USB_STATE_ATTACHED_WAIT_SNK:
        switch (new_attach) {
            case ATTACH_TYPE_NONE:
                vdbg("wait_snk -> unattached\n");
                info->state = USB_STATE_UNATTACHED;
                fusb302_write_disconnected();
                break;
            case ATTACH_TYPE_SNK:
                vdbg("wait_snk -> snk\n");
                info->state = USB_STATE_ATTACHED_SNK;
                fusb302_notify_device_usb_ext(info);
                fusb302_notify_device_ext_power(info);
                break;
            case ATTACH_TYPE_SRC:
                vdbg("wait_snk -> wait_src\n");
                info->state = USB_STATE_ATTACHED_WAIT_SRC;
                fusb302_write_connected_src(cc);

                if (info && work_available(&info->dly_work)) {
                    work_queue(LPWORK, &info->dly_work, fusb302_dly_worker,
                            info, MSEC2TICK(USB_DEBOUNCE_MS));
                }
                break;
            case ATTACH_TYPE_ACC:
                vdbg("wait_snk -> acc\n");
            default:
                ASSERT(FALSE);
                break;
        }
        break;

    case USB_STATE_ATTACHED_WAIT_SRC:
        switch (new_attach) {
            case ATTACH_TYPE_NONE:
                vdbg("wait_src -> unattached\n");
                info->state = USB_STATE_UNATTACHED;
                fusb302_write_disconnected();
                break;
            case ATTACH_TYPE_SNK:
                vdbg("wait_src -> wait_snk\n");
                info->state = USB_STATE_ATTACHED_WAIT_SNK;
                fusb302_write_connected_snk(cc);

                if (info && work_available(&info->dly_work)) {
                    work_queue(LPWORK, &info->dly_work, fusb302_dly_worker,
                            info, MSEC2TICK(USB_DEBOUNCE_MS));
                }
                break;
            case ATTACH_TYPE_SRC:
                vdbg("wait_src -> src\n");
                info->state = USB_STATE_ATTACHED_SRC;
                fusb302_notify_device_usb_ext(info);
                fusb302_notify_device_ext_power(info);

                gpio_set_value(info->gpio_vbus_en, 1);
                break;
            case ATTACH_TYPE_ACC:
                vdbg("wait_src -> acc\n");
            default:
                ASSERT(FALSE);
                break;
        }
        break;

    case USB_STATE_ATTACHED_SNK:
        if (!(fusb302_regs.interrupt& FUSB302_INTERRUPT_I_VBUSOK_MASK))
            break;
        if (fusb302_regs.status0 & FUSB302_STATUS0_VBUSOK_MASK)
            break;

        switch (new_attach) {
            case ATTACH_TYPE_NONE:
                vdbg("snk -> unattached\n");
                info->state = USB_STATE_UNATTACHED;
                fusb302_write_disconnected();
                fusb302_notify_device_usb_ext(info);
                fusb302_notify_device_ext_power(info);
                break;
            case ATTACH_TYPE_SNK:
                vdbg("snk -> snk\n");
                break;
            case ATTACH_TYPE_SRC:
                vdbg("snk -> wait_src\n");
                info->state = USB_STATE_ATTACHED_WAIT_SRC;
                fusb302_write_connected_src(cc);

                if (info && work_available(&info->dly_work)) {
                    work_queue(LPWORK, &info->dly_work, fusb302_dly_worker,
                            info, MSEC2TICK(USB_DEBOUNCE_MS));
                }
                break;
            case ATTACH_TYPE_ACC:
                vdbg("snk -> acc\n");
            default:
                ASSERT(FALSE);
                break;
        }
        break;

    case USB_STATE_ATTACHED_SRC:
        switch (new_attach) {
            case ATTACH_TYPE_NONE:
                vdbg("src -> unattached\n");
                info->state = USB_STATE_UNATTACHED;
                fusb302_write_disconnected();
                fusb302_notify_device_usb_ext(info);

                gpio_set_value(info->gpio_vbus_en, 0);
                break;
            case ATTACH_TYPE_SNK:
                vdbg("src -> wait_snk\n");
                info->state = USB_STATE_ATTACHED_WAIT_SNK;
                fusb302_write_connected_snk(cc);

                gpio_set_value(info->gpio_vbus_en, 0);

                if (info && work_available(&info->dly_work)) {
                    work_queue(LPWORK, &info->dly_work, fusb302_dly_worker,
                            info, MSEC2TICK(USB_DEBOUNCE_MS));
                }
                break;
            case ATTACH_TYPE_SRC:
                vdbg("src -> src\n");
                break;
            case ATTACH_TYPE_ACC:
                vdbg("src -> acc\n");
            default:
                ASSERT(FALSE);
                break;
        }
        break;
    default:
        ASSERT(FALSE);
        break;
    }
    sem_post(&info->fusb302_sem);
}

static void fusb302_dly_worker(FAR void *arg)
{
    struct fusb302_dev_s *info = arg;

    fusb302_update_state(info);
}

static void fusb302_isr_worker(FAR void *arg)
{
    struct fusb302_dev_s *info = arg;
    bool waiting;

    sem_wait(&info->fusb302_sem);
    waiting = ((info->state == USB_STATE_ATTACHED_WAIT_SNK ||
                info->state == USB_STATE_ATTACHED_WAIT_SRC));
    sem_post(&info->fusb302_sem);

    if (!waiting)
        fusb302_update_state(info);
}

static int fusb302_isr(int irq, void *context)
{
    pm_activity(PM_USBC_ACTIVITY);
    if (fusb302_info && work_available(&fusb302_info->isr_work))
        work_queue(LPWORK, &fusb302_info->isr_work, fusb302_isr_worker, fusb302_info, 0);

    return OK;
}

/**
 *  Entry point for configuration
 */
int fusb302_register(int int_n, int gpio_vbus_en)
{
    int ret;
    struct fusb302_dev_s *info;

    info = kmm_zalloc(sizeof(*fusb302_info));
    if (!info) {
        dbg("failed to allocate info\n");
        ret = -ENOMEM;
        goto init_done;
    }

    info->int_n = int_n;
    info->gpio_vbus_en = gpio_vbus_en;

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
    gpio_direction_out(info->gpio_vbus_en, 0);

    sem_init(&info->fusb302_sem, 0, 1);

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

    /* reset and initialize chipset */
    fusb302_init();

    return 0;

init_free_info:
    fusb302_info = NULL;
    free(info);
init_done:
    return ret;
}

void fusb302_unregister(void)
{
  kmm_free(fusb302_info);
  fusb302_info = NULL;
}

#ifdef CONFIG_FUSB302_USB_EXT
/*
 * device_usb_ext interface
 */
int fusb302_usb_ext_register_callback(struct device *dev, usb_ext_event_callback callback)
{
    fusb302_info->usb_ext.usb_ext_cb = callback;

    return 0;
}

int fusb302_usb_ext_unregister_callback(struct device *dev)
{
    fusb302_info->usb_ext.usb_ext_cb = NULL;

    return 0;
}

static uint8_t fusb302_usb_ext_get_attached(void)
{
    return (fusb302_get_state() == USB_STATE_ATTACHED_SRC ||
            fusb302_get_state() == USB_STATE_ATTACHED_SNK);
}

static uint8_t fusb302_usb_ext_get_protocol(void)
{
    return GB_USB_EXT_PROTOCOL_3_1;
}

static uint8_t fusb302_usb_ext_get_path(void)
{
    return fusb302_info->usb_ext.path;
}

static uint8_t fusb302_usb_ext_get_type(void)
{
    return (fusb302_get_state() == USB_STATE_ATTACHED_SRC) ? 
            GB_USB_EXT_REMOTE_DEVICE : GB_USB_EXT_REMOTE_HOST;
}

static int fusb302_usb_ext_open(struct device *dev)
{
    return 0;
}

static void fusb302_usb_ext_close(struct device *dev)
{
}

static int fusb302_usb_ext_probe(struct device *dev)
{
    struct device_resource *path_res;

    fusb302_info->usb_ext.dev = dev;
    path_res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS, "path");
    if (path_res)
        fusb302_info->usb_ext.path = path_res->start;

    return 0;
}

static void fusb302_usb_ext_remove(struct device *dev)
{
    fusb302_info->usb_ext.dev = NULL;
    fusb302_usb_ext_close(dev);
}

static struct device_usb_ext_type_ops fusb302_type_ops = {
    .get_attached         = fusb302_usb_ext_get_attached,
    .get_protocol         = fusb302_usb_ext_get_protocol,
    .get_path             = fusb302_usb_ext_get_path,
    .get_type             = fusb302_usb_ext_get_type,
    .register_callback    = fusb302_usb_ext_register_callback,
    .unregister_callback  = fusb302_usb_ext_unregister_callback,
};

static struct device_driver_ops fusb302_driver_ops = {
    .probe              = fusb302_usb_ext_probe,
    .remove             = fusb302_usb_ext_remove,
    .open               = fusb302_usb_ext_open,
    .close              = fusb302_usb_ext_close,
    .type_ops           = &fusb302_type_ops,
};

struct device_driver fusb302_usb_ext_driver = {
    .type       = DEVICE_TYPE_USB_EXT_HW,
    .name       = "fusb302_usb_ext",
    .desc       = "USB-EXT Interface",
    .ops        = &fusb302_driver_ops,
};
#endif /* CONFIG_FUSB302_USB_EXT */

#ifdef CONFIG_FUSB302_EXT_POWER
/**
 * device_ext_power interface
 */
static int fusb302_ext_power_register_callback(struct device *dev,
        device_ext_power_notification cb, void *arg)
{
    struct fusb302_dev_s *info = device_get_private(dev);

    if (!info)
        return 0;

    /* get the initial state of the connection */
    info->state = fusb302_get_state();

    if (info->ext_power.power_changed_cb) {
        dbg("already registered\n");
        return -EINVAL;
    }

    if (!cb) {
        dbg("invalid parameter\n");
        return -EINVAL;
    }

    info->ext_power.power_changed_cb = cb;
    info->ext_power.arg = arg;

    return 0;
}

static int fusb302_ext_power_set_max_output_voltage(struct device *dev,
                                                    int voltage)
{
    return voltage >= OUTPUT_VOLTAGE ? 0 : -EINVAL;
}

static int fusb302_ext_power_get_output(struct device *dev,
                                        device_ext_power_output_s *output)
{
    struct fusb302_dev_s *info = device_get_private(dev);

    if (info->state == USB_STATE_ATTACHED_SNK) {
        output->current = fusb302_get_current();
        output->voltage = OUTPUT_VOLTAGE;
    } else
        output->voltage = output->current = 0;

    return 0;
}

static int fusb302_ext_power_open(struct device *dev)
{
    return 0;
}

static void fusb302_ext_power_close(struct device *dev)
{
}

static int fusb302_ext_power_probe(struct device *dev)
{
    device_set_private(dev, fusb302_info);
    fusb302_info->ext_power.dev = dev;
    return 0;
}

void fusb302_ext_power_remove(struct device *dev)
{
    fusb302_info->ext_power.dev = NULL;
    device_set_private(dev, NULL);
}

static struct device_ext_power_type_ops fusb302_ext_power_type_ops  = {
    .register_callback = fusb302_ext_power_register_callback,
    .set_max_output_voltage = fusb302_ext_power_set_max_output_voltage,
    .get_output = fusb302_ext_power_get_output,
};

static struct device_driver_ops fusb302_ext_power_driver_ops = {
    .probe = fusb302_ext_power_probe,
    .remove = fusb302_ext_power_remove,
    .open = fusb302_ext_power_open,
    .close = fusb302_ext_power_close,
    .type_ops = &fusb302_ext_power_type_ops,
};

/* device_ext_power */
struct device_driver fusb302_ext_power_driver = {
    .type = DEVICE_TYPE_EXT_POWER_HW,
    .name = "fusb302_ext_power",
    .desc = "FUSB 302 External Power",
    .ops = &fusb302_ext_power_driver_ops,
};
#endif /* CONFIG_FUSB302_EXT_POWER */
