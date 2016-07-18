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

/* The apbe-ctrl driver performs two functions:
 *    1. Provide a service that drivers use to 'vote' to request the APBA/APBE
 *       functionality.  The votes are tallied and the APBA/APBE are enabled IFF
 *       at least one driver needs the functionality.  The results of the tally
 *       are known as the 'slave_state'.
 *    2. Provide a service to the APBA (that it uses via the AP and Greybus) to
 *       control the APBE's power and to check the APBE's boot status.  The
 *       current power status is known as the 'slave status'.
 *
 *  These functions are accessed via three groups of thread-contexts:
 *    1. Driver threads (e.g. display or camera threads) for voting for the
 *       APBA/APBE.
 *    2. Greybus threads (e.g. GB Mods Control thread) to control or report the
 *       APBE status.
 *    3. Attach notifier thread for attach and detach requests.
 *
 *  Block diagram:
 *                        ----------      ------------
 *                        |        |      |  SLAVE   | <--- (un)register_slave_state_cb
 *  send_slave_state ---> | VOTING | ---> |  STATE   | ----> slave_state_cb (notify current vote)
 *    (cast vote)         |        |      | CALLBACK | <--- op_get_mask     (query current vote)
 *                        ----------      -----------
 *
 *                                   ------------      ------------
 * (un)register_slave_status_cb ---> |  SLAVE   |      |  POWER   | <--- op_set_mode (power on/off)
 *              slave_status_cb <--- |  STATUS  | <--- | CONTROL  |
 *                                   | CALLBACK |      |          | <--- attach_changed (attach state)
 *                                   ------------      -----------
 */

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <unistd.h>

#include <arch/board/mods.h>

#include <nuttx/device.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/gpio.h>
#include <nuttx/util.h>

#include <nuttx/greybus/mods-ctrl.h>
#include <nuttx/greybus/mods.h>

#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>

#define APBE_PWR_EN_DELAY (100 * 1000)
#define APBE_RST_N_DELAY (100 * 1000)

struct apbe_ctrl_info {
    sem_t apbe_pwrctrl_sem;
    slave_state_callback slave_state_cb;
    struct device *slave_status_dev[4];
    slave_status_callback slave_status_cb[4];
    uint32_t slave_state_ref_cnt;
    enum base_attached_e attached;
    uint32_t status;
    bool is_initialized;
    struct device *mhb_dev;
    gpio_cfg_t uart_tx_cfg;
    gpio_cfg_t uart_rts_cfg;
};

static struct apbe_ctrl_info apbe_ctrl;

static int mhb_handle_pm(struct device *dev, struct mhb_hdr *hdr,
               uint8_t *payload, size_t payload_length);


static void apbe_pwrctrl_disable_uart(struct apbe_ctrl_info *info)
{
    if (apbe_ctrl.uart_tx_cfg || apbe_ctrl.uart_rts_cfg) {
        dbg("WARNING: UART already disabled\n");
        return;
    }

    /* Save pin config for the output UART pins and set to inputs */
#ifdef GPIO_APBE_UART_TX
    apbe_ctrl.uart_tx_cfg = gpio_cfg_save(GPIO_APBE_UART_TX);
    gpio_direction_in(GPIO_APBE_UART_TX);
#endif
#ifdef GPIO_APBE_UART_RTS
    apbe_ctrl.uart_rts_cfg = gpio_cfg_save(GPIO_APBE_UART_RTS);
    gpio_direction_in(GPIO_APBE_UART_RTS);
#endif
}

static void apbe_pwrctrl_enable_uart(struct apbe_ctrl_info *info)
{
    /* Restore all output UART pins to UART */
#ifdef GPIO_APBE_UART_TX
    if (apbe_ctrl.uart_tx_cfg) {
        gpio_cfg_restore(GPIO_APBE_UART_TX, info->uart_tx_cfg);
        apbe_ctrl.uart_tx_cfg = NULL;
    }
#endif
#ifdef GPIO_APBE_UART_RTS
    if (apbe_ctrl.uart_rts_cfg) {
        gpio_cfg_restore(GPIO_APBE_UART_RTS, info->uart_rts_cfg);
        apbe_ctrl.uart_rts_cfg = NULL;
    }
#endif
}

static int apbe_pwrctrl_op_get_mask(struct device *dev, uint32_t *mask)
{
    struct apbe_ctrl_info *ctrl_info = device_get_private(dev);

    if (!ctrl_info)
        return -EINVAL;

    if (mask) {
        *mask = MB_CONTROL_SLAVE_MASK_APBE;
    }

    dbg("state=%d\n", (mask ? *mask : 0));
    return OK;
}

static int apbe_pwrctrl_notify_slave_status(struct apbe_ctrl_info *ctrl_info)
{
    int i;

    sem_wait(&ctrl_info->apbe_pwrctrl_sem);

    for (i=0; i < ARRAY_SIZE(ctrl_info->slave_status_cb); i++) {
        if (ctrl_info->slave_status_cb[i]) {
            struct device *dev = ctrl_info->slave_status_dev[i];
            ctrl_info->slave_status_cb[i](dev, ctrl_info->status);
        }
    }
    sem_post(&ctrl_info->apbe_pwrctrl_sem);

    return 0;
}

static int apbe_power_off(struct apbe_ctrl_info *ctrl_info)
{
    vdbg("\n");

    /* Close the UART */
    if (ctrl_info->mhb_dev) {
        /* Close the MHB device. */
        device_mhb_unregister_receiver(ctrl_info->mhb_dev, MHB_ADDR_PM, mhb_handle_pm);

        device_close(ctrl_info->mhb_dev);
        ctrl_info->mhb_dev = NULL;
    }

    /* Power off the APBE */
    gpio_set_value(GPIO_APBE_SPIBOOT_N, 0);
    gpio_set_value(GPIO_APBE_PWR_EN, 0);
    gpio_set_value(GPIO_APBE_RST_N, 0);

    /* Disable UART */
    apbe_pwrctrl_disable_uart(ctrl_info);

    /* Notify listeners */
    ctrl_info->status = MHB_PM_STATUS_PEER_NONE;
    apbe_pwrctrl_notify_slave_status(ctrl_info);

    vdbg("done\n");

    return 0;
}

static int apbe_power_on(struct apbe_ctrl_info *ctrl_info)
{
    vdbg("\n");

    if (ctrl_info->mhb_dev) {
        dbg("ERROR: MHB already opened\n");
        return -EINVAL;
    }

    /* Enable UART */
    apbe_pwrctrl_enable_uart(ctrl_info);

    /* Open the MHB device */
    ctrl_info->mhb_dev = device_open(DEVICE_TYPE_MHB, MHB_ADDR_PM);
    if (!ctrl_info->mhb_dev) {
        dbg("ERROR: MHB failed to open\n");
        return -ENOSYS;
    }

    device_mhb_register_receiver(apbe_ctrl.mhb_dev, MHB_ADDR_PM, mhb_handle_pm);

    /* Power on the APBE */
    gpio_set_value(GPIO_APBE_SPIBOOT_N, 0);
    gpio_set_value(GPIO_APBE_PWR_EN, 1);
    usleep(APBE_PWR_EN_DELAY);
    gpio_set_value(GPIO_APBE_RST_N, 1);
    usleep(APBE_RST_N_DELAY);

    vdbg("done\n");

    return 0;
}

static int apbe_pwrctrl_op_set_mode(struct device *dev, enum slave_pwrctrl_mode mode)
{
    struct apbe_ctrl_info *ctrl_info = device_get_private(dev);

    dbg("power mode=%d\n", mode);
    switch (mode) {
    case SLAVE_PWRCTRL_POWER_ON:
    case SLAVE_PWRCTRL_POWER_FLASH_MODE:
        apbe_power_on(ctrl_info);
        break;
    case SLAVE_PWRCTRL_POWER_OFF:
        apbe_power_off(ctrl_info);
        break;
    }

    return OK;
}

static int apbe_pwrctrl_attach_changed(FAR void *arg, const void *data)
{
    struct apbe_ctrl_info *ctrl_info = arg;
    enum base_attached_e state = *((enum base_attached_e *)data);

    DEBUGASSERT(ctrl_info);

    while (sem_wait(&ctrl_info->apbe_pwrctrl_sem) != OK) {
        if (errno == EINVAL) {
            return -errno;
        }
    }

    vdbg("new attach=%d, old attach=%d\n", state, ctrl_info->attached);

    ctrl_info->attached = state;
    sem_post(&ctrl_info->apbe_pwrctrl_sem);

    return OK;
}

static int mhb_handle_pm(struct device *dev, struct mhb_hdr *hdr,
               uint8_t *payload, size_t payload_length)
{
    struct apbe_ctrl_info *ctrl_info = &apbe_ctrl;
    struct mhb_pm_status_not *not = (struct mhb_pm_status_not *)payload;

    if (hdr->type != MHB_TYPE_PM_STATUS_NOT) {
        /* Ignore other messages */
        return 0;
    }

    if (payload_length != sizeof(*not)) {
        dbg("ERROR: Invalid PM status notify\n");
        return -EINVAL;
    }

    vdbg("status=%d\n", not->status);

    ctrl_info->status = not->status;
    apbe_pwrctrl_notify_slave_status(ctrl_info);

    return 0;
}

static int apbe_pwrctrl_probe(struct device *dev)
{
    device_set_private(dev, &apbe_ctrl);
    /* multiple devices can be registered for this driver,
     * if its already initialized return.
     */
    if (apbe_ctrl.is_initialized)
        return 0;

    sem_init(&apbe_ctrl.apbe_pwrctrl_sem, 0, 1);
#ifdef GPIO_MODS_SPI_SEL
    gpio_direction_out(GPIO_MODS_SPI_SEL, 0);
#endif
    gpio_direction_out(GPIO_APBE_SPIBOOT_N, 0);
    gpio_direction_out(GPIO_APBE_PWR_EN, 0);
    gpio_direction_out(GPIO_APBE_RST_N, 0);
    apbe_ctrl.attached = BASE_DETACHED;
    if (mods_attach_register(apbe_pwrctrl_attach_changed, &apbe_ctrl))
        dbg("ERROR: failed to register mods_attach callback\n");

    apbe_ctrl.is_initialized = true;

    return 0;
}

static int apbe_pwrctrl_register_slave_state_cb(struct device *dev,
                           slave_state_callback cb)
{
    struct apbe_ctrl_info *ctrl_info = device_get_private(dev);

    if (!ctrl_info)
        return -EINVAL;

    ctrl_info->slave_state_cb = cb;
    return 0;
}

static int apbe_pwrctrl_unregister_slave_state_cb(struct device *dev)
{
    struct apbe_ctrl_info *ctrl_info = device_get_private(dev);

    if (!ctrl_info)
        return -EINVAL;

    ctrl_info->slave_state_cb = NULL;

    return 0;
}

static int apbe_pwrctrl_send_slave_state(struct device *dev, uint32_t slave_state)
{
    int ret = 0;
    struct apbe_ctrl_info *ctrl_info = device_get_private(dev);
    uint32_t curr_ref_cnt;
    bool power_off = false;

    if (!ctrl_info)
        return -EINVAL;

    while (sem_wait(&ctrl_info->apbe_pwrctrl_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    curr_ref_cnt = ctrl_info->slave_state_ref_cnt;
    if (slave_state == SLAVE_STATE_ENABLED) {
        ctrl_info->slave_state_ref_cnt++;
    } else if (slave_state == SLAVE_STATE_DISABLED) {
        if (ctrl_info->slave_state_ref_cnt != 0)
            ctrl_info->slave_state_ref_cnt--;
    } else if (slave_state == SLAVE_STATE_RESET) {
        if (ctrl_info->slave_state_cb) {
            dbg("Reset Slave\n");
            ret  = ctrl_info->slave_state_cb(dev, SLAVE_STATE_DISABLED);
            ret |= ctrl_info->slave_state_cb(dev, SLAVE_STATE_ENABLED);
            if (ret) dbg("FAILED to Reset Slave\n");
        } else {
            dbg("slave state cb is not registered - no reset\n");
            ret = -EIO;
            goto err;
        }
    } else {
       ret = -EINVAL;
       goto err;
    }

    if (!!ctrl_info->slave_state_ref_cnt != !!curr_ref_cnt) {
        slave_state = (slave_state == SLAVE_STATE_ENABLED) ?
            SLAVE_STATE_ENABLED:SLAVE_STATE_DISABLED;

        dbg("update state state %s, attached=%d, new votes=%d, old votes=%d\n",
            slave_state?"Enabled":"Disabled",
            ctrl_info->attached, ctrl_info->slave_state_ref_cnt, curr_ref_cnt);

        if (ctrl_info->attached == BASE_ATTACHED) {
            if (ctrl_info->slave_state_cb) {
                vdbg("send state\n");
                ret = ctrl_info->slave_state_cb(dev, slave_state);
            } else {
                dbg("ERROR: slave state cb is not registered\n");
                ret = -EIO;
            }
        } else if (slave_state == SLAVE_STATE_DISABLED) {
            /* The base is not attached, but we want to disable. */
            power_off = true;
        }
    } else {
        vdbg("no change, state=%d, votes=%d\n",
             slave_state, ctrl_info->slave_state_ref_cnt);
    }

err:
    sem_post(&ctrl_info->apbe_pwrctrl_sem);

    /* Execute outside of the semaphore since the notification needs the sem. */
    if (power_off) {
        apbe_power_off(ctrl_info);
    }

    return ret;
}

static int apbe_pwrctrl_register_slave_status_cb(struct device *dev,
                                                 slave_status_callback cb)
{
    size_t i;
    struct apbe_ctrl_info *ctrl_info = device_get_private(dev);

    if (!ctrl_info) {
        dbg("ERROR: Invalid device\n");
        return -ENODEV;
    }

    if (cb == NULL) {
        dbg("ERROR: Invalid cb\n");
        return -EINVAL;
    }

    for (i=0; i < ARRAY_SIZE(ctrl_info->slave_status_cb); i++) {
        if (!ctrl_info->slave_status_cb[i]) {
            ctrl_info->slave_status_dev[i] = dev;
            ctrl_info->slave_status_cb[i] = cb;
            ctrl_info->slave_status_cb[i](dev, ctrl_info->status);
            return 0;
        }
    }

    dbg("ERROR: Failed to register\n");
    return -ENOMEM;
}

static int apbe_pwrctrl_unregister_slave_status_cb(struct device *dev,
                                                   slave_status_callback cb)
{
    size_t i;
    struct apbe_ctrl_info *ctrl_info = device_get_private(dev);

    if (!ctrl_info) {
        dbg("ERROR: Invalid device\n");
        return -ENODEV;
    }

    for (i=0; i < ARRAY_SIZE(ctrl_info->slave_status_cb); i++) {
        if (ctrl_info->slave_status_cb[i] == cb) {
            ctrl_info->slave_status_dev[i] = NULL;
            ctrl_info->slave_status_cb[i] = NULL;
            return 0;
        }
    }

    dbg("ERROR: Failed to unregister\n");
    return -EINVAL;
}

static struct device_slave_pwrctrl_type_ops apbe_pwrctrl_type_ops = {
    .get_mask = apbe_pwrctrl_op_get_mask,
    .set_mode = apbe_pwrctrl_op_set_mode,
    .register_slave_state_cb = apbe_pwrctrl_register_slave_state_cb,
    .unregister_slave_state_cb = apbe_pwrctrl_unregister_slave_state_cb,
    .send_slave_state = apbe_pwrctrl_send_slave_state,
    .register_slave_status_cb = apbe_pwrctrl_register_slave_status_cb,
    .unregister_slave_status_cb = apbe_pwrctrl_unregister_slave_status_cb,
};

static struct device_driver_ops apbe_pwrctrl_driver_ops = {
    .probe    = &apbe_pwrctrl_probe,
    .type_ops = &apbe_pwrctrl_type_ops,
};

const struct device_driver apbe_pwrctrl_driver = {
    .type   = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
    .name   = "slave_pwrctrl",
    .desc   = "slave power control",
    .ops    = &apbe_pwrctrl_driver_ops,
};
