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

#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <arch/board/mods.h>
#include <nuttx/device.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/greybus/mods-ctrl.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/gpio.h>


#define APBE_PWR_EN_DELAY (100 * 1000)
#define APBE_RST_N_DELAY (100 * 1000)

struct apbe_ctrl_info {
    sem_t apbe_pwrctrl_sem;
    slave_state_callback slave_state_cb;
    uint32_t slave_state_ref_cnt;
    enum base_attached_e attached;
    bool is_initialized;
};

static struct apbe_ctrl_info apbe_ctrl;

static void apbe_power_off(void)
{
    gpio_set_value(GPIO_APBE_SPIBOOT_N, 0);
    gpio_set_value(GPIO_APBE_WAKE, 0);
    gpio_set_value(GPIO_APBE_PWR_EN, 0);
    gpio_set_value(GPIO_APBE_RST_N, 0);
}

static void apbe_power_on_normal(void)
{
    gpio_set_value(GPIO_APBE_SPIBOOT_N, 0);
    gpio_set_value(GPIO_APBE_WAKE, 0);
    gpio_set_value(GPIO_APBE_PWR_EN, 1);
    usleep(APBE_PWR_EN_DELAY);
    gpio_set_value(GPIO_APBE_RST_N, 1);
    usleep(APBE_RST_N_DELAY);
}

static void apbe_power_on_flash(void)
{
    gpio_set_value(GPIO_APBE_SPIBOOT_N, 1);
    gpio_set_value(GPIO_APBE_WAKE, 1);
    gpio_set_value(GPIO_APBE_PWR_EN, 1);
    usleep(APBE_PWR_EN_DELAY);
    gpio_set_value(GPIO_APBE_RST_N, 1);
    usleep(APBE_RST_N_DELAY);
}

static int apbe_pwrctrl_op_get_mask(struct device *dev, uint32_t *mask)
{
    if (mask)
        *mask = MB_CONTROL_SLAVE_MASK_APBE;

    lldbg("mask=0x%x\n", (mask ? *mask : 0));
    return OK;
}

static int apbe_pwrctrl_op_set_mode(struct device *dev, enum slave_pwrctrl_mode mode)
{
    lldbg("mode=%d\n", mode);
    switch (mode) {
    case SLAVE_PWRCTRL_POWER_ON:
        apbe_power_on_normal();
        break;
    case SLAVE_PWRCTRL_POWER_FLASH_MODE:
        apbe_power_on_flash();
        break;
    case SLAVE_PWRCTRL_POWER_OFF:
    default:
        apbe_power_off();
        break;
    }

    return OK;
}
static void apbe_pwrctrl_attach_changed(FAR void *arg, enum base_attached_e state)
{
    struct apbe_ctrl_info *ctrl_info = arg;

    if (!ctrl_info) {
        lldbg("ctrl info null \n");
        return;
    }

    while (sem_wait(&ctrl_info->apbe_pwrctrl_sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    ctrl_info->attached = state;
    sem_post(&ctrl_info->apbe_pwrctrl_sem);
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
    gpio_direction_out(GPIO_APBE_SPIBOOT_N, 0);
    gpio_direction_out(GPIO_APBE_WAKE, 0);
    gpio_direction_out(GPIO_APBE_PWR_EN, 0);
    gpio_direction_out(GPIO_APBE_RST_N, 0);
    apbe_ctrl.attached = BASE_DETACHED;
    if (mods_attach_register(apbe_pwrctrl_attach_changed, &apbe_ctrl))
        lldbg("failed to register mods_attach callback\n");

    apbe_ctrl.is_initialized = true;

    return 0;
}

static void apbe_pwrctrl_remove(struct device *dev)
{
    apbe_power_off();
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

    if (!ctrl_info)
        return -EINVAL;

    while (sem_wait(&ctrl_info->apbe_pwrctrl_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    if (ctrl_info->attached != BASE_ATTACHED) {
         lldbg("base is not attached\n");
         ret = -EINVAL;
         goto err;
    }

    if (!ctrl_info->slave_state_cb) {
        lldbg("slave state cb is not registered\n");
        ret = -EIO;
        goto err;
    }
    curr_ref_cnt = ctrl_info->slave_state_ref_cnt;
    if (slave_state == SLAVE_STATE_ENABLED)
       ctrl_info->slave_state_ref_cnt++;
    else if ((slave_state == SLAVE_STATE_DISABLED) &&
                               (ctrl_info->slave_state_ref_cnt != 0))
       ctrl_info->slave_state_ref_cnt--;
    else {
       ret = -EINVAL;
       goto err;
    }

    if ((ctrl_info->slave_state_ref_cnt <= 1) &&
                      (curr_ref_cnt <= 1)) {
            lldbg("update slave state\n");
            ret = ctrl_info->slave_state_cb(dev, slave_state);
    }

err:
    sem_post(&ctrl_info->apbe_pwrctrl_sem);

    return ret;
}

static struct device_slave_pwrctrl_type_ops apbe_pwrctrl_type_ops = {
    .get_mask = apbe_pwrctrl_op_get_mask,
    .set_mode = apbe_pwrctrl_op_set_mode,
    .register_slave_state_cb = apbe_pwrctrl_register_slave_state_cb,
    .unregister_slave_state_cb = apbe_pwrctrl_unregister_slave_state_cb,
    .send_slave_state = apbe_pwrctrl_send_slave_state,
};

static struct device_driver_ops apbe_pwrctrl_driver_ops = {
    .probe    = &apbe_pwrctrl_probe,
    .remove   = &apbe_pwrctrl_remove,
    .type_ops = &apbe_pwrctrl_type_ops,
};

const struct device_driver apbe_pwrctrl_driver = {
    .type   = DEVICE_TYPE_SLAVE_PWRCTRL_HW,
    .name   = "slave_pwrctrl",
    .desc   = "slave power control",
    .ops    = &apbe_pwrctrl_driver_ops,
};
