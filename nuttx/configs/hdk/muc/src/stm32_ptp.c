/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#include <nuttx/device_ptp.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/power/bq24292.h>

#include <debug.h>
#include <errno.h>

static int ptp_set_current_flow(struct device *dev, uint8_t direction)
{
    switch (direction) {
    case PTP_CURRENT_OFF:
        return bq24292_set_chg(BQ24292_CHG_OFF);
        break;
    case PTP_CURRENT_TO_MOD:
        // If battery is dead, charge IC is off and does not communicate. It
        // will default to the "charge battery" once power is applied.
        (void) bq24292_set_chg(BQ24292_CHG_BATTERY);
        return 0;
    case PTP_CURRENT_FROM_MOD:
        return bq24292_set_chg(BQ24292_OTG_1300MA);
    default:
         return -EINVAL;
    }
}

static void ptp_attach_cb(FAR void *arg, enum base_attached_e state)
{
    switch (state) {
    case BASE_ATTACHED_OFF:
        /* Base is off and/or dead. Enable base charging to recover. */
        bq24292_set_chg(BQ24292_OTG_1300MA);
        break;
    case BASE_ATTACHED:
    case BASE_DETACHED:
    case BASE_INVALID:
    default:
        /* Ensure that base charging is disabled */
        bq24292_set_chg(BQ24292_CHG_OFF);
        break;
    }
}

static int ptp_open(struct device *dev)
{
    int retval;

    retval = mods_attach_register(ptp_attach_cb, NULL);
    if (retval) {
        dbg("failed to register mods_attach cb\n");
        return retval;
    }

    return 0;
}

static struct device_ptp_type_ops ptp_type_ops = {
    .set_current_flow = ptp_set_current_flow,
};

static struct device_driver_ops ptp_driver_ops = {
    .open = ptp_open,
    .type_ops = &ptp_type_ops,
};

struct device_driver ptp_driver = {
    .type = DEVICE_TYPE_PTP_HW,
    .name = "ptp_device",
    .desc = "Power transfer protocol",
    .ops = &ptp_driver_ops,
};
