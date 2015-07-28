/*
 * Copyright (c) 2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c.h>
#include <nuttx/gpio/tca64xx.h>

#include <stdio.h>
#include <errno.h>

#include <arch/tsb/gpio.h>
#include <arch/tsb/unipro.h>
#include <arch/tsb/device_table.h>
#include <arch/tsb/driver.h>
#include <apps/greybus-utils/utils.h>
#include <apps/ara/service_mgr.h>
#include <apps/ara/gb_loopback.h>
#include <apps/nsh.h>

#ifdef CONFIG_BOARD_HAVE_DISPLAY
#include <arch/board/dsi.h>
#endif

#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
#include <arch/board/csi.h>
#endif

#ifdef CONFIG_BOARD_HAVE_DISPLAY
#define TCA6408_U72             0x20
#define TCA6408_U72_INT_GPIO    0x03
#define TCA6408_U72_RST_GPIO    0x04

int io_expander_init(struct i2c_dev_s **dev)
{
    void *driver_data;

    *dev = up_i2cinitialize(0);
    if (!*dev) {
        lowsyslog("%s(): Failed to get I/O Expander I2C bus 0\n", __func__);
        return -ENODEV;
    } else {
        if (tca64xx_init(&driver_data,
                         TCA6408_PART,
                         *dev,
                         TCA6408_U72,
                         TCA6408_U72_RST_GPIO,
                         TCA6408_U72_INT_GPIO,
                         TCA6408_GPIO_BASE) < 0) {
            lowsyslog("%s(): Failed to register I/O Expander(0x%02x)\n",
                      __func__, TCA6408_U72);
            up_i2cuninitialize(*dev);
            return -ENODEV;
        }
    }

    return 0;
}
#endif

static struct srvmgr_service services[] = {
#if defined(CONFIG_ARA_GB_LOOPBACK)
    {
        .name = "gb_loopback",
        .func = gb_loopback_service,
    },
#endif
    { NULL, NULL }
};

int bridge_main(int argc, char *argv[])
{
#ifdef CONFIG_BOARD_HAVE_DISPLAY
    struct i2c_dev_s *dev;
#endif

    tsb_gpio_register(NULL);

#ifdef CONFIG_BOARD_HAVE_DISPLAY
    io_expander_init(&dev);

    display_init(dev);
#endif

#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
    camera_init();
#endif

    tsb_device_table_register();
    tsb_driver_register();

    enable_manifest("IID-1", NULL, 0);
    gb_unipro_init();
    enable_cports();
    srvmgr_start(services);

#ifdef CONFIG_EXAMPLES_NSH
    printf("Calling NSH\n");
    return nsh_main(argc, argv);
#else
    return 0;
#endif
}

