/**
 * Copyright (c) 2015 Google Inc.
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
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <nuttx/config.h>
#include <nuttx/usb.h>
#include <nuttx/arch.h>

#include <stdio.h>

#define SET_PORT_FEATURE 0x2303
#define GET_PORT_STATUS  0xa300

#define PORT_POWERING_DELAY_IN_MS   1000

#define PORT_POWER       0x8

/**
 * Test the set-up of an HSIC link
 * @return 0 if successful
 */
static int test_hsic_link(void)
{
    int retval;
    char buf[4];
    struct device *dev;

    dev = device_open(DEVICE_TYPE_USB_HCD, 0);
    if (!dev) {
        return -ENODEV;
    }

    retval = device_usb_hcd_start(dev);
    if (retval) {
        goto error;
    }

    // Put power on Port 1
    device_usb_hcd_hub_control(dev, SET_PORT_FEATURE, PORT_POWER, 1, buf, 0);
    up_mdelay(PORT_POWERING_DELAY_IN_MS);

    // Get port status of Port 1. Expecting a 4 bytes response.
    device_usb_hcd_hub_control(dev, GET_PORT_STATUS, 0, 1, buf, 4);

    device_close(dev);

    return !*((uint32_t*) buf);

error:
    device_close(dev);
    return retval;
}

/**
 * Pretty print the result of a test
 *
 * @param name Name of the test
 * @param result true if the test passed, false otherwise
 */
static void print_test_result(const char *name, bool result)
{
    printf("%s: %s\e[m\n", name, result ? "\e[0;32mPASS" : "\e[1;31mFAIL");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int usb_host_main(int argc, char *argv[])
#endif
{
    int retval;

    printf("Tests:\n");

    retval = test_hsic_link();
    print_test_result("HSIC connection to the hub", !retval);
    if (retval) {
        fprintf(stderr,
                "ERROR: Cannot establish hsic connection with the hub.\n");
        goto end;
    }

    retval = test_hsic_link();
    print_test_result("Reset of the hub", !retval);
    if (retval) {
        fprintf(stderr, "ERROR: Cannot establish hsic connection with the hub "
                        "a second time (RESET_N broken?).\n");
        goto end;
    }

end:
    return 0;
}
