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
#include <nuttx/util.h>
#include <arch/byteorder.h>

#include <stdio.h>

#define USB_SPEED_HIGH 3

#define SET_PORT_FEATURE 0x2303
#define GET_PORT_STATUS  0xa300

#define PORT_POWERING_DELAY_IN_MS   1000
#define PORT_RESETING_DELAY_IN_MS   1000

#define PORT_RESET      0x4
#define PORT_POWER      0x8

#define HS_DEVICE_VENDOR_ID  0x0424
#define HS_DEVICE_PRODUCT_ID 0x3813

#define DEVICE_DESCRIPTOR (1 << 8)

#define GET_DESCRIPTOR        0x8006
#define SET_ADDRESS           0x0005
#define SET_CONFIGURATION     0x0009
#define SET_FEATURE           0x0003

#define ROOT_HUB_PORT           1

#define USB4624_CONFIG_ID       1
#define USB4624_ADDRESS         2
#define USB4624_EXTERNAL_PORT   3

#define URB_MAX_PACKET  0x40

struct device_descriptor {
    uint8_t unused[8];
    uint16_t idVendor;
    uint16_t idProduct;
};

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

static void usb_control_transfer_complete(struct urb *urb)
{
    DEBUGASSERT(urb);
    sem_post(&urb->semaphore);
}

static int usb_control_transfer(struct device *dev, int direction, int speed,
                                int addr, int ttport, uint16_t typeReq,
                                uint16_t wValue, uint16_t wIndex, char *buf,
                                uint16_t wLength)
{
    int retval;
    struct urb *urb;

    DEBUGASSERT(dev);

    urb = urb_create();
    if (!urb) {
        return -ENOMEM;
    }

    urb->complete = usb_control_transfer_complete;
    urb->pipe.type = USB_HOST_PIPE_CONTROL;
    urb->pipe.device = addr;
    urb->pipe.direction = direction;
    urb->dev_speed = speed;
    urb->devnum = addr;
    urb->dev_ttport = ttport;
    urb->length = wLength;
    urb->maxpacket = URB_MAX_PACKET;
    urb->buffer = buf;

    urb->setup_packet[0] = typeReq >> 8,
    urb->setup_packet[1] = typeReq & 0xff,
    urb->setup_packet[2] = wValue & 0xff;
    urb->setup_packet[3] = wValue >> 8;
    urb->setup_packet[4] = wIndex & 0xff;
    urb->setup_packet[5] = wIndex >> 8;
    urb->setup_packet[6] = wLength & 0xff;
    urb->setup_packet[7] = wLength >> 8;

    retval = device_usb_hcd_urb_enqueue(dev, urb);
    if (retval) {
        goto out;
    }

    sem_wait(&urb->semaphore);
    retval = urb->status;

out:
    urb_destroy(urb);
    return retval;
}

static int test_hs_transfer(void)
{
    int retval;
    char buf[255];
    struct device *dev;
    struct device_descriptor *desc = (struct device_descriptor*) buf;

    dev = device_open(DEVICE_TYPE_USB_HCD, 0);
    if (!dev) {
        return -ENODEV;
    }

    retval = device_usb_hcd_start(dev);
    if (retval) {
        goto out;
    }

    /*
     * Enable port 1 on root hub
     */
    device_usb_hcd_hub_control(dev, SET_PORT_FEATURE, PORT_POWER, ROOT_HUB_PORT,
                               NULL, 0);
    up_mdelay(PORT_POWERING_DELAY_IN_MS);

    device_usb_hcd_hub_control(dev, SET_PORT_FEATURE, PORT_RESET, ROOT_HUB_PORT,
                               NULL, 0);
    up_mdelay(PORT_RESETING_DELAY_IN_MS);

    /*
     * Configure usb4624 hub
     */
    retval = usb_control_transfer(dev, USB_HOST_DIR_OUT, USB_SPEED_HIGH, 0, 0,
                                  SET_ADDRESS, USB4624_ADDRESS, 0, NULL, 0);
    if (retval) {
        goto out;
    }

    retval = usb_control_transfer(dev, USB_HOST_DIR_OUT, USB_SPEED_HIGH,
                                  USB4624_ADDRESS, 0, SET_CONFIGURATION,
                                  USB4624_CONFIG_ID, 0, NULL, 0);
    if (retval) {
        goto out;
    }

    /*
     * Enable port 3 on usb4624 hub
     */
    retval = usb_control_transfer(dev, USB_HOST_DIR_OUT, USB_SPEED_HIGH,
                                  USB4624_ADDRESS, 0, SET_PORT_FEATURE,
                                  PORT_POWER, USB4624_EXTERNAL_PORT, NULL, 0);
    if (retval) {
        goto out;
    }

    up_mdelay(PORT_POWERING_DELAY_IN_MS);

    retval = usb_control_transfer(dev, USB_HOST_DIR_OUT, USB_SPEED_HIGH,
                                  USB4624_ADDRESS, 0, SET_PORT_FEATURE,
                                  PORT_RESET, USB4624_EXTERNAL_PORT, NULL, 0);
    if (retval) {
        goto out;
    }

    up_mdelay(PORT_RESETING_DELAY_IN_MS);

    /*
     * Get USB3813 device descriptor
     */
    retval = usb_control_transfer(dev, USB_HOST_DIR_IN, USB_SPEED_HIGH, 0, 0,
                                  GET_DESCRIPTOR, DEVICE_DESCRIPTOR, 0,
                                  buf, ARRAY_SIZE(buf));
    if (retval) {
        goto out;
    }

    if (le16_to_cpu(desc->idVendor) != HS_DEVICE_VENDOR_ID ||
        le16_to_cpu(desc->idProduct) != HS_DEVICE_PRODUCT_ID) {
        retval = -ENODEV;
    }

out:
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

    retval = test_hs_transfer();
    print_test_result("High-Speed transfer", !retval);
    if (retval) {
        fprintf(stderr, "ERROR: Cannot get descriptor from USB3813 Hub.\n");
        fprintf(stderr, "Did you plug the usb cable to APBridgeA usb port?\n");
        goto end;
    }

end:
    return 0;
}
