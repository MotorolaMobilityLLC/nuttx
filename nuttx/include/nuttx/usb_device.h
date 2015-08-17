#ifndef __NUTTX_USB_DEVICE_H__
#define __NUTTX_USB_DEVICE_H__

#include <stdint.h>
#include <stddef.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <stdlib.h>

#include <nuttx/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_USB_PCD     "usb-pcd"

struct device_usb_pcd_type_ops {
    int (*register_gadget)(struct device *dev, struct usbdevclass_driver_s *driver);
    int (*unregister_gadget)(struct device *dev, struct usbdevclass_driver_s *driver);
};

/**
 * Register the USB Gadget
 *
 * @param dev PCD device
 * @return 0 if successful
 */
static inline int device_usbdev_register_gadget(struct device *dev,
                                                struct usbdevclass_driver_s *driver)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, usb_pcd)->register_gadget) {
        return DEVICE_DRIVER_GET_OPS(dev, usb_pcd)->register_gadget(dev,
                                     driver);
    }

    return -ENOSYS;
}

/**
 * Register the USB Gadget
 *
 * @param dev PCD device
 * @return 0 if successful
 */
static inline int device_usbdev_unregister_gadget(struct device *dev,
                                                  struct usbdevclass_driver_s *driver)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, usb_pcd)->unregister_gadget) {
        return DEVICE_DRIVER_GET_OPS(dev, usb_pcd)->unregister_gadget(dev,
                                                                      driver);
    }

    return -ENOSYS;
}


#endif __NUTTX_USB_DEVICE_H__

