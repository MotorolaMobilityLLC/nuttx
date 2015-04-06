#ifndef _APB_ES1_H_
#define _APB_ES1_H_

#include <sys/types.h>

struct apbridge_dev_s;

struct apbridge_usb_driver
{
  int (*usb_to_svc)(struct apbridge_dev_s *dev, void *payload, size_t size);
  int (*usb_to_unipro)(struct apbridge_dev_s *dev, void *payload, size_t size);
  int (*init)(struct apbridge_dev_s *dev);
};

int unipro_to_usb(struct apbridge_dev_s *dev, void *payload, size_t size);
int svc_to_usb(struct apbridge_dev_s *dev, void *payload, size_t len);

void usb_wait(struct apbridge_dev_s *dev);
int usbdev_apbinitialize(struct apbridge_usb_driver *driver);

#endif /* _APB_ES1_H_ */
