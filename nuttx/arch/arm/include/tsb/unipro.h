/**
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 * @author: Perry Hung
 */

#ifndef _UNIPRO_H_
#define _UNIPRO_H_

#include <stdlib.h>

#define CPORT_MAX                   (44)
#define CPORT_BUF_SIZE              (1024)

struct unipro_driver {
    const char name[32];
    int (*rx_handler)(unsigned int cportid,  // Called in irq context
                      void *data,
                      size_t len);
};

void unipro_init(void);
void unipro_info(void);
int unipro_send(unsigned int cportid, const void *buf, size_t len);
int unipro_attr_read(uint16_t attr,
                     uint32_t *val,
                     uint16_t selector,
                     int peer,
                     uint32_t *result_code);
int unipro_driver_register(struct unipro_driver *drv, unsigned int cportid);

#endif
