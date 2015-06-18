/**
 * Copyright (c) 2014-2015 Google Inc.
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
 * @author: Perry Hung
 */

#ifndef _UNIPRO_H_
#define _UNIPRO_H_

#include <stdlib.h>

#ifdef CONFIG_TSB_CHIP_REV_ES1
#define CPORT_MAX                   (32)
#elif defined(CONFIG_TSB_CHIP_REV_ES2)
#define CPORT_MAX                   (44)
#endif

#define CPORT_BUF_SIZE              (1024)

typedef int (*unipro_send_completion_t)(int status, const void *buf,
                                        void *priv);

struct unipro_driver {
    const char name[32];
    int (*rx_handler)(unsigned int cportid,  // Called in irq context
                      void *data,
                      size_t len);
};

void unipro_init(void);
int unipro_init_cport(unsigned int cportid);
void unipro_info(void);
int unipro_send(unsigned int cportid, const void *buf, size_t len);
int unipro_send_async(unsigned int cportid, const void *buf, size_t len,
                      unipro_send_completion_t callback, void *priv);
int unipro_unpause_rx(unsigned int cportid);
int unipro_attr_read(uint16_t attr,
                     uint32_t *val,
                     uint16_t selector,
                     int peer,
                     uint32_t *result_code);
int unipro_attr_write(uint16_t attr,
                      uint32_t val,
                      uint16_t selector,
                      int peer,
                      uint32_t *result_code);
int unipro_driver_register(struct unipro_driver *drv, unsigned int cportid);
int unipro_driver_unregister(unsigned int cportid);

static inline int unipro_attr_local_read(uint16_t attr,
                                         uint32_t *val,
                                         uint16_t selector,
                                         uint32_t *result_code)
{
    return unipro_attr_read(attr, val, selector, 0, result_code);
}

static inline int unipro_attr_peer_read(uint16_t attr,
                                        uint32_t *val,
                                        uint16_t selector,
                                        uint32_t *result_code)
{
    return unipro_attr_read(attr, val, selector, 1, result_code);
}

static inline int unipro_attr_local_write(uint16_t attr,
                                          uint32_t val,
                                          uint16_t selector,
                                          uint32_t *result_code)
{
    return unipro_attr_write(attr, val, selector, 0, result_code);
}

static inline int unipro_attr_peer_write(uint16_t attr,
                                         uint32_t val,
                                         uint16_t selector,
                                         uint32_t *result_code)
{
    return unipro_attr_write(attr, val, selector, 1, result_code);
}

#endif
