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
 * @author Mark Greer
 */

#ifndef __INCLUDE_NUTTX_DEVICE_H
#define __INCLUDE_NUTTX_DEVICE_H

#include <nuttx/device_resource.h>
#include <nuttx/ring_buf.h>

enum device_state {
    DEVICE_STATE_REMOVED,
    DEVICE_STATE_PROBING,
    DEVICE_STATE_PROBED,
    DEVICE_STATE_OPENING,
    DEVICE_STATE_OPEN,
    DEVICE_STATE_CLOSING,
    DEVICE_STATE_REMOVING,
};

struct device_pll_type_ops;

struct device_driver_ops {
    int     (*probe)(struct device *dev);
    void    (*remove)(struct device *dev);
    int     (*open)(struct device *dev);
    void    (*close)(struct device *dev);
    union {
        struct device_pll_type_ops     *pll;
    } type_ops;
};

struct device_driver {
    char                        *type;
    char                        *name;
    char                        *desc;
    struct device_driver_ops    *ops;
    void                        *private;
};

struct device {
    char                    *type;
    char                    *name;
    char                    *desc;
    unsigned int            id;
    struct device_resource  *resources;
    unsigned int            resource_count;
    void                    *init_data;
    enum device_state       state;
    struct device_driver    *driver;
    void                    *private;
};

/* Called by device driver clients */
struct device *device_open(char *type, unsigned int id);
void device_close(struct device *dev);

/* Called by device drivers */
int device_register_driver(struct device_driver *driver);
void device_unregister_driver(struct device_driver *driver);

#endif /* __INCLUDE_NUTTX_DEVICE_H */
