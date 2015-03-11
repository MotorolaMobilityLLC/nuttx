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
 * @brief Low-level device driver interface
 */

#include <errno.h>
#include <string.h>

#include <arch/irq.h>

#include <nuttx/device.h>
#include <nuttx/device_table.h>

struct device *device_open(char *class, unsigned int id)
{
    struct device *dev;
    irqstate_t flags;
    int ret;

    if (!class)
        return NULL;

    flags = irqsave();

    device_table_for_each_dev(dev) {
        if (!strcmp(dev->class, class) && (dev->id == id)) {
            if (dev->state != DEVICE_STATE_PROBED)
                goto err_irqrestore;

            dev->state = DEVICE_STATE_OPENING;

            if (dev->driver->ops->open) {
                irqrestore(flags);
                ret = dev->driver->ops->open(dev);
                flags = irqsave();

                if (ret) {
                    dev->state = DEVICE_STATE_PROBED;
                    goto err_irqrestore;
                }
            }

            dev->state = DEVICE_STATE_OPEN;
            break;
        }
    }

    irqrestore(flags);

    return dev;

err_irqrestore:
    irqrestore(flags);

    return NULL;
}

void device_close(struct device *dev)
{
    irqstate_t flags;

    if (!dev)
        return;

    flags = irqsave();

    if (dev->state != DEVICE_STATE_OPEN) {
        irqrestore(flags);
        return;
    }

    dev->state = DEVICE_STATE_CLOSING;

    if (dev->driver->ops->close) {
        irqrestore(flags);
        dev->driver->ops->close(dev);
        flags = irqsave();
    }

    dev->state = DEVICE_STATE_PROBED;

    irqrestore(flags);
}

int device_register_driver(struct device_driver *driver)
{
    struct device *dev;
    irqstate_t flags;
    int ret;

    if (!driver || !driver->class || !driver->name || !driver->ops)
        return -EINVAL;

    flags = irqsave();

    device_table_for_each_dev(dev) {
        if (!strcmp(dev->class, driver->class) &&
            !strcmp(dev->name, driver->name)) {

            if (dev->state != DEVICE_STATE_REMOVED)
                continue;

            dev->driver = driver;
            dev->state = DEVICE_STATE_PROBING;

            if (driver->ops->probe) {
                irqrestore(flags);
                ret = driver->ops->probe(dev);
                flags = irqsave();

                if (ret) {
                    dev->state = DEVICE_STATE_REMOVED;
                    dev->driver = NULL;
                    continue;
                }
            }

            dev->state = DEVICE_STATE_PROBED;
        }
    }

    irqrestore(flags);

    return 0;
}

void device_unregister_driver(struct device_driver *driver)
{
    struct device *dev;
    irqstate_t flags;

    if (!driver)
        return;

    flags = irqsave();

    device_table_for_each_dev(dev) {
        if (dev->driver == driver) {
            if (dev->state != DEVICE_STATE_PROBED)
                continue;

            dev->state = DEVICE_STATE_REMOVING;

            if (driver->ops->remove) {
                irqrestore(flags);
                driver->ops->remove(dev);
                flags = irqsave();
            }

            dev->driver = NULL;
            dev->state = DEVICE_STATE_REMOVED;
        }
    }

    irqrestore(flags);
}
