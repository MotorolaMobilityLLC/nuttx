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

#include <stddef.h>
#include <assert.h>
#include <nuttx/ring_buf.h>

enum device_resource_type {
    DEVICE_RESOURCE_TYPE_INVALID,
    DEVICE_RESOURCE_TYPE_REGS,
    DEVICE_RESOURCE_TYPE_IRQ,
    DEVICE_RESOURCE_TYPE_GPIO,
    DEVICE_RESOURCE_TYPE_I2C_ADDR,
};

struct device_resource {
    char                        *name;
    enum device_resource_type   type;
    uint32_t                    start;
    unsigned int                count;
};

enum device_state {
    DEVICE_STATE_REMOVED,
    DEVICE_STATE_PROBING,
    DEVICE_STATE_PROBED,
    DEVICE_STATE_OPENING,
    DEVICE_STATE_OPEN,
    DEVICE_STATE_CLOSING,
    DEVICE_STATE_REMOVING,
};

struct device;

struct device_driver_ops {
    int     (*probe)(struct device *dev);
    void    (*remove)(struct device *dev);
    int     (*open)(struct device *dev);
    void    (*close)(struct device *dev);
    void    *type_ops;
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

struct device_resource *device_resource_get(struct device *dev,
                                            enum device_resource_type type,
                                            unsigned int num);
struct device_resource *device_resource_get_by_name(
                                                 struct device *dev,
                                                 enum device_resource_type type,
                                                 char *name);
/**
 * @brief Get the type of a device
 * @param dev Device whose type will be returned
 * @return Pointer to the device's type
 */
static inline char *device_get_type(struct device *dev)
{
    return dev->type;
}

/**
 * @brief Get the name of a device
 * @param dev Device whose name will be returned
 * @return Pointer to the device's name
 */
static inline char *device_get_name(struct device *dev)
{
    return dev->name;
}

/**
 * @brief Get the description of a device
 * @param dev Device whose description will be returned
 * @return Pointer to the device's description
 */
static inline char *device_get_desc(struct device *dev)
{
    return dev->desc;
}

/**
 * @brief Get the ID of a device
 * @param dev Device whose ID will be returned
 * @return The device's ID
 */
static inline unsigned int device_get_id(struct device *dev)
{
    return dev->id;
}

/**
 * @brief Get the init_data of a device
 * @param dev Device whose init_data will be returned
 * @return The device's init_data
 */
static inline void *device_get_init_data(struct device *dev)
{
    return dev->init_data;
}

/**
 * @brief Set the init_data of a device
 * @param dev Device whose init_data will be set
 * @return The device's init_data
 */
static inline void device_set_init_data(struct device *dev, void *init_data)
{
    dev->init_data = init_data;
}

/**
 * @brief Get the state of a device
 * @param dev Device whose state will be returned
 * @return The device's state
 */
static inline enum device_state device_get_state(struct device *dev)
{
    return dev->state;
}

/**
 * @brief Return whether the device is open or not
 * @param dev Device whose state will be tested
 * @return 0: Device is not open; 1: Device is open
 */
static inline enum device_state device_is_open(struct device *dev)
{
    return device_get_state(dev) == DEVICE_STATE_OPEN;
}

/**
 * @brief Get the resource count of a device
 * @param dev Device whose resource count will be returned
 * @return The number of resources associated with the device
 */
static inline unsigned int device_get_resource_count(struct device *dev)
{
    return dev->resource_count;
}

/**
 * @brief Get the private data of a device
 * @param dev Device whose private data will be returned
 * @return The device's private data
 */
static inline void *device_get_private(struct device *dev)
{
    return dev->private;
}

/**
 * @brief Set the private data of a device
 * @param dev Device whose private data will be set
 * @param priv Device's private data
 */
static inline void device_set_private(struct device *dev, void *priv)
{
    dev->private = priv;
}

/**
 * @brief Return whether a driver is attached to the device
 * @param dev Device that will be examined
 * @return 0: Driver is not assigned to device; 1: Driver is assigned to device
 */
static inline int device_driver_is_attached(struct device *dev)
{
    return !!dev->driver;
}

/**
 * @brief Get the type of the driver for a device
 * @param dev Device whose driver's type will be returned
 * @return Pointer to the device driver's type
 */
static inline char *device_driver_get_type(struct device *dev)
{
    return dev->driver->type;
}

/**
 * @brief Get the name of the driver for a device
 * @param dev Device whose driver's name will be returned
 * @return Pointer to the device driver's name
 */
static inline char *device_driver_get_name(struct device *dev)
{
    return dev->driver->name;
}

/**
 * @brief Get the description of the driver for a device
 * @param dev Device whose driver's description will be returned
 * @return Pointer to the device driver's description
 */
static inline char *device_driver_get_desc(struct device *dev)
{
    return dev->driver->desc;
}

/**
 * @brief NULL check entire path from '_dev' to '_type'
 *
 * Check the chain of pointers from the dev pointer to
 * the type_ops pointer to ensure none of them are NULL.
 * If one is NULL, ASSERT iff CONFIG_DEBUG is enabled.
 *
 * @param dev Device whose dev structure will be checked
 * @param type Device type
 */
#define DEVICE_DRIVER_ASSERT_OPS(_dev)                              \
    DEBUGASSERT((_dev) && (_dev)->driver && (_dev)->driver->ops &&  \
                (_dev)->driver->ops->type_ops)

/**
 * @brief Get the type ops of the driver for a device
 * @param dev Device whose driver's type ops will be returned
 * @param type Device type
 * @return Pointer to the device driver's type ops
 */
#define DEVICE_DRIVER_GET_OPS(_dev, _type)  \
    ((struct device_##_type##_type_ops *)((_dev)->driver->ops->type_ops))

/**
 * @brief Get the private data of the driver for a device
 * @param dev Device whose driver's private data will be returned
 * @return The device driver's private data
 */
static inline void *device_driver_get_private(struct device *dev)
{
    return dev->driver->private;
}

/**
 * @brief Set the private data of the driver for a device
 * @param dev Device whose driver's private data will be set
 * @param priv Device driver's private data
 */
static inline void device_driver_set_private(struct device *dev, void *priv)
{
    dev->driver->private = priv;
}

/**
 * @brief Get the name of the specified device resource
 * @param dev Device whose specified device resource will be returned
 * @param idx Index of the device resource to be returned
 * @return The device's device resource
 */
static inline char *device_resource_get_name(struct device *dev,
                                             unsigned int idx)
{
    return dev->resources[idx].name;
}

/**
 * @brief Get the type of the specified device resource
 * @param dev Device whose specified device resource will be returned
 * @param idx Index of the device resource to be returned
 * @return The device's device resource
 */
static inline enum device_resource_type device_resource_get_type(
                                        struct device *dev, unsigned int idx)
{
    return dev->resources[idx].type;
}

/**
 * @brief Get the start of the specified device resource
 * @param dev Device whose specified device resource will be returned
 * @param idx Index of the device resource to be returned
 * @return The device's device resource
 */
static inline char *device_resource_get_start(struct device *dev,
                                              unsigned int idx)
{
    return dev->resources[idx].start;
}

/**
 * @brief Get the count of the specified device resource
 * @param dev Device whose specified device resource will be returned
 * @param idx Index of the device resource to be returned
 * @return The device's device resource
 */
static inline char *device_resource_get_count(struct device *dev,
                                              unsigned int idx)
{
    return dev->resources[idx].count;
}

#endif /* __INCLUDE_NUTTX_DEVICE_H */
