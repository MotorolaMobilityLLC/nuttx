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

#include <string.h>

#include <nuttx/device_resource.h>

/**
 * @brief Search for device resource by type and number
 * @param dev Address of structure representing device
 * @param type Resource type to search for
 * @param num Instance of the resource of specified type
 * @return Address of resource or NULL on failure
 */
struct device_resource *device_resource_get(struct device *dev,
                                            enum device_resource_type type,
                                            unsigned int num)
{
    unsigned int i;

    for (i = 0; i < dev->resource_count; i++)
        if ((dev->resources[i].type == type) && (num-- == 0))
            return &dev->resources[i];

    return NULL;
}

/**
 * @brief Search for device resource by type and name
 * @param dev Address of structure representing device
 * @param type Resource type to search for
 * @param name Name of the resource
 * @return Address of resource or NULL on failure
 */
struct device_resource *device_resource_get_by_name(
                                                struct device *dev,
                                                enum device_resource_type type,
                                                char *name)
{
    unsigned int i;

    for (i = 0; i < dev->resource_count; i++)
        if ((dev->resources[i].type == type) &&
            !strcmp(dev->resources[i].name, name))
            return &dev->resources[i];

    return NULL;
}
