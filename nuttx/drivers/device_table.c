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

#include <errno.h>

#include <nuttx/device.h>
#include <nuttx/device_table.h>

/* Only valid to have one device table per system */
static struct device *device_table;
static unsigned int device_table_entries;

/**
 * @brief Search device table for device by index
 * @param idx Device table index
 * @return Address of structure representing device or NULL on failure
 */
struct device *device_table_get_dev(unsigned int idx)
{
    if (idx >= device_table_entries)
        return NULL;

    return &device_table[idx];
}

/**
 * @brief Get next device table entry
 * @param dev Current device table entry
 * @return Address of next device table entry or NULL if no more
 */
struct device *device_table_get_next_dev(struct device *dev)
{
    if ((++dev - device_table) >= device_table_entries)
        return NULL;

    return dev;
}

/**
 * @brief Register a device table
 * @param table Device table to be registered
 * @param entries Number of device table entries
 * @return 0: Device table registered
 *         -EINVAL: If 'table' is NULL or 'entries' is zero
 */
int device_table_register(struct device *table, unsigned int entries)
{
    if (!table || !entries)
        return -EINVAL;

    device_table = table;
    device_table_entries = entries;

    return 0;
}

/**
 * @brief unregister a device table
 */
void device_table_unregister(void)
{
    device_table = NULL;
    device_table_entries = 0;
}
