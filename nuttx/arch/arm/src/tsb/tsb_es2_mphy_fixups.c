/*
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
 */

#include <stdint.h>
#include <errno.h>

#include "tsb_es2_mphy_fixups.h"
#include "tsb_scm.h"
#include "debug.h"

#include <nuttx/unipro/unipro.h>

#define MPHY_FIXUP_RETRIES  3

/*
 * "Map" constants for M-PHY fixups.
 */
#define TSB_MPHY_MAP (0x7F)
    #define TSB_MPHY_MAP_TSB_REGISTER_1 (0x01)
    #define TSB_MPHY_MAP_NORMAL         (0x00)
    #define TSB_MPHY_MAP_TSB_REGISTER_2 (0x81)

#define __TSB_MPHY_FIXUP(a, s, v, f)                                    \
    { .attrid = (a), .select_index = (s), .value = (v), .flags = (f) }
#define TSB_MPHY_FIXUP(a, s, v)                                         \
    __TSB_MPHY_FIXUP((a), (s), (v), 0)
#define TSB_MPHY_LAST_FIXUP(a, s, v)                                    \
    __TSB_MPHY_FIXUP((a), (s), (v), TSB_MPHY_FIXUP_LAST)
#define TSB_MPHY_MAGIC_R1_FIXUP()                                       \
    __TSB_MPHY_FIXUP(0, 0, 0, TSB_MPHY_FIXUP_MAGIC_R1)

const struct tsb_mphy_fixup tsb_register_1_map_mphy_fixups[] = {
    TSB_MPHY_MAGIC_R1_FIXUP(),

    TSB_MPHY_FIXUP(0x8004, 0, 0xCA),
    TSB_MPHY_FIXUP(0x8015, 0, 0x01),
    TSB_MPHY_FIXUP(0x8022, 0, 0x44),
    TSB_MPHY_FIXUP(0x8023, 0, 0x42),
    TSB_MPHY_FIXUP(0x80A2, 0, 0x00),
    TSB_MPHY_FIXUP(0x80AA, 0, 0xA8),
    TSB_MPHY_FIXUP(0x80BA, 0, 0x20),

    TSB_MPHY_FIXUP(0x80A2, 1, 0x00),
    TSB_MPHY_FIXUP(0x80AA, 1, 0xA8),
    TSB_MPHY_FIXUP(0x80BA, 1, 0x20),

    TSB_MPHY_FIXUP(0x8094, 4, 0x09),
    TSB_MPHY_FIXUP(0x809A, 4, 0x06),
    TSB_MPHY_FIXUP(0x809B, 4, 0x03),
    TSB_MPHY_FIXUP(0x809C, 4, 0x00),
    TSB_MPHY_FIXUP(0x80AA, 4, 0x0F),
    TSB_MPHY_FIXUP(0x80B4, 4, 0x50),
    TSB_MPHY_FIXUP(0x80B6, 4, 0x82),
    TSB_MPHY_FIXUP(0x80B7, 4, 0x01),

    TSB_MPHY_FIXUP(0x8094, 5, 0x09),
    TSB_MPHY_FIXUP(0x809A, 5, 0x06),
    TSB_MPHY_FIXUP(0x809B, 5, 0x03),
    TSB_MPHY_FIXUP(0x809C, 5, 0x00),
    TSB_MPHY_FIXUP(0x80AA, 5, 0x0F),
    TSB_MPHY_FIXUP(0x80B4, 5, 0x50),
    TSB_MPHY_FIXUP(0x80B6, 5, 0x82),
    TSB_MPHY_FIXUP(0x80B7, 5, 0x01),

    TSB_MPHY_LAST_FIXUP(0x8000, 0, 0x01),
};

const struct tsb_mphy_fixup tsb_register_2_map_mphy_fixups[] = {
    TSB_MPHY_FIXUP(0x8000, 0, 0x02),

    TSB_MPHY_FIXUP(0x8080, 0, 0x20),
    TSB_MPHY_FIXUP(0x8081, 0, 0x03),

    TSB_MPHY_FIXUP(0x8080, 1, 0x20),
    TSB_MPHY_FIXUP(0x8081, 1, 0x03),

    TSB_MPHY_FIXUP(0x8082, 4, 0x3F),
    TSB_MPHY_FIXUP(0x8084, 4, 0x10),
    TSB_MPHY_FIXUP(0x8086, 4, 0x10),
    TSB_MPHY_FIXUP(0x8087, 4, 0x01),
    TSB_MPHY_FIXUP(0x8088, 4, 0x10),
    TSB_MPHY_FIXUP(0x808D, 4, 0x0B),
    TSB_MPHY_FIXUP(0x808E, 4, 0x00),
    TSB_MPHY_FIXUP(0x8094, 4, 0x00),
    TSB_MPHY_FIXUP(0x8096, 4, 0x00),
    TSB_MPHY_FIXUP(0x8098, 4, 0x08),
    TSB_MPHY_FIXUP(0x8099, 4, 0x50),

    TSB_MPHY_FIXUP(0x8082, 5, 0x3F),
    TSB_MPHY_FIXUP(0x8084, 5, 0x10),
    TSB_MPHY_FIXUP(0x8086, 5, 0x10),
    TSB_MPHY_FIXUP(0x8087, 5, 0x01),
    TSB_MPHY_FIXUP(0x8088, 5, 0x10),
    TSB_MPHY_FIXUP(0x808D, 5, 0x0B),
    TSB_MPHY_FIXUP(0x808E, 5, 0x00),
    TSB_MPHY_FIXUP(0x8094, 5, 0x00),
    TSB_MPHY_FIXUP(0x8096, 5, 0x00),
    TSB_MPHY_FIXUP(0x8098, 5, 0x08),
    TSB_MPHY_LAST_FIXUP(0x8099, 5, 0x50),
};

static int es2_fixup_mphy(void)
{
    uint32_t debug_0720 = tsb_get_debug_reg(0x0720);
    int rc;
    const struct tsb_mphy_fixup *fu;

    /*
     * Apply the "register 2" map fixups.
     */
    rc = unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_TSB_REGISTER_2, 0);
    if (rc) {
        lldbg("%s: failed to switch to register 2 map: %d\n", __func__, rc);
        return rc;
    }
    fu = tsb_register_2_map_mphy_fixups;
    do {
        rc = unipro_attr_local_write(fu->attrid, fu->value, fu->select_index);
        if (rc) {
            lldbg("%s: failed to apply register 1 map fixup: %d\n", __func__,
                  rc);
            return rc;
        }
    } while (!tsb_mphy_fixup_is_last(fu++));

    /*
     * Switch to "normal" map.
     */
    rc = unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_NORMAL, 0);
    if (rc) {
        lldbg("%s: failed to switch to normal map: %d\n", __func__, rc);
        return rc;
    }

    /*
     * Apply the "register 1" map fixups.
     */
    rc = unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_TSB_REGISTER_1, 0);
    if (rc) {
        lldbg("%s: failed to switch to register 1 map: %d\n", __func__, rc);
        return rc;
    }
    fu = tsb_register_1_map_mphy_fixups;
    do {
        if (tsb_mphy_r1_fixup_is_magic(fu)) {
            /* The magic R1 fixups come from the mysterious and solemn
             * debug register 0x0720. */
            rc = unipro_attr_local_write(0x8002, (debug_0720 >> 1) & 0x1f, 0);
        } else {
            rc = unipro_attr_local_write(fu->attrid, fu->value,
                                         fu->select_index);
        }
        if (rc) {
            lldbg("%s: failed to apply register 1 map fixup: %d\n", __func__,
                  rc);
            return rc;
        }
    } while (!tsb_mphy_fixup_is_last(fu++));

    /*
     * Switch to "normal" map.
     */
    rc = unipro_attr_local_write(TSB_MPHY_MAP, TSB_MPHY_MAP_NORMAL, 0);
    if (rc) {
        lldbg("%s: failed to switch to normal map: %d\n", __func__, rc);
        return rc;
    }

    return rc;
}

void es2_apply_mphy_fixup(void)
{
    int i;
    int retval;

    lowsyslog("UniPro: applying TSB ES2 M-PHY fixups: ");

    retval = -EIO;
    for (i = 0; i < MPHY_FIXUP_RETRIES && retval; i++) {
        retval = es2_fixup_mphy();
    }

    lowsyslog(retval ? "FAILED\n" : "DONE\n");
}
