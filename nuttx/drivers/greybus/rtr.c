/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/greybus/debug.h>

#include "rtr.h"

/* Number of bits per LEVEL */
#define BITS_PER_LEVEL  4
#define LEVEL_MASK     ((1 << BITS_PER_LEVEL) - 1)
#define TOTAL_BITS     16
#define TABLE_ENTRIES  16 /* 2 ^ BITS_PER_LEVEL */

#define LEVELS (TOTAL_BITS / BITS_PER_LEVEL)
#define TABLE_SIZE (TABLE_ENTRIES * sizeof(void *))

#define SUB_TABLE(a, lvl) ((a >> (lvl * LEVELS)) & LEVEL_MASK)

void *rtr_alloc_table(void)
{
    void *p = zalloc(TABLE_SIZE);
    return p;
}

void rtr_free_table(void **tbl)
{
    gb_debug("%s FREE %p\n", __func__, tbl);
    free(tbl);
}

static void _add_value(void *tbl[], uint16_t a, int lvl, void *data)
{
    int a3 = SUB_TABLE(a, lvl);

    if (lvl == 0) {
            /* TODO: should this be an error if tbl[a3] already has a value */
            tbl[a3] = data;
    } else {
        if (tbl[a3] == NULL) {
            tbl[a3] = rtr_alloc_table();
        }
        _add_value(tbl[a3], a, lvl - 1, data);
    }
}

void rtr_add_value(void *tbl[], uint16_t a, void *data)
{
    gb_debug("%s(%p, 0x%04x, %p)\n", __func__, tbl, a, data);
    _add_value(tbl, a, LEVELS - 1, data);
}

static void _remove_value(void *tbl[], uint16_t a, int lvl)
{
    int a3 = SUB_TABLE(a, lvl);
    int empty = 1;
    int i;

    if (tbl[a3] == NULL)
        return;

    /* remove child level */
    if (lvl != 0)
        _remove_value(tbl[a3], a, lvl - 1);

    /* clear our reference */
    tbl[a3] = NULL;

    /* if this was the last entry, table will be empty - free it */
    for (i = 0; i < TABLE_ENTRIES; i++) {
        if (tbl[i] != NULL) {
            empty = 0;
            break;
        }
    }

    if (empty && (lvl < LEVELS - 1))
        rtr_free_table(tbl);
}

void rtr_remove_value(void *tbl[], uint16_t a)
{
    gb_debug("%s(%p, 0x%04x)\n", __func__, tbl, a);
    _remove_value(tbl, a, LEVELS - 1);
}

static void *_get_value(void *tbl[], int lvl, uint16_t a)
{
    int a3 = SUB_TABLE(a, lvl);

    if (tbl[a3] == NULL) {
        gb_error("NOT FOUND\n");
        return NULL;
    }

    if (lvl == 0) {
        return tbl[a3];
    } else {
        return _get_value(tbl[a3], lvl - 1, a);
    }
}

void *rtr_get_value(void *tbl[], uint16_t a)
{
    return _get_value(tbl, LEVELS - 1, a);
}

#ifdef CONFIG_GREYBUS_DEBUG
static void _dump_tbls(void *tbl[], int lvl, int p)
{
    int i;
    uint16_t full_address;

    if (lvl == 0) {
        for (i = 0; i < TABLE_ENTRIES; i++) {
            full_address = i | (p << BITS_PER_LEVEL);
            if (tbl[i] != NULL)
                gb_debug("DUMP: %d %08x %p\n", lvl, full_address, tbl[i]);
        }
    } else {
        for (i = 0; i < TABLE_ENTRIES; i++) {
            if (tbl[i] != NULL) {
                full_address = i | (p << BITS_PER_LEVEL);
                _dump_tbls(tbl[i], lvl -1, full_address);
            }
        }
    }
}

void rtr_dump_tbls(void *tbl[])
{
    _dump_tbls(tbl, LEVELS - 1, 0);
}
#endif
