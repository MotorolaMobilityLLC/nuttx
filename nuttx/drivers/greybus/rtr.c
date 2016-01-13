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

#include <assert.h>
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

#define SUB_TABLE(a, lvl) ((a >> (lvl * BITS_PER_LEVEL)) & LEVEL_MASK)

#define HDR_ENTRIES 1  /* one entry for parent pointer */
#define HDR_SIZE (HDR_ENTRIES * sizeof(void *))
#define HDR_NDX    -1

static inline void **_rtr_get_parent(void *tbl[])
{
    return tbl[-HDR_ENTRIES];
}

static void *_rtr_alloc_table(void *parent[])
{
    void **child = zalloc(TABLE_SIZE + HDR_SIZE);
    void *rv;

    child[0] = parent;
    rv = &child[HDR_ENTRIES];
    gb_debug("%s ALLOC %p -> %p %p\n", __func__, parent, child, rv);
    return rv;
}

void *rtr_alloc_table(void)
{
    return _rtr_alloc_table(NULL);
}

static void _rtr_free_table(void *tbl[])
{
    gb_debug("%s FREE %p %p\n", __func__, tbl, &tbl[-HDR_ENTRIES]);
    free(&tbl[-HDR_ENTRIES]);
}

void rtr_free_table(void *tbl)
{
    _rtr_free_table((void **)tbl);
}

static void _add_value(void *tbl[], uint16_t a, int lvl, void *data)
{
    int a3 = SUB_TABLE(a, lvl);

    if (lvl == 0) {
            tbl[a3] = data;
    } else {
        if (tbl[a3] == NULL) {
            tbl[a3] = _rtr_alloc_table(tbl);
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
        _rtr_free_table(tbl);
}

void rtr_remove_value(void *tbl[], uint16_t a)
{
    gb_debug("%s(%p, 0x%04x)\n", __func__, tbl, a);
    _remove_value(tbl, a, LEVELS - 1);
}

/*
 * tbl    : tbl to search
 * lvl   : level countdown
 * a     : requested value
 * ndx   : offset in table
 * returns the pointer to the table containing the value.  The
 * value then is retrieved using  the ndx returned;
 */
static void **_get_value(void *tbl[], int lvl, uint16_t a, uint16_t *ndx)
{
    int a3 = SUB_TABLE(a, lvl);

    if (tbl[a3] == NULL) {
        gb_debug("NOT FOUND\n");
        return NULL;
    }

    if (lvl == 0) {
        *ndx = a3;
        return tbl;
    } else {
        return _get_value(tbl[a3], lvl - 1, a, ndx);
    }
}

void *rtr_get_value(void *tbl[], uint16_t a)
{
    uint16_t ndx;
    void **l0_tbl;

    l0_tbl = _get_value(tbl, LEVELS - 1, a, &ndx);
    if (l0_tbl)
        return l0_tbl[ndx];

    return NULL;
}

/* return the first entry greater than or equal to the ndx given */
/* ndx: value representing the index into the table              */
static void *_rtr_get_first_tbl_entry(void **tbl, uint16_t ndx)
{
    int i;

    for (i = ndx; i < TABLE_ENTRIES; i++) {
        if (tbl[i])
            return tbl[i];
    }

    return NULL;
}

/* find the given pointer in the table */
static int _rtr_find_ndx(void **tbl, void *p)
{
    int i;

    for (i = 0; i < TABLE_ENTRIES; i++)
        if (tbl[i] == p)
            return i;

    return -1;
}

static void *_rtr_get_next_value(void *tbl, int lvl, int ndx)
{
    void *rv = NULL;
    void **parent = NULL;
    int parent_ndx;

    if (tbl == NULL) {
        /* we walked off the top - nothing found */
        return NULL;
    }

    rv = _rtr_get_first_tbl_entry(tbl, ndx + 1);
    if (rv) {
        if (lvl == 0) {
            /* we are at the bottom level and found an entry */
            /* we are done */
            return rv;
        } else {
            /* head back down the tree */
            return _rtr_get_next_value(rv, lvl - 1, -1);
        }
    }

    /* up one level, find our parent and index in that table */
    parent = _rtr_get_parent(tbl);
    if (!parent) {
        /* can't go any higher */
        return NULL;
    }
    parent_ndx = _rtr_find_ndx(parent, tbl);
    assert(parent_ndx >= 0);

    return _rtr_get_next_value(parent, lvl + 1, parent_ndx);
}

void *rtr_get_next_value(void *tbl, uint16_t last_value)
{
    uint16_t ndx;

    /* find the last_value table */
    void **last_tbl = _get_value(tbl, LEVELS - 1, last_value, &ndx);
    if (last_tbl) {
        return _rtr_get_next_value(last_tbl, 0, ndx);
    }

    return NULL;
}

static void *_rtr_get_first_value(void **tbl, int lvl)
{
    int i;

    gb_debug("%s %p %d\n", __func__, tbl, lvl);
    if (lvl == 0) {
        for (i = 0; i < TABLE_ENTRIES; i++) {
            if (tbl[i]) {
                gb_debug("found value at ndx %d\n", i);
                return tbl[i];
            }

        }
    } else {
        for (i = 0; i < TABLE_ENTRIES; i++) {
            if (tbl[i] != NULL) {
                gb_debug("found value at ndx %d\n", i);
                return _rtr_get_first_value(tbl[i], lvl - 1);
            }
        }
    }

    return NULL;
}

void *rtr_get_first_value(void *tbl)
{
    return _rtr_get_first_value((void **)tbl, LEVELS - 1);
}

#ifdef CONFIG_GREYBUS_DEBUG
static void _dump_tbls(void *tbl[], int lvl, int p)
{
    int i;
    uint16_t full_address;

    if (lvl == 0) {
        gb_debug("DUMP: self %p parent %p\n", tbl, _rtr_get_parent(tbl));
        for (i = 0; i < TABLE_ENTRIES; i++) {
            full_address = i | (p << BITS_PER_LEVEL);
            gb_debug("DUMP: %d 0x%08x 0x%08x %p\n", lvl, p, full_address, tbl[i]);
        }
    } else {
        gb_debug("DUMP: self %p parent %p\n", tbl, tbl[-1]);
        for (i = 0; i < TABLE_ENTRIES; i++) {
            full_address = i | (p << BITS_PER_LEVEL);
            gb_debug("DUMP: %d 0x%08x 0x%08x %p\n", lvl, p, full_address, tbl[i]);
        }
        for (i = 0; i < TABLE_ENTRIES; i++) {
            if (tbl[i] != NULL) {
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
