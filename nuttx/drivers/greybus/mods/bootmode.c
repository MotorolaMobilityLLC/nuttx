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

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/bootmode.h>
#include <nuttx/progmem.h>
#include <nuttx/greybus/debug.h>

static inline size_t bootmode_get_barker_page(void)
{
    return up_progmem_npages() - 1;
}

static int bootmode_erase_page(size_t page)
{
    size_t page_size = up_progmem_pagesize(page);
    size_t written;

    /* return of this function is backwards 0 = true */
    if (up_progmem_ispageerased(page) != 0) {
        gb_debug("erasing page %d\n", page);
        written = up_progmem_erasepage(page);
        if (written != page_size) {
            gb_error("error erasing page\n");
            return -EIO;
        }
    }

    return 0;
}

static int bootmode_write_barker(const char *barker)
{
    size_t page = bootmode_get_barker_page();
    size_t addr = up_progmem_getaddress(page);
    size_t written = 0;
    size_t barker_size = strlen(barker);
    int ret;

    ret = bootmode_erase_page(page);
    if (ret) {
        gb_error("failed to erase barker page err = %d\n", ret);
        return ret;
    }

    written = up_progmem_write(addr, barker, barker_size);
    if (written != barker_size) {
        gb_error("error writing flash\n");
        return -EIO;
    }

    return 0;
}

static int bootmode_erase_barker(void)
{
    size_t page = bootmode_get_barker_page();

    return bootmode_erase_page(page);
}

int gb_bootmode_set(enum bootmode bm)
{
    int ret;

    switch(bm) {
    case BOOTMODE_REQUEST_FLASH:
        ret = bootmode_write_barker("BOOTMODE");
        break;
    case BOOTMODE_FLASHING:
        ret = bootmode_write_barker("FLASHING");
        break;
    case BOOTMODE_NORMAL:
    default:
        ret = bootmode_erase_barker();
        break;
    }

    return ret;
}

