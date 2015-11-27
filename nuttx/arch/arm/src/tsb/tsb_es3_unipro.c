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
 */

#include "debug.h"

#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>

#include <errno.h>

int tsb_unipro_set_init_status(uint32_t val)
{
    int retval;
    uint32_t status;

    /*
     * See SW-1228
     *  -> https://projectara.atlassian.net/browse/SW-1228?focusedCommentId=28889&page=com.atlassian.jira.plugin.system.issuetabpanels:comment-tabpanel#comment-28889
     */
    if (val & INIT_STATUS_ERROR_CODE_MASK) {
        lowsyslog("%s() cannot override the init status error code (init status: %u).\n",
                  __func__, val);
        return -EINVAL;
    }

    retval = unipro_attr_local_read(TSB_DME_ES3_INIT_STATUS, &status,
                                    UNIPRO_SELINDEX_NULL);
    if (retval) {
        lowsyslog("init-status read failed: retval = %d\n", retval);
        return retval;
    }

    /* preserve error code: see SW-1228 */
    val |= status & INIT_STATUS_ERROR_CODE_MASK;

    retval = unipro_attr_local_write(TSB_DME_ES3_INIT_STATUS, val,
                                     UNIPRO_SELINDEX_NULL);
    if (retval) {
        lowsyslog("init-status write failed: retval = %d\n", retval);
        return retval;
    }

    return 0;
}
