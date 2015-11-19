/*
 * Copyright (c) 2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>

#include <nuttx/greybus/tsb_unipro.h>
#include <apps/greybus-utils/utils.h>
#include <apps/ara/service_mgr.h>
#include <apps/ara/gb_loopback.h>
#include <apps/nsh.h>

static struct srvmgr_service services[] = {
#if defined(CONFIG_ARA_GB_LOOPBACK)
    {
        .name = "gb_loopback",
        .func = gb_loopback_service,
    },
#endif
    { NULL, NULL }
};

static sem_t linkup_sem;

static void xpb_unipro_evt_handler(enum unipro_event evt)
{
    switch (evt) {
    case UNIPRO_EVT_LUP_DONE:
        sem_post(&linkup_sem);
        break;

    default:
        break;
    }
}

int bridge_main(int argc, char *argv[])
{
    sem_init(&linkup_sem, 0, 0);

    enable_manifest("IID-1", NULL, 0);
    unipro_set_event_handler(xpb_unipro_evt_handler);
    gb_unipro_init();
    srvmgr_start(services);
    enable_cports();
    sem_wait(&linkup_sem);
    tsb_unipro_set_init_status(INIT_STATUS_OPERATING);
    tsb_unipro_mbox_send(TSB_MAIL_READY_OTHER);
    sem_destroy(&linkup_sem);

#ifdef CONFIG_EXAMPLES_NSH
    printf("Calling NSH\n");
    return nsh_main(argc, argv);
#else
    return 0;
#endif
}

