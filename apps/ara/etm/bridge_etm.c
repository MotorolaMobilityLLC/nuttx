/**
 * Copyright (c) 2014-2015 Google Inc.
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

#include <nuttx/config.h>
#include <arch/tsb/chip.h>

#include <sys/types.h>
#include "tsb_scm.h"

#include "up_arch.h"

#include <stdio.h>
#include <stdlib.h>

#define DRIVE_MA_MIN         2
#define DRIVE_MA_DEFAULT     4
#define DRIVE_MA_MAX         8

#define TPIU_CPSR            0xE0040004
#define TPIU_SPPR            0xE00400F0

#define ETMCR                0xE0041000
#define ETMTRIGGER           0xE0041008

#define ETMTEEVR             0xE0041020
#define ETMTRACEIDR          0xE0041200
#define ETMLAR               0xE0041FB0
#define ETMLSR               0xE0041FB4

#define ETM_UNLOCK           0xC5ACCE55 /* Magic value for ETMLAR to unlock ETM access */
#define ETM_LOCK             0x0        /* Less magic lock value for ETMLAR */

#define ETMLSR_LOCKED        0x2        /* Bit is set in ETMLSR when ETM locked */

#define ETMCR_PROGRAM        0x00022C40 /* ETM is awake and in programming mode */
#define ETMCR_OPERATE        0x00022840 /* ETM is awake and in operational mode */

#define ETM_EVENT_ALWAYS     0x6F       /* ETM decodes value as ALWAYS */
#define ETM_EVENT_NEVER      0x406F     /* ETM decodes value as !ALWAYS */

static struct {
    int enabled;
    unsigned short drive_ma;
    unsigned short drive_ma_save;
    uint32_t etm_pinshare_save;
    uint32_t cpsr_save;
    uint32_t sppr_save;
    uint32_t cr_save;
    uint32_t trigger_save;
    uint32_t teevr_save;
    uint32_t traceidr_save;
    uint32_t lsr_save;
} etm = {.drive_ma = DRIVE_MA_DEFAULT};

/* set drive strength for the TRACE lines to the specified value */
void set_trace_drive_ma(unsigned short drive_ma) {
    switch(drive_ma) {
    case DRIVE_MA_MIN:
        tsb_set_drivestrength(TSB_TRACE_DRIVESTRENGTH, tsb_ds_min);
        break;
    case DRIVE_MA_DEFAULT:
        tsb_set_drivestrength(TSB_TRACE_DRIVESTRENGTH, tsb_ds_default);
        break;
    case DRIVE_MA_MAX:
        tsb_set_drivestrength(TSB_TRACE_DRIVESTRENGTH, tsb_ds_max);
        break;
    default:
        printf("Provided drive strength value of %u milliamp is invalid\n", drive_ma);
        break;
    }
}

/* get the currently configured drive strength for the TRACE lines */
unsigned short get_trace_drive_ma(void) {
    switch(tsb_get_drivestrength(TSB_TRACE_DRIVESTRENGTH)) {
    case tsb_ds_min:
        return DRIVE_MA_MIN;
        break;
    case tsb_ds_default:
        return DRIVE_MA_DEFAULT;
        break;
    case tsb_ds_max:
        return DRIVE_MA_MAX;
        break;
    default:
        printf("Unable to decode TRACE drive strength\n");
        return 0xFFFF;
    }
}

static void print_usage(void) {
    printf("ETM: Usage:\n");
    printf("ETM:  etm e [2|4|8]    : enable TRACE output lines, with optional drive milliamps\n");
    printf("ETM:  etm d            : return TRACE output lines to their orignal state\n");
    printf("ETM:  etm s            : display state of TRACE output lines\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[]) {
#else
    int etm_main(int argc, char *argv[]) {
#endif
    char cmd;
    long drive_ma;
    unsigned short last_drive_ma = etm.drive_ma;
#ifndef CONFIG_TSB_PINSHARE_ETM
    int retval;
#endif

    if (argc < 2) {
        print_usage();
        return EXIT_FAILURE;
    } else {
        cmd = argv[1][0];
    }

    switch (cmd) {
    case 'h':
    case '?':
        print_usage();
        break;
    case 'e':
        /* enable ETM */
        if (argc == 2) {
            etm.drive_ma = DRIVE_MA_MAX;
        } else if (argc == 3) {
            drive_ma = strtol(argv[2], NULL, 10);
            if (drive_ma != DRIVE_MA_MIN &&
                drive_ma != DRIVE_MA_DEFAULT &&
                drive_ma != DRIVE_MA_MAX) {
                printf("Invalid drive strength of %ld ma when trying to enable ETM (%u|%u|%u are valid)\n",
                       drive_ma, DRIVE_MA_MIN, DRIVE_MA_DEFAULT, DRIVE_MA_MAX);
                return EXIT_FAILURE;
            }
            etm.drive_ma = (unsigned short) drive_ma;
        } else {
            printf("Too many arguments specified when attempting to enable ETM\n");
            return EXIT_FAILURE;
        }

        if (etm.enabled) {
            /* update drive strength if it has changed */
            if (etm.drive_ma != last_drive_ma) {
                set_trace_drive_ma(etm.drive_ma);
                printf("ETM drive strength changed to %u milliamps\n", etm.drive_ma);
            } else
                printf("WARNING: ETM is already enabled\n");
        } else {
            /*
             * perhaps we ought to be recording the old value
             * but the present API doesn't expose it
             */
            etm.etm_pinshare_save = tsb_get_pinshare() & TSB_PIN_ETM;
#ifndef CONFIG_TSB_PINSHARE_ETM
            retval = tsb_request_pinshare(TSB_PIN_ETM);
            if (retval) {
                fprintf(stderr, "ETM pin already held by another driver.\n");
                return retval;
            }
#endif

            tsb_set_pinshare(TSB_PIN_ETM);

            /* set drive strength for the TRACE signals to the specified value */
            etm.drive_ma_save = get_trace_drive_ma();
            set_trace_drive_ma(etm.drive_ma);

            /*
             * record current value of the ETM lock status register so that we can restore
             * the original lock state if ETM TRACE is disabled
             */
            etm.lsr_save = getreg32(ETMLSR);

            /* unlock ETM: may need a delay right after */
            putreg32(ETM_UNLOCK, ETMLAR);

            /* put ETM into programming mode: may need a delay right after */
            etm.cr_save = getreg32(ETMCR);
            putreg32(ETMCR_PROGRAM, ETMCR);

            /* Don't define a trigger for TRACE (NOT Always) */
            etm.trigger_save = getreg32(ETMTRIGGER);
            putreg32(ETM_EVENT_NEVER, ETMTRIGGER);

            /* TRACE all memory */
            etm.teevr_save = getreg32(ETMTEEVR);
            putreg32(ETM_EVENT_ALWAYS, ETMTEEVR);

            /* Set identifier for trace output (match what DTRACE selects) */
            etm.traceidr_save = getreg32(ETMTRACEIDR);
            putreg32(0x2, ETMTRACEIDR);

            /* Set trace port width to 4 (versus 1) */
            etm.cpsr_save = getreg32(TPIU_CPSR);
            putreg32(0x8, TPIU_CPSR);

            /* Set trace output type to parallel (versus 1 == serial) */
            etm.sppr_save = getreg32(TPIU_SPPR);
            putreg32(0x0, TPIU_SPPR);

            /* And finally, set ETM to operational mode */
            putreg32(ETMCR_OPERATE, ETMCR);

            etm.enabled = 1;
            printf("ETM enabled with a drive strength of %u milliamps\n", etm.drive_ma);
        }
        break;
    case 'd':
        /* Disable ETM */
        if (!etm.enabled) {
            printf("WARNING: etm is already disabled\n");
        } else {
            /* Put ETM back into programming mode */
            putreg32(ETMCR_PROGRAM, ETMCR);

            /* Restore registers to saved values */
            putreg32(etm.sppr_save, TPIU_SPPR);
            putreg32(etm.cpsr_save, TPIU_CPSR);
            putreg32(etm.traceidr_save, ETMTRACEIDR);
            putreg32(etm.teevr_save, ETMTEEVR);
            putreg32(etm.trigger_save, ETMTRIGGER);
            putreg32(etm.cr_save, ETMCR);

            /* Relock ETM if it was locked when we enabled TRACE */
            if (etm.lsr_save & ETMLSR_LOCKED)
                putreg32(ETM_LOCK, ETMLAR);

            /* Restore the original drive strength */
            set_trace_drive_ma(etm.drive_ma_save);

            /* Clear the ETM pinshare if it wasn't set on entry */
            if (!etm.etm_pinshare_save)
                tsb_clr_pinshare(TSB_PIN_ETM);

#ifndef CONFIG_TSB_PINSHARE_ETM
            tsb_release_pinshare(TSB_PIN_ETM);
#endif

            etm.enabled = 0;
            etm.drive_ma = 0;
        }
        break;
    case 's':
        /* Get the ETM state */
        if (etm.enabled) {
            printf("ETM state = enabled, with a drive strength of %u milliamps\n", etm.drive_ma);
        } else {
            printf("ETM state = either never enabled or returned to original state\n");
        }
        break;
    default:
        printf("ETM: Unknown command\n");
        print_usage();
        return EXIT_FAILURE;
    }

    return 0;
}
