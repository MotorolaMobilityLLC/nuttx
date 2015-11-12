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

/*
 * @file    apps/ara/springpm/springpm_app.c
 * @brief   Spring Power Measurement SVC Application
 * @author  Patrick Titiano
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <up_spring_pm.h>

#define DEFAULT_REFRESH_RATE        500000 /* 500ms */
#define DEFAULT_LOOPCOUNT           1
#define DEFAULT_CONTINUOUS          0

/* #define DEBUG_SPRINGPM */

#undef dbg_verbose
#ifdef DEBUG_SPRINGPM
#define dbg_verbose printf
#else
#define dbg_verbose(format, ...)
#endif

static uint32_t refresh_rate = DEFAULT_REFRESH_RATE;
static uint32_t loopcount = DEFAULT_LOOPCOUNT;
static uint8_t continuous = DEFAULT_CONTINUOUS;
static bool csv_export = false;
static uint8_t user_spring_id = 0;
static uint8_t avg_count = ADC_DEFAULT_AVG_SAMPLE_COUNT;
static int32_t *measurements_uA;
static uint32_t timestamp = 0;
static uint8_t spring_start, spring_end;
static char separator[128];

/**
 * @brief           Display application help message
 */
static void springpm_main_usage(void)
{
            printf("Usage: springpm [-s spring] [-n avg_count] [-u refresh_rate] [-l loop] [-c] [-h]\n");
            printf("         -s: select Spring (1-%u) (default: all).\n",
                   spwrm_get_count());
            printf("         -n: select number of averaging samples (max: %u, default: %u).\n",
                   ADC_MAX_AVG_SAMPLE_COUNT, ADC_DEFAULT_AVG_SAMPLE_COUNT);
            printf("         -u: refresh rate in milliseconds (default: %ums).\n",
                   DEFAULT_REFRESH_RATE / 1000);
            printf("         -l: select number of power measurements (default: 1).\n");
            printf("         -c: select continuous power measurements mode (default: disabled).\n");
            printf("         -x: export power measurements as .csv trace instead of table.\n");
            printf("         -h: print help.\n\n");
}

/**
 * @brief           Convert user argument into spring ID.
 * @return          Spring ID (1..spwrm_get_count()), 0 in case of error
 * @param[out]      optarg: command line option to decode
 */
static uint8_t springpm_main_optarg_to_spring_id(const char *optarg)
{
    int ret;
    uint8_t id;

    if (!optarg) {
        return 0;
    }

    ret = sscanf(optarg, "%u", &id);
    if ((ret != 1) || (id == 0) || (id > spwrm_get_count())) {
        return 0;
    }

    return id;
}

/**
 * @brief           Convert user argument into averaging count.
 * @return          averaging count (1..ADC_MAX_AVG_SAMPLE_COUNT),
 *                  0 in case of error
 * @param[out]      optarg: command line option to decode
 */
static uint8_t pwrm_main_optarg_to_avg_count(const char *optarg)
{
    int ret;
    uint8_t count;

    if (!optarg) {
        return 0;
    }

    ret = sscanf(optarg, "%u", &count);
    if ((ret != 1) || (count == 0) || (count > ADC_MAX_AVG_SAMPLE_COUNT)) {
        return 0;
    }

    return count;
}

/**
 * @brief           Retrieve command-line user options.
 * @return          0 on success, -EINVAL otherwise
 * @param[in]       argc: user arguments count
 * @param[in]       argv: user arguments
 */
static int pwrm_main_get_user_options(int argc, char **argv)
{
    char c;
    int ret = 0;

    /*
     * As Nuttx apps are not really apps, (it is allocated statically),
     * make sure all static variables are initialized each time
     * the application is launched.
     */
    refresh_rate = DEFAULT_REFRESH_RATE;
    loopcount = DEFAULT_LOOPCOUNT;
    continuous = DEFAULT_CONTINUOUS;
    user_spring_id = 0;
    avg_count = ADC_DEFAULT_AVG_SAMPLE_COUNT;
    csv_export = false;

    dbg_verbose("%s(): retrieving user options...\n", __func__);
    optind = -1;
    while ((c = getopt(argc, argv, "xhcs:n:u:l:")) != 255) {
        switch (c) {
        case 's':
            user_spring_id = springpm_main_optarg_to_spring_id(optarg);
            if (user_spring_id == 0) {
                fprintf(stderr, "Invalid spring! (%s)\n", optarg);
                return -EINVAL;
            } else {
                dbg_verbose("%s => spring=%u.\n", optarg, user_spring_id);
            }
            break;

        case 'n':
            avg_count = pwrm_main_optarg_to_avg_count(optarg);
            if (avg_count == 0) {
                avg_count = ADC_DEFAULT_AVG_SAMPLE_COUNT;
                fprintf(stderr, "Invalid averaging sample count (%s)! Using default %u.\n",
                        optarg, ADC_DEFAULT_AVG_SAMPLE_COUNT);
            } else {
                dbg_verbose("%s => avg_count=%u.\n", optarg, avg_count);
            }
            break;

        case 'c':
            continuous = 1;
            printf("Continuous monitoring mode enabled.\n");
            break;

        case 'u':
            ret = sscanf(optarg, "%u", &refresh_rate);
            if (ret != 1) {
                refresh_rate = DEFAULT_REFRESH_RATE;
                fprintf(stderr,
                        "Invalid display refresh rate (%s)! Using default %ums.\n",
                        optarg, refresh_rate / 1000);
            } else {
                refresh_rate *= 1000;
                dbg_verbose("Using %uus sampling rate.\n", refresh_rate);
            }
            break;

        case 'l':
            ret = sscanf(optarg, "%u", &loopcount);
            if (ret != 1) {
                loopcount = DEFAULT_LOOPCOUNT;
                fprintf(stderr,
                        "Invalid argument (%s). Use default %u.\n",
                        optarg, loopcount);
            } else {
                dbg_verbose("Looping %u times.\n", loopcount);
            }
            break;

        case 'x':
            csv_export = true;
            printf("Using .csv format to display power measurements.\n");
            break;

        case 'h':
        default:
            return -EINVAL;
        }
    }

    dbg_verbose("%s(): done\n", __func__);
    return 0;
}

/**
 * @brief           Collect current measurements of user-selected spring(s).
 * @return          0 on success, standard error codes otherwise
 */
static int springpm_main_collect_measurements(void)
{
    int ret;
    uint8_t spring;

    for (spring = spring_start; spring < spring_end; spring++) {
        /* Init ADC and GPIO */
        ret = spwrm_init(spring, avg_count);
        if (ret) {
            fprintf(stderr, "%s(): failed to init spring #%u!!! (%d)\n",
                    __func__, spring);
            spwrm_deinit(spring);
            break;
        }
        /* Do the measurement */
        ret = spwrm_get_current(spring, &measurements_uA[spring]);
        if (ret) {
            fprintf(stderr, "failed to get %s current measurement! (%d)\n",
                    __func__, spwrm_get_name(spring), ret);
            spwrm_deinit(spring);
            break;
        }
        dbg_verbose("%s(): %s current measurement = %duA\n", __func__,
                    spwrm_get_name(spring), measurements_uA[spring]);
        /* Deinit HW */
        spwrm_deinit(spring);
    }

    return ret;
}

/**
 * @brief           Draw a table in console displaying all measurements.
 */
static void springpm_main_display_measurements(void)
{
    uint8_t spring;

    if ((csv_export) && (timestamp == 0)) {
        /* Display .csv trace header */
        if (user_spring_id == 0) {
            printf("%-14s", "Timestamp (ms)");
            /* Loop through all springs */
            for (spring = spring_start; spring < spring_end; spring++) {
                printf(", %-8s (mA)", spwrm_get_name(spring));
            }
            printf("\n");
        } else {
            printf("%-14s, %-8s (mA)\n",
                   "Timestamp (ms)", spwrm_get_name(user_spring_id));
        }
    }

    if (user_spring_id == 0) {
        /* All springs selected for measurements */
        if (!csv_export) {
            /* Draw table header in console */
            printf("%s\n", separator);
            printf("| Spring   |     Current      |\n", separator);
            printf("|          | Consumption (mA) |\n", separator);
            printf("%s\n", separator);

            /* Loop through all springs */
            for (spring = spring_start; spring < spring_end; spring++) {
                /* Print result */
                printf("| %-8s |     %-12d |\n", spwrm_get_name(spring),
                       measurements_uA[spring] / 1000);
            }
            printf("%s\n\n", separator);
        } else {
            /* Print result in .csv format */
            /* Loop through all springs */
            printf("%14u", timestamp / 1000);
            for (spring = spring_start; spring < spring_end; spring++) {
                printf(", %13d", measurements_uA[spring] / 1000);
            }
            printf("\n");
        }
    } else {
        if (!csv_export) {
            /* Print result */
            printf("%d\n", measurements_uA[user_spring_id] / 1000);
        } else {
            /* Print result in .csv format */
            printf("%14u, %13d\n", timestamp / 1000,
                   measurements_uA[user_spring_id] / 1000);
        }
    }
}

/**
 * @brief           Application main entry point.
 * @return          0 on success, standard error codes otherwise
 * @param[in]       argc: user arguments count
 * @param[in]       argv: user arguments
 */
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int springpm_main(int argc, char *argv[])
#endif
{
    int ret;

    /* Retrieve user options */
    ret = pwrm_main_get_user_options(argc, argv);
    if (ret) {
        springpm_main_usage();
        exit(-EINVAL);
    }

    /* Init global variables accordingly */
    timestamp = 0;
    measurements_uA = (int32_t *) malloc((spwrm_get_count() + 1) * sizeof(int32_t));
    if (!measurements_uA) {
        fprintf(stderr, "Failed to allocate memory!\n\n");
        return -ENOMEM;
    }
    if (user_spring_id == 0) {
        /* All spring selected for measurements */
        spring_start = 1;
        spring_end = spwrm_get_count() + 1;
        sprintf(separator, "-------------------------------");
        printf("\nGetting Spring power measurement(s)...\n\n");
    } else {
        /* Single Spring Measurement */
        spring_start = user_spring_id;
        spring_end = user_spring_id + 1;
    }

    /* Clear terminal only in case of periodic measurements */
    if ((!csv_export) && ((continuous) || (loopcount > 1))) {
        printf("\033[2J\033[1;1H");
    }

    do {
        ret = springpm_main_collect_measurements();
        if (ret) {
            break;
        }
        springpm_main_display_measurements();
        timestamp += refresh_rate;
        if (!continuous) {
            loopcount--;
        }
        if (loopcount != 0) {
            usleep(refresh_rate);
            if (!csv_export) {
                /* Clear terminal */
                printf("\033[2J\033[1;1H");
            }
        }
    } while (loopcount != 0);

    free(measurements_uA);
    return ret;
}
