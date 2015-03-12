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
 * @file    apps/ara/bdbpm/bdbpm_app.c
 * @brief   BDB Power Measurement SVC Application
 * @author  Patrick Titiano
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pwr_measure.h>
#include <up_bdb_pm.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

#define DEFAULT_CONVERSION_TIME     ina230_ct_1_1ms /* 1.1ms */
#define DEFAULT_AVG_SAMPLE_COUNT    ina230_avg_count_64 /* 64 samples average */
#define DEFAULT_CURRENT_LSB         100 /* 100uA current LSB */
#define DEFAULT_REFRESH_RATE        500000 /* 500ms */
#define DEFAULT_LOOPCOUNT           1
#define DEFAULT_CONTINUOUS          0

/* #define DEBUG_BDBPM */

#ifdef DEBUG_BDBPM
#define dbg_verbose printf
#else
#define dbg_verbose(format, ...)
#endif

static bdbpm_rail *bdbpm_rails[DEV_COUNT][DEV_MAX_RAIL_COUNT];
static pwr_measure measurements[DEV_COUNT][DEV_MAX_RAIL_COUNT];
static uint32_t refresh_rate = DEFAULT_REFRESH_RATE;
static uint32_t loopcount = DEFAULT_LOOPCOUNT;
static uint8_t continuous = DEFAULT_CONTINUOUS;
static uint8_t user_dev_id = DEV_COUNT;
static uint8_t user_rail_id = DEV_MAX_RAIL_COUNT;
static uint32_t current_lsb;
static ina230_conversion_time conversion_time;
static ina230_avg_count avg_count;
static char separator[512];
static char header[512];

static const char ct_strings[ina230_ct_count + 1][8] = {
    "140us",
    "204us",
    "332us",
    "588us",
    "1.1ms",
    "2.116ms",
    "4.156ms",
    "8.244ms",
    "ERROR"};

/**
 * @brief           Display application help message
 */
static void usage(void)
{
            printf("Usage: bdbpm [-d device] [-r rail] [-i current_lsb] [-t conversion_time] [-g avg_count] [-u refresh_rate] [-l loop] [-c] [-h]\n");
            printf("         -d: select device (SW, APB[1-3], GPB[1-2]) (default: all).\n");
            printf("         -r: select rail (default: all):\n");
            printf("             VSW_1P1_PLL, VSW_1P1_CORE, VSW_1P8_UNIPRO, VSW_1P8_IO\n");
            printf("             VAPB[1-3]_1P1_CORE, VAPB[1-3]_1P1_PLL1, VAPB[1-3]_1P2_CDSI_PLL, VAPB[1-3]_1P2_CDSI, VAPB[1-3]_1P2_HSIC, VAPB[1-3]_1P8_UNIPRO, VAPB[1-3]_1P8_IO, VAPB[1-3]_1P1_PLL2\n");
            printf("             VGPB[1-2]_1P1_CORE, VGPB[1-2]_1P1_PLL1, VGPB[1-2]_SDIO, VGPB[1-2]_1P2_HSIC, VGPB[1-2]_1P8_UNIPRO, VGPB[1-2]_1P8_IO, VGPB[1-2]_1P1_PLL2\n");
            printf("         -i: select current measurement precision (LSB, in uA) (default: 100uA):\n");
            printf("         -t: select conversion time (default: 1.1ms):\n");
            printf("             Available options: 140us, 204us, 332us, 588us, 1.1ms, 2.116ms, 4.156ms, 8.244ms.\n");
            printf("         -n: select number of averaging samples (default: 64).\n");
            printf("             Available options: 1, 4, 16, 64, 128, 256, 512, 1024.\n");

            printf("         -u: display refresh rate in milliseconds (default: 500).\n");
            printf("         -l: select number of power measurements (default: 1).\n");
            printf("         -c: select continuous power measurements mode (default: disabled).\n");
            printf("         -h: print help.\n\n");
}

/**
 * @brief           Return device start and end IDs depending on user options.
 * @return          device start and end IDs
 * @param[out]      d_start: device start ID
 * @param[out]      d_end: device end ID
 */
static void bdbpm_main_get_device_list(uint8_t *d_start, uint8_t *d_end)
{
    if (user_dev_id == DEV_COUNT) {
        *d_start = 0;
        *d_end = DEV_COUNT;
    } else {
        *d_start = user_dev_id;
        *d_end = user_dev_id + 1;
    }
    dbg_verbose("%s(): user_dev_id=%u => d_start=%u d_end=%u\n",
                __func__, user_dev_id, *d_start, *d_end);
}

/**
 * @brief           Return rail start and end IDs depending on user options.
 * @return          rail start and end IDs
 * @param[in]       dev: device ID
 * @param[out]      r_start: power rail start ID
 * @param[out]      r_end: power rail end ID
 */
static void bdbpm_main_get_rail_list(uint8_t dev,
                                    uint8_t *r_start, uint8_t *r_end)
{
    if (user_rail_id == DEV_MAX_RAIL_COUNT) {
        *r_start = 0;
        *r_end = bdbpm_dev_rail_count(dev);
    } else {
        *r_start = user_rail_id;
        *r_end = user_rail_id + 1;
    }
    dbg_verbose("%s(): dev=%u => r_start=%u r_end=%u\n",
                __func__, dev, *r_start, *r_end);
}

/**
 * @brief           Convert user option into conversion time.
 * @return          conversion time
 * @param[in]       optarg: user option
 */
static ina230_conversion_time bdbpm_main_optarg_to_ct(const char *optarg)
{
    if (!optarg) {
        return ina230_ct_count;
    } else if (strcmp(optarg, "140us") == 0) {
        return ina230_ct_140us;
    } else if (strcmp(optarg, "204us") == 0) {
        return ina230_ct_204us;
    } else if (strcmp(optarg, "332us") == 0) {
        return ina230_ct_332us;
    } else if (strcmp(optarg, "588us") == 0) {
        return ina230_ct_588us;
    } else if (strcmp(optarg, "1.1ms") == 0) {
        return ina230_ct_1_1ms;
    } else if (strcmp(optarg, "2.116ms") == 0) {
        return ina230_ct_2_116ms;
    } else if (strcmp(optarg, "4.156ms") == 0) {
        return ina230_ct_4_156ms;
    } else if (strcmp(optarg, "8.244ms") == 0) {
        return ina230_ct_8_244ms;
    } else {
        return ina230_ct_count;
    }
}

/**
 * @brief           Convert conversion time into string.
 * @return          string
 * @param[in]       time: INA230 conversion time
 */
static const char *bdbpm_main_ct_to_string(ina230_conversion_time time)
{
    if (time >=  ina230_ct_count) {
        return ct_strings[ina230_ct_count];
    } else {
        return ct_strings[time];
    }
}

/**
 * @brief           Convert user option into averaging count.
 * @return          averaging count
 * @param[in]       optarg: user option
 */
static ina230_avg_count bdbpm_main_optarg_to_avg_count(const char *optarg)
{
    uint32_t val;

    if (!optarg) {
        return ina230_avg_count_max;
    }

    if (sscanf(optarg, "%u", &val) != 1) {
        return ina230_avg_count_max;
    }

    switch (val) {
    case 1:
        return ina230_avg_count_1;

    case 4:
        return ina230_avg_count_4;

    case 16:
        return ina230_avg_count_16;

    case 64:
        return ina230_avg_count_64;

    case 128:
        return ina230_avg_count_128;

    case 256:
        return ina230_avg_count_256;

    case 512:
        return ina230_avg_count_512;

    case 1024:
        return ina230_avg_count_1024;

    default:
        return ina230_avg_count_max;
    }
}

/**
 * @brief           Retrieve command-line user options.
 * @return          0 on success, -EINVAL otherwise
 * @param[in]       argc: user arguments count
 * @param[in]       argv: user arguments
 */
static int bdbpm_main_get_user_options(int argc, char **argv)
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
    user_dev_id = DEV_COUNT;
    user_rail_id = DEV_MAX_RAIL_COUNT;
    current_lsb = DEFAULT_CURRENT_LSB;
    conversion_time = DEFAULT_CONVERSION_TIME;
    avg_count = DEFAULT_AVG_SAMPLE_COUNT;

    dbg_verbose("%s(): retrieving user options...\n", __func__);
    optind = 1;
    while ((c = getopt(argc, argv, "hcd:r:l:u:i:t:n:")) != 255) {
        switch (c) {
        case 'd':
            ret = bdbpm_device_id(optarg, &user_dev_id);
            if (ret) {
                fprintf(stderr, "Invalid device! (%s)\n", optarg);
                return -EINVAL;
            } else {
                dbg_verbose("%s => dev=%u.\n", optarg, user_dev_id);
                printf("%s device selected.\n", optarg);
            }
            break;

        case 'r':
            ret = bdbpm_rail_id(optarg, &user_dev_id, &user_rail_id);
            if (ret) {
                fprintf(stderr, "Invalid rail! (%s)\n", optarg);
                return -EINVAL;
            } else {
                dbg_verbose("%s => dev=%u, rail=%u.\n",
                            optarg, user_dev_id, user_rail_id);
                printf("%s rail selected.\n", optarg);
            }
            break;

        case 'i':
            ret = sscanf(optarg, "%u", &current_lsb);
            if (ret != 1) {
                current_lsb = DEFAULT_CURRENT_LSB;
                fprintf(stderr, "Invalid current precision (%s)! Using default %uuA.\n",
                        optarg, current_lsb);
            } else {
                dbg_verbose("%s => current_lsb=%uuA.\n",
                            optarg, current_lsb);
                printf("Selected %uuA current measurement precision.\n",
                       current_lsb);
            }
            break;

        case 't':
            conversion_time = bdbpm_main_optarg_to_ct(optarg);
            if (conversion_time == ina230_ct_count) {
                conversion_time = DEFAULT_CONVERSION_TIME;
                fprintf(stderr, "Invalid conversion time (%s)! Using default %s.\n",
                        optarg, bdbpm_main_ct_to_string(conversion_time));
            } else {
                dbg_verbose("%s => conversion_time=%u.\n", optarg, conversion_time);
                printf("Selected %s conversion time.\n", optarg);
            }
            break;

        case 'n':
            avg_count = bdbpm_main_optarg_to_avg_count(optarg);
            if (avg_count == ina230_avg_count_max) {
                avg_count = DEFAULT_AVG_SAMPLE_COUNT;
                fprintf(stderr, "Invalid averaging sample count (%s)! Using default %u.\n",
                        optarg, ina230_avg_count_to_int(avg_count));
            } else {
                dbg_verbose("%s => avg_count=%u.\n", optarg, avg_count);
                printf("Selected %s averaging sample count.\n", optarg);
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
                printf("Using %uus sampling rate.\n", refresh_rate);
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
                printf("Looping %u times.\n", loopcount);
            }
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
 * @brief           Return maximum rail count depending on user options.
 * @return          maximum rail count
 */
static uint8_t bdbpm_main_get_max_rail_count(void)
{
    uint8_t max_rcount;

    if (user_dev_id == DEV_COUNT) {
        max_rcount = DEV_MAX_RAIL_COUNT;
    } else {
        if (user_rail_id == DEV_MAX_RAIL_COUNT) {
            max_rcount = bdbpm_dev_rail_count(user_dev_id);
        } else {
            max_rcount = 1;
        }
    }
    dbg_verbose("%s(): max_rcount=%u\n", __func__, max_rcount);
    return max_rcount;
}

/**
 * @brief           Collect power measurements of all user-selected rails.
 * @return          0 on success, standard error codes otherwise
 */
static int bdbpm_main_collect_measurements(void)
{
    int ret = 0;
    uint8_t d_start, d_end;
    uint8_t r_start, r_end;
    uint8_t d, r;

    bdbpm_main_get_device_list(&d_start, &d_end);
    for (d = d_start; d < d_end; d++) {
        bdbpm_main_get_rail_list(d, &r_start, &r_end);
        for (r = r_start; r < r_end; r++) {
            ret = bdbpm_measure_rail(bdbpm_rails[d][r], &measurements[d][r]);
            if (ret) {
                fprintf(stderr, "failed to collect %s measurements!!! (%d)\n",
                    bdbpm_rail_name(d, r));
                return ret;
            } else {
                dbg_verbose("%s(): %s: %uuV %uuA %uuW\n", __func__,
                    bdbpm_rail_name(d, r),
                    measurements[d][r].uV, measurements[d][r].uA,
                    measurements[d][r].uW);
            }
        }
    }

    return 0;
}

/**
 * @brief           Init variables used for drawing table in console.
 */
static void bdbpm_main_display_init(void)
{
    int max_rcount;
    uint8_t r;

    sprintf(separator, "---------------");
    sprintf(header, "| Module/Rail |");
    max_rcount = bdbpm_main_get_max_rail_count();
    for (r = 0; r < max_rcount; r++) {
        strcat(separator, "---------------------");
        strcat(header, "                    |");
    }
}

/**
 * @brief           Draw a table in console displaying all measurements.
 */
static void bdbpm_main_display_measurements(void)
{
    uint8_t d_start, d_end;
    uint8_t r_start, r_end;
    uint8_t d, r;
    int max_rcount;

    printf("%s\n", separator);
    printf("%s\n", header);

    max_rcount = bdbpm_main_get_max_rail_count();
    bdbpm_main_get_device_list(&d_start, &d_end);
    for (d = d_start; d < d_end; d++) {
        printf("| %-11s |", bdbpm_dev_name(d));
        bdbpm_main_get_rail_list(d, &r_start, &r_end);
        for (r = 0; r < max_rcount; r++) {
            if ((r_start + r) < r_end) {
                printf(" %-18s |", bdbpm_rail_name(d, r_start + r));
            } else {
                printf(" %-18s |", "");
            }
        }
        printf("\n");

        printf("| %-11s |", "");
        for (r = 0; r < max_rcount; r++) {
            if ((r_start + r) < r_end) {
                printf(" V (mV): %-10d |", measurements[d][r_start + r].uV);
            } else {
                printf(" %-18s |", "");
            }
        }
        printf("\n| %-11s |", "");
        for (r = 0; r < max_rcount; r++) {
            if ((r_start + r) < r_end) {
                printf(" I (uA): %-10d |", measurements[d][r_start + r].uA);
            } else {
                printf(" %-18s |", "");
            }
        }
        printf("\n| %-11s |", "");
        for (r = 0; r < max_rcount; r++) {
            if ((r_start + r) < r_end) {
                printf(" P (uW): %-10d |", measurements[d][r_start + r].uW);
            } else {
                printf(" %-18s |", "");
            }
        }
        printf("\n%s\n", separator);
    }
}

/**
 * @brief           Do all the necessary initializations.
 * @return          0 on success, standard error codes otherwise.
 */
static int bdbpm_main_init(void)
{
    uint8_t d, r;
    uint8_t d_start, d_end;
    uint8_t r_start, r_end;
    int ret;

    /* Init library */
    ret = bdbpm_init(current_lsb, conversion_time, avg_count);
    if (ret) {
        fprintf(stderr, "Error during initialization!!! (%d)\n", ret);
        return ret;
    }

    /* Clear pointers and measurements*/
    for (d = 0; d < DEV_COUNT; d++) {
        for (r = 0; r < DEV_MAX_RAIL_COUNT; r++) {
            bdbpm_rails[d][r] = NULL;
            measurements[d][r].uV = 0;
            measurements[d][r].uA = 0;
            measurements[d][r].uW = 0;
        }
    }

    /* Init all necessary power measurement rails */
    bdbpm_main_get_device_list(&d_start, &d_end);
    for (d = d_start; d < d_end; d++) {
        bdbpm_main_get_rail_list(d, &r_start, &r_end);
        for (r = r_start; r < r_end; r++) {
            bdbpm_rails[d][r] = bdbpm_init_rail(d, r);
            if (!bdbpm_rails[d][r]) {
                fprintf(stderr, "%s(): Failed to init %s rail! (%d)\n",
                        __func__, bdbpm_rail_name(d, r), ret);
                return -EIO;
           }
        }
    }

    bdbpm_main_display_init();

    /* Check user configured refresh rate >= configured sampling time */
    if (refresh_rate < bdbpm_get_sampling_time(bdbpm_rails[d_start][r_start])) {
        printf("WARNING: configured refresh rate (%uus) < configured sampling time (%uus).\n",
               refresh_rate,
               bdbpm_get_sampling_time(bdbpm_rails[d_start][r_start]));
        refresh_rate = bdbpm_get_sampling_time(bdbpm_rails[d_start][r_start]);
        printf("         Changing refresh rate to %uus.\n\n", refresh_rate);
    }

    /*
     * After initialization, and due to averaging and acquisition time,
     * an initial delay is required before starting collecting measurements
     * (or 0 will be read).
     */
    usleep(bdbpm_get_sampling_time(bdbpm_rails[d_start][r_start]));

    printf("Power measurement HW initialized.\n");
    return 0;
}

/**
 * @brief           Do all the necessary deinitializations.
 */
static void bdbpm_main_deinit(void)
{
    uint8_t d, r;
    uint8_t d_start, d_end;
    uint8_t r_start, r_end;

    bdbpm_main_get_device_list(&d_start, &d_end);
    for (d = d_start; d < d_end; d++) {
        bdbpm_main_get_rail_list(d, &r_start, &r_end);
        for (r = r_start; r < r_end; r++) {
            if (bdbpm_rails[d][r]) {
                bdbpm_deinit_rail(bdbpm_rails[d][r]);
            }
        }
    }
    bdbpm_deinit();

    printf("Power measurement HW and library deinitialized.\n\n");
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
int bdbpm_main(int argc, char *argv[])
#endif
{
    int ret;

    ret = bdbpm_main_get_user_options(argc, argv);
    if (ret) {
        usage();
        exit(-EINVAL);
    }

    ret = bdbpm_main_init();
    if (ret) {
        bdbpm_main_deinit();
        exit(ret);
    }

    printf("\nGetting power measurements...\n\n");
    if ((continuous) || (loopcount > 1)) {
         /* Clear terminal */
        printf("\033[2J\033[1;1H");
    }

    do {
        ret = bdbpm_main_collect_measurements();
        if (ret) {
            break;
        }

        bdbpm_main_display_measurements();

        if (!continuous) {
            loopcount -= 1;
        }
        if (loopcount != 0) {
            usleep(refresh_rate);
            /* Clear terminal */
            printf("\033[2J\033[1;1H");
        }
    } while (loopcount != 0);
    printf("\n");

    bdbpm_main_deinit();

    return 0;
}
