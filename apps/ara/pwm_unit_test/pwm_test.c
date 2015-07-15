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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/pwm.h>
#include <nuttx/device.h>
#include <nuttx/device_pwm.h>
#include <arch/tsb/chip.h>

#include <tsb_pwm.h>

/** Globe variable for interrupt status */
uint32_t val;

struct pwm_app_info {
    uint16_t    index;
    uint32_t    period;
    uint32_t    duty;
    uint8_t     polarity;
    uint32_t    secs;
    uint8_t     mode_num;
    uint32_t    param;
    bool        is_mode_test;
};

static void print_usage(void) {
    printf("Usage: pwm_test [-n pwm-num] [-m mode case] [-d duty] [-f period] "
           "[-t output time (second)] [-a argument] [-p polarity]\n");
    printf("    -n: '0' for PWM0 or '1' for PWM1 (default: 0).\n");
    printf("    -m: Mode case (0, 1 and 2) (default: 0).\n");
    printf("    -d: Active time, in nanosecond (default: 500ms).\n");
    printf("    -f: Waveform time, in nanosecond (default: 1 Sec).\n");
    printf("    -t: Waveform output times, in second, (default: 30 Sec).\n");
    printf("        for mode 0 case, this value depend on totally iteration * "
           "each cycle time\n");
    printf("    -a: Extra parameter for case mode.\n");
    printf("        For mode 0, totally output cycle times.\n");
    printf("        For mode 1, 0 is low level, 1 is high level.\n");
    printf("        For mode 2, true to start sync.\n");
    printf("    -p: 1 for normal, 0 for inverted.\n");
}

static void intr_handler(void* state)
{
    val = *(uint32_t *)state;
}

/**
 * Condifure a specific generator of waveform by period, duty and polarity.
 */
static int pwm_configure(struct device *dev, struct pwm_app_info *info,
                         bool mode_test)
{
    int ret = 0;

    ret = device_pwm_request_activate(dev, info->index);
    if (ret) {
        printf("pwm_test: active generator(%u) return error(%x)\n",
               info->index, ret);
        return ret;
    }

    ret = device_pwm_request_config(dev, info->index, info->duty,
                                    info->period);
    if (ret) {
        printf("pwm_test: config generator(%u) return error(%x)\n",
               info->index, ret);
        goto err_configure;
    }

    ret = device_pwm_request_set_polarity(dev, info->index, info->polarity);
    if (ret) {
        printf("pwm_test: set generator(%u) polarity return error(%x)\n",
               info->index, ret);
        goto err_configure;
    }

    if (mode_test) {
        ret = device_pwm_request_set_mode(dev, info->index, info->mode_num,
                                          &info->param);
        if (ret) {
            printf("pwm_test: configure mode %u return error(%x)\n",
                   info->mode_num, ret);
            goto err_configure;
        }

        if (info->mode_num == PWM_PULSECOUNT_MODE) {
            /* Test PWM1 interrupt function here!!!!*/
            val = 0;
            device_pwm_request_callback(dev, 0x02, intr_handler);
        }

    }

    ret = device_pwm_request_enable(dev, info->index);
    if (ret) {
        printf("pwm_test: enable generator(%u) return error(%x)\n",
               info->index, ret);
        goto err_configure;
    }

    return ret;

err_configure:
    device_pwm_request_deactivate(dev, info->index);
    return ret;
}

static int comm_parse(struct pwm_app_info *info, int argc, char **argv)
{
    int option;

    optind = -1; /* Force NuttX's getopt() to reinitialize. */
    while ((option = getopt(argc, argv, "n:m:d:f:t:a:p:")) != ERROR) {
        switch(option) {
            case 'n':
                info->index = (uint16_t)atoi(optarg);
                break;
            case 'm':
                info->is_mode_test = true;
                info->mode_num = (uint8_t)atoi(optarg);
                break;
            case 'd':
                info->duty = (uint32_t)atoi(optarg);
                if (info->duty < 1) {
                    printf("pwm_test: provided duty %s is invalid:check out "
                           "HW miminum supported!\n", info->duty);
                    return -EINVAL;
                }
                break;
            case 'f':
                info->period = (uint32_t)atoi(optarg);
                if (info->period < 1) {
                    printf("pwm_test: provided period %s is invalid:check out "
                           "HW miminum supported!\n", info->period);
                    return -EINVAL;
                }
                break;
            case 't':
                info->secs = (uint32_t)atoi(optarg);
                if (info->secs < 1) {
                    printf("pwm_test: provided secs %s is invalid: must be "
                           "positive\n", info->secs);
                    return -EINVAL;
                }
                break;
            case 'a':
                info->param = (uint32_t)atoi(optarg);
                break;
            case 'p':
                info->polarity = (uint32_t)atoi(optarg);
                break;
            case '?':
                printf("Illegal option: -%c\n", optopt);
                return -EINVAL;
            default:
                return -EINVAL;
        }
    }

    return 0;
}

static void dump_params(struct pwm_app_info *info)
{
    printf("pwm_test: index = %u\n", info->index);
    printf("pwm_test: period = %u\n", info->period);
    printf("pwm_test: duty = %u\n", info->duty);
    printf("pwm_test: polarity = %u\n", info->polarity);
    printf("pwm_test: secs =%u\n", info->secs);
    printf("pwm_test: mode_num = %u\n", info->mode_num);
    printf("pwm_test: param = %u\n", info->param);
    printf("pwm_test: is_mode_test = %u\n", info->is_mode_test);
}

static void default_params(struct pwm_app_info *info)
{
    info->index = 0;
    info->period = 1000000000;
    info->duty = 500000000;
    info->polarity = 0;
    info->secs = 30;
    info->mode_num = 0;
    info->param = 0;
    info->is_mode_test= false;
}


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[]) {
#else
int pwm_test_main(int argc, char *argv[]) {
#endif
    struct device *dev;
    struct pwm_app_info *info = NULL;
    uint16_t count;
    uint32_t ret = 0;
    int i;

    info = zalloc(sizeof(struct pwm_app_info));
    if (!info) {
        printf("pwm_test: App initialization failed!\n");
        return EXIT_FAILURE;
    }

    default_params(info);

    if (comm_parse(info, argc, argv)) {
        print_usage();
        goto err_free_info;
    }

    dev = device_open(DEVICE_TYPE_PWM_HW, 0);
    if (!dev) {
        printf("pwm_test: Open PWM controller driver failure!\n");
        ret = EXIT_FAILURE;
        goto err_free_info;
    }

    device_pwm_request_count(dev, &count);

    if (info->index > count) {
        printf("pwm_test: pwm%u > maximum supported number %u\n", info->index,
               count);
        print_usage();
        ret =  EXIT_FAILURE;
        goto err_out;
    }

    if (!info->is_mode_test) {
        ret = pwm_configure(dev, info, false);
        if (ret) {
            printf("pwm_test: Failed to enabled PWM%u \n", info->index);
            goto err_out;
        }
    } else {
        switch (info->mode_num) {
        case PWM_PULSECOUNT_MODE:
            if (info->param < 1) {
                printf("pwm_test: mode 0 of -a must > 0\n");
                print_usage();
                ret =  EXIT_FAILURE;
                goto err_out;
            }

            info->secs = ((info->period/1000000000) * info->param) < 1 ? 1 :
                          (info->period/1000000000) * info->param;

            ret = pwm_configure(dev, info, true);
            if (ret) {
                printf("pwm_test: mode 0, set generator(%u) in mode(%u) has "
                       "failed\n", info->index, info->mode_num);
                goto err_out;
            }
            break;
        case PWM_STOP_LEVEL_MODE:
            ret = pwm_configure(dev, info, true);
            if (ret) {
                printf("pwm_test: mode1, set generator(%u) in mode(%u) has "
                       "failed\n", info->index, info->mode_num);
                goto err_out;
            }
            break;
        case PWM_SYNC_MODE:
            /**
             * This case will test all supported generstors to output waveform
             * at sametime that have same period configuration.
             */
            info->param = 1;
            for (i = 0; i < count; i++) {
                ret = device_pwm_request_activate(dev, i);
                if (ret) {
                    printf("pwm_test: mode2, active generator(%d) return "
                           "error(%x)\n", i, ret);
                    break;
                }

                ret = device_pwm_request_config(dev, i, info->period /(2+i),
                                                info->period);
                if (ret) {
                    printf("pwm_test: mode2, config generator(%d) return "
                           "error(%x)\n", i, ret);
                    break;
                }

                ret = device_pwm_request_set_mode(dev, i, info->mode_num,
                                                  &info->param);
                if (ret) {
                    printf("pwm_test: mode 2, set generator(%d) of SELENB bit "
                           "return error(%x)\n", i, ret);
                    break;
                }
            }

            if (ret) {
                /**
                 * Make sure already activated pwm can be deactivated if has
                 * error.
                 */
                for (; i >= 0; i--) {
                    device_pwm_request_deactivate(dev, i);
                }
                goto err_out;
            }

            device_pwm_request_sync(dev, true);
            break;
        default:
            print_usage();
            ret = EXIT_FAILURE;
            goto err_out;
        }
    }

    dump_params(info);

    printf("pwm_test: Enabled PWM%u for a duration of %u seconds\n",
           info->index, info->secs);

    sleep((unsigned int)info->secs);

    if (info->is_mode_test && (info->mode_num == PWM_PULSECOUNT_MODE)) {
        printf("pwm_test: test completed, interrupt callback return value "
               "= %u\n", val);
    } else {
        printf("pwm_test: test completed\n");
    }

err_out:
    device_close(dev);
err_free_info:
    free(info);

    return ret;
}
