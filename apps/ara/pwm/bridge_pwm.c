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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <nuttx/pwm.h>
#include <arch/tsb/chip.h>

#include <tsb_scm.h>

#include <up_arch.h>

/*
 * Interface provided by TSB (currently for BDBs) and other architectures to initialize
 * the PWM hardware.  It is safe to call repeatedly.
 */
int pwm_devinit(void);

static void print_usage(void) {
    printf("PWM: Usage:\n");
    printf("PWM:  pwm <pwm-num> <secs> : output PWM for <sec> seconds\n");
    printf("PWM:  <pwm-num> is '0' for PWM0 or '1' for PWM1\n");
    printf("PWM:  Output waveform is 300Khz with a 50%% duty cycle\n");
}

/*
 * Returns a file descriptor for the PWM device that was opened and enabled,
 * or a negative value if an error occurred.
 * Disabling pwm is not necessary after a negative return.
 */
static int pwm_enable(unsigned int pwm_id) {
    struct pwm_info_s pwm_info;
    int ret;
    int fd;
    char *pwm_name;

    pwm_info.frequency = 300000; /* 300Khz */
    pwm_info.duty = 32768;       /* Approximately 50% duty cycle: subject to rounding errors */

    switch(pwm_id) {
    case 0:
        pwm_name = "/dev/pwm0";
        break;
    case 1:
        pwm_name = "/dev/pwm1";
        break;
    default:
        printf("PWM: pwm_enable called with unknown pwm_id %u: valid values are 0 and 1\n",
               pwm_id);
        return -1;
    }

    ret = pwm_devinit();
    if (ret != OK)
    {
        printf("PWM: pwm_devinit() failed to initialize the PWM hardware: %d\n", ret);
        return ret;
    }

    fd = open(pwm_name, O_RDONLY);
    if (fd < 0) {
        printf("PWM: open of \"%s\" failed for pwm_id %u\n", pwm_name, pwm_id);
        return -1;
    }

    ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&pwm_info);
    if (ret < 0) {
        printf("PWM: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
        close(fd);
        return ret;
    }

    ret = ioctl(fd, PWMIOC_START, 0);
    if (ret < 0) {
        printf("PWM: ioctl(PWMIOC_START) failed: %d\n", errno);
        close(fd);
        return ret;
    }
    return fd;
}

static void pwm_disable(int fd) {
    int ret;

    if (fd < 0) {
        printf("PWM: pwm_disable called with an invalid fd %d\n", fd);
    } else {
        ret = ioctl(fd, PWMIOC_STOP, 0);
        if (ret < 0) {
            printf("PWM: ioctl(PWMIOC_STOP) failed: %d\n", errno);
        }
        close(fd);
    }
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[]) {
#else
    int pwm_main(int argc, char *argv[]) {
#endif
    unsigned int pwm_id;
    char pwm_num;
    int fd;
    int ret = 0;
    int secs;

    if (argc != 3 || argv[1][1] != '\0') {
        print_usage();
        return EXIT_FAILURE;
    } else {
        pwm_num = argv[1][0];
    }

    switch (pwm_num) {
    case '0':
        pwm_id = 0;
        break;
    case '1':
        pwm_id = 1;
        break;
    default:
        printf("PWM: pwm-num '%c' is invalid: must be 0 or 1\n", pwm_num);
        print_usage();
        return EXIT_FAILURE;
    }

    secs = atoi(argv[2]);
    if (secs < 1) {
        printf("PWM: provided secs of %s is invalid: must be positive\n",
               argv[2]);
        print_usage();
        return EXIT_FAILURE;
    }

    /*
     * GPIO9 - PWM1 and SDIO - PWM0 share the same pin on the Bridge ASICs
     * and are therefore mutually exclusive.
     *
     * Setting the TSB_PIN_UART_CTSRTS bit of the PINSHARE register also blocks PWM1 output.
     *
     * Therefore, if we want to see PWM1 output, we need to ensure that the TSB_PIN_GPIO9
     * and TSB_PIN_UART_CTSRTS bits in the PINSHARE register are clear..
     * For PWM0 output we need to clear the PWM0/SDIO pin.
     */
    switch (pwm_id) {
    case 0:
        ret = tsb_request_pinshare(TSB_PIN_SDIO);
        if (ret) {
            fprintf(stderr, "PWM: pins already held by another driver.\n");
            return ret;
        }

        tsb_clr_pinshare(TSB_PIN_SDIO);
        break;

    case 1:
        /*
         * We don't request the pinshare here because the driver is already
         * requesting it.
         * The architecture of the driver is not really good right now because
         * the driver shall be the one requesting the pinshare for all the pins
         * and we shouldn't have to configure them here.
         * Until the PWM driver is fixed to correctly handle the pinsharing,
         * lets just not request it here.
         */

        tsb_clr_pinshare(TSB_PIN_GPIO9 | TSB_PIN_UART_CTSRTS);
        break;
    }

    fd = pwm_enable(pwm_id);
    if (fd < 0) {
        printf("PWM: failed to enable PWM for pwm_id %u\n", pwm_id);
        ret = EXIT_FAILURE;
    } else {
        printf("PWM: Enabled PWM%u for a duration of %d seconds\n", pwm_id, secs);
        fflush(stdout);
        sleep((unsigned int)secs);
        pwm_disable(fd);
        printf("PWM: test completed\n");
    }

    switch (pwm_id) {
    case 0:
        tsb_release_pinshare(TSB_PIN_SDIO);
        break;
    }

    return ret;
}
