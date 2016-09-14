/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#include <debug.h>
#include <fcntl.h>
#include <poll.h>

#include <stdbool.h>
#include <stdlib.h>
#include <sched.h>
#include <unistd.h>

#include <nuttx/analog/adc.h>
#include <nuttx/device.h>
#include <nuttx/device_ext_power.h>
#include <nuttx/device_wrls_tx.h>
#include <nuttx/power/ext_power.h>
#include <nuttx/power/wrls_coil_adc.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#define WRLS_COIL_ADC_TASK_PRIORITY     (100)
#define WRLS_COIL_ADC_TASK_STACK_SIZE   (1024)

/* FIFO to pass docked state to task */
static const char *fifo_name = "/dev/wrls_coil_adc_docked_state_fifo";

static bool wrls_coil_adc_docked(struct device *dev)
{
    device_ext_power_output_s output;

    if (dev && !device_ext_power_get_output(dev, &output))
        return output.current != 0;
    else
        return false;
}

void wrls_coil_adc_pad_detect(void *arg, struct device *const dev[])
{
    int fd;
    ssize_t nbytes;
    bool docked =  wrls_coil_adc_docked(dev[EXT_POWER_WIRELESS]);

    fd = open(fifo_name, O_WRONLY);
    if (fd == -1) {
        dbg("open() fifo to write failed: %d\n", errno);
        return;
    }

    nbytes = write(fd, &docked, sizeof(docked));
    if (nbytes != sizeof(docked))
        dbg("write() docked failed: %d\n", errno);

    close(fd);
}

/* fds[] indices */
#define DOCKED      (0)
#define ADC         (1)
#define FDS_SIZE    (2)
static int wrls_coil_adc_task(int argc, char *argv[])
{
    struct device *wrls_tx;      /* control wireless charger TX */
    struct pollfd fds[FDS_SIZE]; /* poll() ADC IRQ and FIFO */
    int fd = -1;
    int ret;

    wrls_tx = device_open(DEVICE_TYPE_WRLS_TX_HW, 0);
    if (!wrls_tx) {
        dbg("opening wireless tx device failed\n");
        return -EXIT_FAILURE;
    }

    ret = adc_devinit();
    if (ret) {
        dbg("adc_devinit() failed: %d\n", ret);
        goto err;
    }

    ret = mkfifo(fifo_name, 0666);
    if (ret) {
        dbg("creating FIFO failed: %d\n", errno);
        goto err;
    }

    /* Open FIFO for writing to be able to open it for reading since all
     * attempts to open a FIFO read-only are blocked until at least one
     * thread has opened the FIFO for writing
     */
    fd = open(fifo_name, O_WRONLY);
    if (fd == -1) {
        dbg("opening FIFO for writing failed: %d\n", errno);
        goto err;
    }

    fds[DOCKED].fd = open(fifo_name, O_RDONLY);
    if (fds[DOCKED].fd == -1) {
        dbg("opening FIFO for reading failed: %d\n", errno);
        goto err;
    } else {
        close(fd);
        fd = -1;
    }

    fds[ADC].fd = -1; /* ADC IRQ is not needed until docked state is known */

    fds[DOCKED].events = POLLIN;
    fds[ADC].events = POLLIN;

    ret = ext_power_register_callback(wrls_coil_adc_pad_detect, NULL);
    if (ret) {
        dbg("register_ext_power_callback() failed: %d\n", ret);
        goto err;
    }

    /* State */
    bool docked = false;
    bool hot    = false;

    do {
        ssize_t nbytes;

        /* Wait for docked state to change or ADC IRQ to go off */
        ret = poll(fds, FDS_SIZE, -1);
        if (ret == -1 && errno != EINTR) {
            dbg("poll() failed: %d\n", errno);
            goto err; /* unrecoverable */
        }

        /* Did docked state change? */
        if (fds[DOCKED].revents & POLLIN) {
            bool tmp;
            nbytes = read(fds[DOCKED].fd, &tmp, sizeof(tmp));
            if (nbytes == sizeof(tmp)) {
                docked = tmp;
            } else if (nbytes == -1 && errno != EINTR) {
                dbg("reading docked failed: %d\n", errno);
            }
        }

        /* Did ADC IRQ go off? */
        if (fds[ADC].revents & POLLIN) {
            struct adc_msg_s sample;
            nbytes = read(fds[ADC].fd, &sample, sizeof(sample));
            if (nbytes == sizeof(sample)) {
                hot = !hot;
                (void) device_wrls_tx_enable(wrls_tx, !hot);
            } else if (nbytes == -1 && errno == EINTR) {
                dbg("reading ADC failed: %d\n", errno);
            }

            /* ADC needs to be closed before new ADC IRQ can be setup */
            close(fds[ADC].fd);
            fds[ADC].fd = -1;
        }

        /* Monitor coil temp while docked and when coil is hot */
        bool monitor = docked || hot;

        if (monitor && fds[ADC].fd == -1) {
            int req;
            unsigned long arg;

            /* Setup ADC IRQ */
            fds[ADC].fd = open(CONFIG_WRLS_COIL_ADC_DEVPATH, O_RDONLY);
            if (fds[ADC].fd == -1) {
                dbg("opening ADC failed: %d\n", errno);
                continue;
            }

            req = hot ? ANIOC_WDOG_UPPER : ANIOC_WDOG_LOWER;
            arg = hot ? CONFIG_WRLS_COIL_ADC_NORMAL : CONFIG_WRLS_COIL_ADC_HOT;

            ret = ioctl(fds[ADC].fd, req, arg);
            if (ret == -1) {
                dbg("Setting ADC threshold failed: %d\n", errno);
                close(fds[ADC].fd);
                fds[ADC].fd = -1;
                continue;
            }

            ret = ioctl(fds[ADC].fd, ANIOC_TRIGGER, 0);
            if (ret == -1) {
                dbg("Triggering ADC conversion failed: %d\n", errno);
                close(fds[ADC].fd);
                fds[ADC].fd = -1;
                continue;
            }
        } else if (!monitor && fds[ADC].fd != -1) {
            /* ADC IRQ is not needed anymore */
            close(fds[ADC].fd);
            fds[ADC].fd = -1;
        }

    } while (1);

err:
    device_close(wrls_tx);
    close(fd);
    close(fds[DOCKED].fd);
    close(fds[ADC].fd);
    return -EXIT_FAILURE;
}

int wrls_coil_adc_init(void)
{
    int pid;

    /* Create task instead of pthread to avoid adding open file descriptors
     * to the main user task
     */
    pid = task_create("wrls_coil_adc", WRLS_COIL_ADC_TASK_PRIORITY,
                       WRLS_COIL_ADC_TASK_STACK_SIZE, wrls_coil_adc_task, NULL);
    if (pid < 0) {
        dbg("task_create() failed: %d\n", errno);
        return -errno;
    }

    return 0;
}
