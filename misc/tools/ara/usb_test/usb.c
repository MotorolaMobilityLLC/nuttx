/*
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

#include <libusb-1.0/libusb.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <endian.h>
#include <stddef.h>

/* Use list library from nuttx folder */
#include <nuttx/list.h>

#define MTU 2048
//#define DUMP

struct gb_operation_hdr {
    uint16_t size;
    uint16_t id;
    uint8_t type;
    uint8_t result;             /* present in response only */
    uint8_t pad[2];
};

struct loopback_transfer {
    unsigned int size;
    void *data;

    struct list_head list;
    void *buffer;
    void *response_buffer;
    struct libusb_transfer *transfer;
    struct libusb_transfer *response;
};

struct transfer_param
{
    unsigned int ep_set;
    unsigned int size;
    unsigned int count;
    unsigned int timeout;
};

static LIST_DECLARE(transfers);
int transfer_error = 0;

uint8_t ep_set_to_epin(unsigned int ep_set)
{
    uint8_t epin;
    ep_set++;
    epin = (ep_set << 1) - 1;
    return (epin | LIBUSB_ENDPOINT_IN);
}

uint8_t ep_set_to_epout(unsigned int ep_set)
{
    uint8_t epout;
    ep_set++;
    epout = (ep_set << 1);
    return (epout | LIBUSB_ENDPOINT_OUT);
}

void transfer_complete(struct loopback_transfer *lb_transfer)
{
    struct libusb_transfer *transfer = lb_transfer->transfer;
    struct libusb_transfer *response = lb_transfer->response;

    free(transfer->buffer);
    libusb_free_transfer(transfer);
    free(response->buffer);
    libusb_free_transfer(response);
    list_del(&lb_transfer->list);
    free(lb_transfer);
}

void send_data_cb(struct libusb_transfer *transfer)
{
    struct loopback_transfer *lb_transfer = transfer->user_data;
    struct libusb_transfer *response = lb_transfer->response;
#ifdef DUMP
    int i;
    printf("send status = %d\n", transfer->status);
    for (i = 0; i < 8; i++) {
        printf("%02x ", transfer->buffer[i]);
    }
    printf("\n");
#endif

    if (transfer->status) {
        libusb_cancel_transfer(response);
    }
}

int send_data(libusb_device_handle * dev_handle, unsigned int ep_set,
              struct loopback_transfer *lb_transfer, unsigned int timeout)
{
    struct libusb_transfer *transfer = lb_transfer->transfer;

    libusb_fill_bulk_transfer(transfer, dev_handle,
                              ep_set_to_epout(ep_set),
                              lb_transfer->data, lb_transfer->size,
                              send_data_cb, lb_transfer, timeout);
    return libusb_submit_transfer(transfer);
}

void recv_data_cb(struct libusb_transfer *response)
{
    int i;
    uint16_t id;
    struct loopback_transfer *lb_transfer = response->user_data;
    struct libusb_transfer *transfer = lb_transfer->transfer;

#ifdef DUMP
    printf("recv status = %d\n", response->status);
    for (i = 0; i < 8; i++) {
        printf("%02x ", response->buffer[i]);
    }
    printf("\n");
#endif

    if (response->status) {
        transfer_error++;
        goto complete;
    }
    if (transfer->length != response->length) {
        transfer_error++;
        goto complete;
    }

    if (memcmp(transfer->buffer, response->buffer, response->length))
        transfer_error++;

 complete:
    transfer_complete(lb_transfer);
}

int recv_data(libusb_device_handle * dev_handle, unsigned int ep_set,
              struct loopback_transfer *lb_transfer, unsigned int timeout)
{
    int r;
    unsigned int size;
    unsigned char *data;
    struct libusb_transfer *transfer = lb_transfer->response;

    libusb_fill_bulk_transfer(transfer, dev_handle,
                              ep_set_to_epin(ep_set),
                              lb_transfer->response_buffer,
                              lb_transfer->size,
                              recv_data_cb, lb_transfer, timeout);
    r = libusb_submit_transfer(transfer);
    if (r != 0) {
        free(data);
    }

    return r;
}

void transfer_data(libusb_device_handle * dev_handle, unsigned int ep_set,
                   unsigned char *data, unsigned int size,
                   unsigned int timeout)
{
    int r;
    int actual;
    struct libusb_transfer *transfer;
    struct loopback_transfer *lb_transfer;

    lb_transfer = malloc(sizeof(*lb_transfer));
    if (!lb_transfer) {
        printf("Can not allocate loopback transfer\n");
        goto errout_lb_alloc;
    }

    lb_transfer->buffer = malloc(size);
    if (!lb_transfer->buffer) {
        printf("Can not allocate buffer\n");
        goto errout_buffer;
    }

    lb_transfer->response_buffer = malloc(size);
    if (!lb_transfer->response_buffer) {
        printf("Can not allocate response buffer\n");
        goto errout_response_buffer;
    }

    lb_transfer->transfer = libusb_alloc_transfer(0);
    if (!lb_transfer->transfer) {
        printf("Can not allocate transfer\n");
        goto errout_alloc_transfer;
        return;
    }

    lb_transfer->response = libusb_alloc_transfer(0);
    if (!lb_transfer->response) {
        printf("Can not allocate response transfer\n");
        goto errout_alloc_response;
        return;
    }

    lb_transfer->size = size;
    lb_transfer->data = data;
    if (send_data(dev_handle, ep_set, lb_transfer, timeout))
        goto errout_send_data;
    if (recv_data(dev_handle, ep_set, lb_transfer, timeout))
        goto errout_send_data;

    list_init(&lb_transfer->list);
    list_add(&transfers, &lb_transfer->list);

    return;

 errout_send_data:
    libusb_free_transfer(lb_transfer->response);
 errout_alloc_response:
    libusb_free_transfer(lb_transfer->transfer);
 errout_alloc_transfer:
    free(lb_transfer->response_buffer);
 errout_response_buffer:
    free(lb_transfer->buffer);
 errout_buffer:
    free(lb_transfer);
 errout_lb_alloc:
    transfer_error++;
}

void help(void)
{
    int size = MTU - sizeof(struct gb_operation_hdr);
    printf("usb_test [-s size] [-n count]\n"
           "\t -s size: size of data to send: 0 <= size <= %d\n"
           "\t -c count: number of packet to send: count > 0\n"
           "\t -t seconds: timeout in seconds: seconds > 0\n", size);
    exit(0);
}

void getopts(int argc, char *argv[], struct transfer_param *param)
{
    int r;
    int opt;

    param->count = 1;
    param->size = 0;
    param->ep_set = 0;
    param->timeout = 1000;

    while ((opt = getopt(argc, argv, "he:s:c:t:")) != -1) {
        switch (opt) {
        case 'c':
            r = sscanf(optarg, "%u", &param->count);
            if (r != 1 || param->count <= 0)
                help();
            break;
        case 's':
            r = sscanf(optarg, "%u", &param->size);
            if (r != 1 || param->size < 0 || param->size > MTU)
                help();
            break;
        case 't':
            r = sscanf(optarg, "%u", &param->timeout);
            param->timeout *= 1000;
            if (r != 1 || param->timeout < 1000)
                help();
            break;
        default:
            help();
        }
    }
}

int main(int argc, char *argv[])
{
    int i;
    int r;
    ssize_t cnt;
    unsigned int size = 0;

    unsigned char *data;
    struct gb_operation_hdr *hdr;
    struct transfer_param param;

    struct timeval tv = {.tv_sec = 0,.tv_usec = 0 };

    FILE *file;

    libusb_device **devs;
    libusb_device_handle *dev_handle;
    libusb_context *ctx = NULL;

    getopts(argc, argv, &param);

    r = libusb_init(&ctx);
    if (r < 0) {
        printf("Init Error %d\n", r);   //there was an error
        return 1;
    }
    libusb_set_debug(ctx, 3);

    cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0) {
        printf("Get Device Error\n");
        return 1;
    }
    printf("%d Devices in list.\n", (int)cnt);

    dev_handle = libusb_open_device_with_vid_pid(ctx, 0xffff, 0x0002);
    if (dev_handle == NULL) {
        printf("Cannot open device\n");
        goto error;
    } else {
        printf("Device Opened\n");
    }
    libusb_free_device_list(devs, 1);

    if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
        printf("Kernel Driver Active\n");
        if (libusb_detach_kernel_driver(dev_handle, 0) == 0)
            printf("Kernel Driver Detached!\n");
    }
    r = libusb_claim_interface(dev_handle, 0);
    if (r < 0) {
        printf("Cannot Claim Interface\n");
        goto error;
    }
    printf("Claimed Interface\n");

    printf("Start transfer\n");
    file = fopen("/dev/urandom", "r");
    for (i = 0; i < param.count; i++) {
        size = param.size;
        size += sizeof(*hdr);

        data = malloc(size);
        hdr = (struct gb_operation_hdr *)data;
        hdr->size = htole16(size);
        hdr->id = htole16(i);
        fread(&hdr[1], 1, size - sizeof(*hdr), file);

        transfer_data(dev_handle, param.ep_set, data, size, param.timeout);
    }
    fclose(file);

    tv.tv_sec = 1;
    printf("Waiting for transfer completion\n");
    while (!list_is_empty(&transfers)) {
        libusb_handle_events(ctx);
    }

    r = libusb_release_interface(dev_handle, 0);
    if (r < 0) {
        printf("Cannot Release Interface\n");
        goto error;
    }
    printf("Released Interface\n");

 error:
    if (dev_handle)
        libusb_close(dev_handle);
    libusb_exit(ctx);

    if (transfer_error) {
        printf("%d/%d transfers failed\n",
               transfer_error, param.count);
        r = -1;
    }

    return r;
}
