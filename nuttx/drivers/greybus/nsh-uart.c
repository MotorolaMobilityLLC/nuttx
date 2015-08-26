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
#include <debug.h>
#include <stdlib.h>
#include <queue.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/buf_con.h>
#include <nuttx/device.h>
#include <nuttx/device_uart.h>
#include <nuttx/util.h>
#include <nuttx/config.h>
#include <nuttx/greybus/types.h>
#include <nuttx/greybus/greybus.h>
//#include <arch/tsb/unipro.h>
#include <apps/greybus-utils/utils.h>
#include <arch/byteorder.h>

#include "uart-gb.h"

#define GB_UART_VERSION_MAJOR   0
#define GB_UART_VERSION_MINOR   1

/* The id of error in protocol operating. */
#define GB_UART_EVENT_PROTOCOL_ERROR    1
#define GB_UART_EVENT_DEVICE_ERROR      2

/**
 * UART protocol information structure.
 */
struct gb_nsh_uart_info {
    /** cport from greybus */
    uint16_t            cport;
};

/* The structure for keeping protocol global data. */
static struct gb_nsh_uart_info *info = NULL;

/**
 * @brief Report the error.
 *
 * When error in callback or thread, this function show error message to user.
 *
 * Note: as the discussion, only UART related errors such as overrun should be
 * reported via UART protocol. So just show the generic errors to the console.
 *
 * @param error The error id.
 * @param func_name The error function name.
 * @return None.
 */
static void nsh_uart_report_error(int error, const char *func_name)
{
    switch (error) {
    case GB_UART_EVENT_PROTOCOL_ERROR:
        gb_info("%s(): operation send error \n", func_name);
        break;
    case GB_UART_EVENT_DEVICE_ERROR:
        gb_info("%s(): device io error \n", func_name);
        break;
    default:
    break;
    }
}

/**
 * @brief Callback for data receiving
 *
 * The callback function provided to device driver for being notified when
 * driver received a data stream.
 * It put the current operation to received queue and gets another operation to
 * continue receiving. Then notifies rx thread to process.
 *
 * @param n Size of data ready.
 * @param arg Callback argument
 * @return None.
 */
void gb_nsh_uart_read_ready_callback(ssize_t n, void *arg) {
    struct gb_uart_receive_data_request *request = NULL;

    struct gb_operation *operation =
        gb_operation_create(info->cport,
                            GB_UART_PROTOCOL_RECEIVE_DATA,
                            sizeof(*request) + n);
    if (!operation) {
        nsh_uart_report_error(GB_UART_EVENT_PROTOCOL_ERROR, __func__);
        return;
    }

    request = gb_operation_get_request_payload(operation);

    uint16_t length = buf_con_out_read(request->data, n);

    request->size = cpu_to_le16(length);

    int ret = gb_operation_send_request(operation, NULL, false);
    if (ret) {
        nsh_uart_report_error(GB_UART_EVENT_PROTOCOL_ERROR, __func__);
    }

    gb_operation_destroy(operation);
}

/**
 * @brief Releases resources for receiver thread.
 *
 * Terminates the thread for receiver and releases the system resouces and
 * operations allocated by uart_receiver_cb_init().
 *
 * @param None.
 * @return None.
 */
static void nsh_uart_receiver_cb_deinit(void)
{
    buf_con_set_read_ready_callback(NULL, 0);
}

/**
 * @brief Receiving data process initialization
 *
 * This function allocates OS resource to support the data receiving
 * function. It allocates two types of operations for undetermined length of
 * data. The semaphore works as message queue and all tasks are done in the
 * thread.
 *
 * @param None.
 * @return 0 for success, -errno for failures.
 */
static int nsh_uart_receiver_cb_init(void)
{
    buf_con_set_read_ready_callback(gb_nsh_uart_read_ready_callback, 0);
    return 0;
}

/**
 * @brief Modem and line status event init process
 *
 * This function creates one operations and uses that request of operation
 * for sending the status change event to peer.
 *
 * @param None.
 * @return 0 on success, error code on failure.
 */
static int nsh_uart_status_report(void)
{
    struct gb_operation *ms_ls_operation =
            gb_operation_create(info->cport,
                                GB_UART_PROTOCOL_SERIAL_STATE,
                                sizeof(struct gb_uart_serial_state_request));
    if (!ms_ls_operation) {
        return -ENOMEM;
    }

    struct gb_uart_serial_state_request *request =
        gb_operation_get_request_payload(ms_ls_operation);
    request->control = 0; /* spec doesn't define what's in this field */
    uint16_t status = GB_UART_CTRL_DCD | GB_UART_CTRL_DSR;
    request->data = cpu_to_le16(status);
    int ret = gb_operation_send_request(ms_ls_operation, NULL, false);
    if (ret) {
         nsh_uart_report_error(GB_UART_EVENT_PROTOCOL_ERROR, __func__);
    }

    gb_operation_destroy(ms_ls_operation);

    return ret;
}

/**
 * @brief Protocol get version function.
 *
 * Returns the major and minor Greybus UART protocol version number supported
 * by the UART device.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_nsh_uart_protocol_version(struct gb_operation *operation)
{
    struct gb_uart_proto_version_response *response =
        gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_UART_VERSION_MAJOR;
    response->minor = GB_UART_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol send data function.
 *
 * Requests that the UART device begin transmitting characters. One or more
 * bytes to be transmitted will be supplied.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_nsh_uart_send_data(struct gb_operation *operation)
{
    struct gb_uart_send_data_request *request =
                    gb_operation_get_request_payload(operation);

    int size = le16_to_cpu(request->size);
    int ret = buf_con_in_write(request->data, size);
    if (ret != size) {
        return GB_OP_INTERNAL;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol set line coding function.
 *
 * Sets the line settings of the UART to the specified baud rate, format,
 * parity, and data bits.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_nsh_uart_set_line_coding(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol set RTS & DTR line status function.
 *
 * Controls RTS and DTR line states of the UART.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_nsh_uart_set_control_line_state(struct gb_operation *operation)
{
   return GB_OP_SUCCESS;
}

/**
 * @brief Protocol send break function.
 *
 * Requests that the UART generate a break condition on its transmit line.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_nsh_uart_send_break(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol initialization function.
 *
 * This function perform the protocto initialization function, such as open
 * the cooperation device driver, launch threads, create buffers etc.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static int gb_nsh_uart_init(unsigned int cport)
{
    int ret;

    info = zalloc(sizeof(*info));
    if (info == NULL) {
        return -ENOMEM;
    }

    gb_debug("%s(): GB uart info struct: 0x%p \n", __func__, info);

    info->cport = cport;

    ret = nsh_uart_status_report();
    if (ret) {
        goto err_free_info;
    }

    ret = nsh_uart_receiver_cb_init();
    if (ret) {
        goto err_free_info;
    }

    return 0;

err_free_info:
    free(info);

    return ret;
}

/**
 * @brief Protocol exit function.
 *
 * This function can be called when protocol terminated.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return None.
 */
static void gb_nsh_uart_exit(unsigned int cport)
{
    nsh_uart_receiver_cb_deinit();

    free(info);
}

static struct gb_operation_handler gb_nsh_uart_handlers[] = {
    GB_HANDLER(GB_UART_PROTOCOL_VERSION, gb_nsh_uart_protocol_version),
    GB_HANDLER(GB_UART_PROTOCOL_SEND_DATA, gb_nsh_uart_send_data),
    GB_HANDLER(GB_UART_PROTOCOL_SET_LINE_CODING, gb_nsh_uart_set_line_coding),
    GB_HANDLER(GB_UART_PROTOCOL_SET_CONTROL_LINE_STATE,
               gb_nsh_uart_set_control_line_state),
    GB_HANDLER(GB_UART_PROTOCOL_SEND_BREAK, gb_nsh_uart_send_break),
};

struct gb_driver gb_nsh_uart_driver = {
    .init = gb_nsh_uart_init,
    .exit = gb_nsh_uart_exit,
    .op_handlers = (struct gb_operation_handler*) gb_nsh_uart_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_nsh_uart_handlers),
};

/**
 * @brief Protocol registering function.
 *
 * This function can be called by greybus to register the UART protocol.
 *
 * @param cport The number of CPort.
 * @return None.
 */
void gb_nsh_uart_register(int cport)
{
    gb_info("%s(): cport %d \n", __func__, cport);
    gb_register_driver(cport, &gb_nsh_uart_driver);
}
