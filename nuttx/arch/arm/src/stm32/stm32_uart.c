/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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
#include <fcntl.h>
#include <semaphore.h>

#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_uart.h>

#define STM32_UART_INVALID_RESOURCE (0xffffffff)

#define STM32_UART_FLAG_OPEN        BIT(0)

struct stm32_uart_info {
    /* ttySx file descriptor */
    int fd;
    /* physical ID of the UART */
    uint32_t phy_id;
    uint32_t flags;
    sem_t sem;
};

/**
 * @brief UART set_configuration function
 *
 * This function is used to set the baud rate, parity, data bit and stop bit
 * settings in the UART controller.
 *
 * IMPORTANT: The underlying tty driver does not currently support termios
 * so the tty device cannot be configured and will always return ENOSYS.
 * However, if the desired configuration is identical to the current hard-coded
 * configuration, then return success.  For this comparison to work, the
 * UART's physical id must be set in the device resources.
 *
 * @param dev pointer to the UART device structure
 * @param baud - the baud rate definition in Baud rate definition.
 * @param parity - the value of parity defined in Parity definition
 * @param databits - the number of data bits between 5 to 8 bits.
 * @param stopbits - the value stop bit defined in Stopbits definition
 * @param flow 0 for disable flow control, 1 for enable flow control.
 * @return 0 for success, -errno for failures.
 */
static int stm32_uart_set_configuration(struct device *dev, int baud,
                                int parity, int databits, int stopbit, int flow)
{
    struct stm32_uart_info *uart_info;
    int identical = 0;
    int expected_flow = 0;

    if (!dev) {
        return -ENODEV;
    }

    uart_info = device_get_private(dev);
    if (!uart_info) {
        return -ENODEV;
    }

    if (uart_info->phy_id == STM32_UART_INVALID_RESOURCE) {
        return -ENOSYS;
    }

    switch (uart_info->phy_id) {
#if CONFIG_USART1_ISUART
        case 1:
#if CONFIG_USART1_IFLOWCONTROL && CONFIG_USART1_OFLOWCONTROL
            expected_flow = 1;
#endif
            identical = ((baud == CONFIG_USART1_BAUD) &&
                         (parity == CONFIG_USART1_PARITY) &&
                         (databits == CONFIG_USART1_BITS) &&
                         (stopbit == CONFIG_USART1_2STOP) &&
                         (flow == expected_flow));
            break;
#endif

#if CONFIG_USART2_ISUART
        case 2:
#if CONFIG_USART2_IFLOWCONTROL && CONFIG_USART2_OFLOWCONTROL
            expected_flow = 1;
#endif
            identical = ((baud == CONFIG_USART2_BAUD) &&
                         (parity == CONFIG_USART2_PARITY) &&
                         (databits == CONFIG_USART2_BITS) &&
                         (stopbit == CONFIG_USART2_2STOP) &&
                         (flow == expected_flow));
            break;
#endif

#if CONFIG_USART3_ISUART
        case 3:
#if CONFIG_USART3_IFLOWCONTROL && CONFIG_USART3_OFLOWCONTROL
            expected_flow = 1;
#endif
            identical = ((baud == CONFIG_USART3_BAUD) &&
                         (parity == CONFIG_USART3_PARITY) &&
                         (databits == CONFIG_USART3_BITS) &&
                         (stopbit == CONFIG_USART3_2STOP) &&
                         (flow == expected_flow));
            break;
#endif

#if CONFIG_USART4_ISUART
        case 4:
#if CONFIG_USART4_IFLOWCONTROL && CONFIG_USART4_OFLOWCONTROL
            expected_flow = 1;
#endif
            identical = ((baud == CONFIG_USART4_BAUD) &&
                         (parity == CONFIG_USART4_PARITY) &&
                         (databits == CONFIG_USART4_BITS) &&
                         (stopbit == CONFIG_USART4_2STOP) &&
                         (flow == expected_flow));
            break;
#endif

#if CONFIG_USART5_ISUART
        case 5:
#if CONFIG_USART5_IFLOWCONTROL && CONFIG_USART5_OFLOWCONTROL
            expected_flow = 1;
#endif
            identical = ((baud == CONFIG_USART5_BAUD) &&
                         (parity == CONFIG_USART5_PARITY) &&
                         (databits == CONFIG_USART5_BITS) &&
                         (stopbit == CONFIG_USART5_2STOP) &&
                         (flow == expected_flow));
            break;
#endif

#if CONFIG_USART6_ISUART
        case 6:
#if CONFIG_USART6_IFLOWCONTROL && CONFIG_USART6_OFLOWCONTROL
            expected_flow = 1;
#endif
            identical = ((baud == CONFIG_USART6_BAUD) &&
                         (parity == CONFIG_USART6_PARITY) &&
                         (databits == CONFIG_USART6_BITS) &&
                         (stopbit == CONFIG_USART6_2STOP) &&
                         (flow == expected_flow));
            break;
#endif

#if CONFIG_USART7_ISUART
        case 7:
#if CONFIG_USART7_IFLOWCONTROL && CONFIG_USART7_OFLOWCONTROL
            expected_flow = 1;
#endif
            identical = ((baud == CONFIG_USART7_BAUD) &&
                         (parity == CONFIG_USART7_PARITY) &&
                         (databits == CONFIG_USART7_BITS) &&
                         (stopbit == CONFIG_USART7_2STOP) &&
                         (flow == expected_flow));
            break;
#endif

#if CONFIG_USART8_ISUART
        case 8:
#if CONFIG_USART8_IFLOWCONTROL && CONFIG_USART8_OFLOWCONTROL
            expected_flow = 1;
#endif
            identical = ((baud == CONFIG_USART8_BAUD) &&
                         (parity == CONFIG_USART8_PARITY) &&
                         (databits == CONFIG_USART8_BITS) &&
                         (stopbit == CONFIG_USART8_2STOP) &&
                         (flow == expected_flow));
            break;
#endif
    }

    return identical ? 0 : -ENOSYS;
}

/**
 * @brief UART stm32_uart_start_transmitter function
 *
 * Implement basic device_uart_start_transmitter() functionality.
 * Only blocking calls are supported.  dma and callback are not
 * supported.
 *
 * @param dev pointer to the UART device structure
 * @param buffer pointer of the buffer to send data to UART port.
 * @param length length of data.
 * @param dma DMA handle. (NOT SUPPORTED)
 * @param sent the length of transmitted data in block mode.
 * @param callback a callback function called when transmitting finished,
 *        timeout or errors. (NOT SUPPORTED)
 * @return 0 for success, -errno for failures.
 */
static int
stm32_uart_start_transmitter(struct device *dev, uint8_t *buffer, int length,
                       void *dma, int *sent,
                       void (*callback)(uint8_t *buffer, int length, int error))
{
    struct stm32_uart_info *uart_info;
    uint8_t *p = buffer;

    if (dma) {
        lldbg("ERROR: dma not supported\n");
        return -ENOSYS;
    }

    if (callback) {
        lldbg("ERROR: callback not supported\n");
        return -ENOSYS;
    }

    if (!dev) {
        return -ENODEV;
    }

    uart_info = device_get_private(dev);
    if (!uart_info) {
        return -ENODEV;
    }

    if (sent) {
        *sent = 0;
    }

    while (1) {
        ssize_t remaining = buffer + length - p;
        if (remaining <= 0) {
            return 0;
        }

        ssize_t n = write(uart_info->fd, p, remaining);
        if (n < 0) {
            n = -get_errno();
            if (n != -EINTR) {
                lldbg("ERROR: write failed: %d\n", n);
            }
            return n;
        } else if (sent) {
            *sent += n;
        }

        p += n;
    }
}

/**
 * @brief UART start_receiver function
 *
 * Implement basic device_uart_start_receiver() functionality.
 * Only blocking calls are supported.  dma and callback are not
 * supported.
 *
 * @param dev pointer to the UART device structure
 * @param buffer pointer of the buffer to receive data from UART port.
 * @param length length of data.
 * @param dma DMA handle. (NOT SUPPORTED)
 * @param got the length of received data in blocking mode.
 * @param callback a callback function called when receiving finished, timeout
 *                 or errors. (NOT SUPPORTED)
 * @return 0 for success, -errno for failures.
 */
static int
stm32_uart_start_receiver(struct device *dev, uint8_t *buffer, int length,
                       void *dma, int *got,
                       void (*callback)(uint8_t *buffer, int length, int error))
{
    struct stm32_uart_info *uart_info;

    if (dma) {
        lldbg("ERROR: dma not supported\n");
        return -ENOSYS;
    }

    if (callback) {
        lldbg("ERROR: callback not supported\n");
        return -ENOSYS;
    }

    if (!dev) {
        return -ENODEV;
    }

    uart_info = device_get_private(dev);
    if (!uart_info) {
        return -ENODEV;
    }

    if (got) {
        *got = 0;
    }

    ssize_t n = read(uart_info->fd, buffer, length);
    if (n < 0) {
        n = -get_errno();
        if (n != -EINTR) {
            lldbg("ERROR: read failed: %d\n", n);
        }
        return n;
    } else if (got) {
        *got = n;
    }

    return 0;
}

/**
 * @brief open UART device
 *

 * @param dev pointer to the UART device structure
 * dev->type must be DEVICE_TYPE_UART_HW.
 * dev->id will be mapped to /dev/ttySx (where x is the ID between 0 and 9).
 * @return Address of structure representing device or NULL on failure
 */
static int stm32_uart_open(struct device *dev)
{
    int ret;
    int fd;
    struct stm32_uart_info *uart_info;
    struct device_resource *phy_id;
    char buf[] = "/dev/ttySx";

    if (!dev) {
        return -ENODEV;
    }

    if (dev->id > 9) {
        lldbg("ERROR: invalid device id: %d\n", dev->id);
        return -EINVAL;
    }

    uart_info = device_get_private(dev);
    if (!uart_info) {
        return -ENODEV;
    }

    sem_wait(&uart_info->sem);

    if (uart_info->flags & STM32_UART_FLAG_OPEN) {
        lldbg("ERROR: already open\n");
        ret = -EBUSY;
        goto err_unlock;
    }

    /* Optional resource to identify the physical UART */
    phy_id = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS, "phy_id");
    if (uart_info->phy_id) {
        uart_info->phy_id = phy_id->start;
    }

    /* Overwrite the second to last character with the ASCII dev->id. */
    buf[sizeof(buf) - 2] = dev->id + '0';

    fd = open(buf, O_RDWR);
    if (fd < 0) {
        lldbg("ERROR: failed to open\n");
        ret = fd;
        goto err_unlock;
    }

    uart_info->fd = fd;
    uart_info->flags |= STM32_UART_FLAG_OPEN;

    ret = 0;

err_unlock:
    sem_post(&uart_info->sem);
    return ret;
}

/**
 * @brief Close UART device
 * @param dev pointer to the UART device structure
 */
static void stm32_uart_close(struct device *dev)
{
    struct stm32_uart_info *uart_info;

    uart_info = device_get_private(dev);
    if (!uart_info) {
        return;
    }

    sem_wait(&uart_info->sem);

    if (!(uart_info->flags & STM32_UART_FLAG_OPEN)) {
        lldbg("ERROR: not open\n");
        goto err_unlock;
    }

    close(uart_info->fd);

    uart_info->fd = -1;
    uart_info->flags = 0;

err_unlock:
    sem_post(&uart_info->sem);
}

/**
 * @brief probe UART device
 * @param dev pointer to the UART device structure
 * @return 0 for success, -errno for failures.
 */
static int stm32_uart_probe(struct device *dev)
{
    struct stm32_uart_info *uart_info;

    uart_info = zalloc(sizeof(*uart_info));
    if (!uart_info) {
        return -ENOMEM;
    }

    uart_info->fd = -1;
    uart_info->phy_id = STM32_UART_INVALID_RESOURCE;
    sem_init(&uart_info->sem, 0, 1);
    device_set_private(dev, uart_info);

    return 0;
}

/**
 * @brief remove UART device
 * @param dev pointer to the UART device structure
 */
static void stm32_uart_remove(struct device *dev)
{
    struct uart_info *uart_info = device_get_private(dev);
    if (uart_info) {
        device_set_private(dev, NULL);
        free(uart_info);
    }
}

const static struct device_uart_type_ops stm32_uart_type_ops = {
    .set_configuration = stm32_uart_set_configuration,
    .start_transmitter = stm32_uart_start_transmitter,
    .start_receiver    = stm32_uart_start_receiver,
};

const static struct device_driver_ops stm32_uart_driver_ops = {
    .probe    = &stm32_uart_probe,
    .remove   = &stm32_uart_remove,
    .open     = &stm32_uart_open,
    .close    = &stm32_uart_close,
    .type_ops = (struct device_uart_type_ops *)&stm32_uart_type_ops,
};

const struct device_driver stm32_uart_driver = {
    .type   = DEVICE_TYPE_UART_HW,
    .name   = "stm32_uart",
    .desc   = "stm32 uart",
    .ops    = (struct device_driver_ops *)&stm32_uart_driver_ops,
};
