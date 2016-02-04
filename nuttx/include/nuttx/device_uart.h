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

#ifndef __INCLUDE_NUTTX_DEVICE_UART_H
#define __INCLUDE_NUTTX_DEVICE_UART_H

#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/ring_buf.h>

#define DEVICE_TYPE_UART_HW                     "UART"

/* Baudrate settings */
#define BAUD_115200     115200
#define BAUD_57600      57600
#define BAUD_38400      38400
#define BAUD_19200      19200
#define BAUD_9600       9600
#define BAUD_4800       4800
#define BAUD_2400       2400
#define BAUD_1800       1800

/* Parity */
#define NO_PARITY       0x00        /* No Parity */
#define ODD_PARITY      0x01        /* Odd Parity */
#define EVEN_PARITY     0x02        /* Even Parity */
#define MARK_PARITY     0x03        /* Mark Parity */
#define SPACE_PARITY    0x04        /* Space Parity */

/* Stopbits */
#define ONE_STOP_BIT    0x00        /* 1 Stop Bit */
#define ONE5_STOP_BITS  0x01        /* 1.5 Stop Bit */
#define TWO_STOP_BITS   0x02        /* 2 Stop bit */

/* Modem control */
#define MCR_DTR         BIT(0)      /* Data Terminal Ready*/
#define MCR_RTS         BIT(1)      /* Request to Send */
#define MCR_OUT1        BIT(2)      /* Out 1 */
#define MCR_OUT2        BIT(3)      /* Out 2 */
#define MCR_LPBK        BIT(4)      /* Loop */

/* Line status */
#define LSR_DR          BIT(0)      /* Data Ready */
#define LSR_OE          BIT(1)      /* Overrun Error */
#define LSR_PE          BIT(2)      /* Parity Error */
#define LSR_FE          BIT(3)      /* Framing Error */
#define LSR_BI          BIT(4)      /* Break Interrupt */
#define LSR_THRE        BIT(5)      /* Transmitter Holding Register */
#define LSR_TEMT        BIT(6)      /* Transmitter Empty */
#define LSR_RXFE        BIT(7)      /* Error in RCVR FIFO */

/* Modem status */
#define MSR_DCTS        BIT(0)      /* Delta Clear to Send */
#define MSR_DDSR        BIT(1)      /* Delta Data Set Ready */
#define MSR_TERI        BIT(2)      /* Trailing Edge Ring Indicator */
#define MSR_DDCD        BIT(3)      /* Delta Data Carrier Detect */
#define MSR_CTS         BIT(4)      /* Clear to Send */
#define MSR_DSR         BIT(5)      /* Data Set Ready */
#define MSR_RI          BIT(6)      /* Ring Indicator */
#define MSR_DCD         BIT(7)      /* Data Carrier Detect */

/**
 * UART device driver ops.
 */
struct device_uart_type_ops {
    /** UART set_configuration() function pointer */
    int (*set_configuration)(struct device *dev, int baud, int parity,
                             int databits, int stopbit, int flow);
    /** UART get_modem_ctrl() function pointer */
    int (*get_modem_ctrl)(struct device *dev, uint8_t *modem_ctrl);
    /** UART set_modem_ctrl() function pointer */
    int (*set_modem_ctrl)(struct device *dev, uint8_t *modem_ctrl);
    /** UART get_modem_status() function pointer */
    int (*get_modem_status)(struct device *dev, uint8_t *modem_status);
    /** UART get_line_status() function pointer */
    int (*get_line_status)(struct device *dev, uint8_t *line_status);
    /** UART set_break() function pointer */
    int (*set_break)(struct device *dev, uint8_t break_on);
    /** UART attach_ms_callback() function pointer */
    int (*attach_ms_callback)(struct device *dev, void (*callback)(uint8_t ms));
    /** UART attach_ls_callback() function pointer */
    int (*attach_ls_callback)(struct device *dev, void (*callback)(uint8_t ls));
    /** UART start_transmitter() function pointer */
    int (*start_transmitter)(struct device *dev, uint8_t *buffer, int length,
                             void *dma, int *sent,
                             void (*callback)(uint8_t *buffer, int length,
                                              int error));
    /** UART stop_transmitter() function pointer */
    int (*stop_transmitter)(struct device *dev);
    /** UART start_receiver() function pointer */
    int (*start_receiver)(struct device *dev, uint8_t*buffer, int length,
                          void *dma, int *got,
                          void (*callback)(uint8_t *buffer, int length,
                                           int error));
    /** UART stop_receiver() function pointer */
    int (*stop_receiver)(struct device *dev);
    /** UART flush tx buffer */
    int (*flush_transmitter)(struct device *dev);
};

/**
 * @brief UART set_configuration function
 *
 * This function is used to set the baud rate, parity, data bit and stop bit
 * settings in the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param baud - the baud rate definition in Baud rate definition.
 * @param parity - the value of parity defined in Parity definition
 * @param databits - the number of data bits between 5 to 8 bits.
 * @param stopbits - the value stop bit defined in Stopbits definition
 * @param flow 0 for disable flow control, 1 for enable flow control.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_set_configuration(struct device *dev,
                                                int baud, int parity,
                                                int databits, int stopbit,
                                                int flow)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->set_configuration)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->set_configuration(dev, baud,
                                                                   parity,
                                                                   databits,
                                                                   stopbit,
                                                                   flow);

    return -ENOSYS;
}

/**
 * @brief UART get_modem_ctrl function
 *
 * This function is to get modem control state from the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param modem_ctrl output value as bitmask of Modem control definition.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_get_modem_ctrl(struct device *dev,
                                             uint8_t *modem_ctrl)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->get_modem_ctrl)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->get_modem_ctrl(dev,
                                                                modem_ctrl);

    return -ENOSYS;
}

/**
 * @brief UART set_modem_ctrl function
 *
 * This function is to write modem control settings to UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param modem_ctrl set value as bitmask of Modem control definition.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_set_modem_ctrl(struct device *dev,
                                             uint8_t *modem_ctrl)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->set_modem_ctrl)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->set_modem_ctrl(dev,
                                                                modem_ctrl);

    return -ENOSYS;
}

/**
 * @brief UART get_modem_status function
 *
 * This function is to get modem status from the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param modem_status output value as bitmask of Modem status definition.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_get_modem_status(struct device *dev,
                                               uint8_t *modem_status)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->get_modem_status)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->get_modem_status(dev,
                                                                  modem_status);

    return -ENOSYS;
}

/**
 * @brief UART get_line_status function
 *
 * The function is to get line status from the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param line_status output value as bitmask of Line status definition.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_get_line_status(struct device *dev,
                                              uint8_t *line_status)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->get_line_status)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->get_line_status(dev,
                                                                 line_status);

    return -ENOSYS;
}

/**
 * @brief UART set_break function
 *
 * The function is to control break state of the UART controller.
 *
 * @param dev pointer to the UART device structure
 * @param break_on break state value, it should be 0 or 1.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_set_break(struct device *dev, uint8_t break_on)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->set_break)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->set_break(dev, break_on);

    return -ENOSYS;
}

/**
 * @brief UART attach_ms_callback function
 *
 * This function registers a modem status (ms) callback function into the
 * driver.
 *
 * @param dev pointer to the UART device structure
 * @param callback null means caller doesn’t need this event.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_attach_ms_callback(struct device *dev,
                                                 void (*callback)(uint8_t ms))
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->attach_ms_callback)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->attach_ms_callback(dev,
                                                                    callback);

    return -ENOSYS;
}

/**
 * @brief UART attach_ls_callback function
 *
 * The function is to register a line status (ls) callback function into the
 * driver.
 *
 * @param dev pointer to the UART device structure
 * @param callback null means caller doesn’t need this event.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_attach_ls_callback(struct device *dev,
                                                 void (*callback)(uint8_t ls))
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->attach_ls_callback)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->attach_ls_callback(dev,
                                                                    callback);

    return -ENOSYS;
}

/**
 * @brief UART start_transmitter function
 *
 * This function is to transmit data through the UART controller.
 * It could be blocking or non-blocking and through DMA or PIO mode.
 *
 * @param dev pointer to the UART device structure
 * @param buffer pointer of the buffer to send data to UART port.
 * @param length length of data.
 * @param dma DMA handle.
 * @param sent the length of transmitted data in block mode.
 * @param callback a callback function called when transmitting finished,
 *        timeout or errors.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_start_transmitter(struct device *dev,
                        uint8_t *buffer, int length, void *dma,
                        int *sent, void (*callback)(uint8_t *buffer, int length,
                                                    int error))
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->start_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->start_transmitter(dev, buffer,
                                                                   length, dma,
                                                                   sent,
                                                                   callback);

    return -ENOSYS;
}

/**
 * @brief UART stop_transmitter function
 *
 * This function is to stop the data transmit in blocking or non-blocking mode.
 *
 * @param dev pointer to the UART device structure
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_stop_transmitter(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->stop_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->stop_transmitter(dev);

    return -ENOSYS;
}

/**
 * @brief UART start_receiver function
 *
 * The function is to receive data from UART controller. It could be
 * blocking or non-blocking and through DMA or PIO mode.
 *
 * @param dev pointer to the UART device structure
 * @param buffer pointer of the buffer to receive data from UART port.
 * @param length length of data.
 * @param dma DMA handle.
 * @param got the length of received data in blocking mode.
 * @param callback a callback function called when receiving finished, timeout
 *                 or errors.
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_start_receiver(struct device *dev,
                        uint8_t* buffer, int length, void *dma,
                        int *got, void (*callback)(uint8_t *buffer, int length,
                                                   int error))
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->start_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->start_receiver(dev, buffer,
                                                                length, dma,
                                                                got, callback);

    return -ENOSYS;
}

/**
 * @brief UART stop_receiver function
 *
 * The function is to stop the data receiving in blocking or non-blocking mode.
 *
 * @param dev pointer to the UART device structure
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_stop_receiver(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->stop_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->stop_receiver(dev);

    return -ENOSYS;
}

/**
 * @brief UART flush transmitter function
 *
 * The function ensures that data in the TX queue has been sent out
 * of the UART.
 *
 * @param dev pointer to the UART device structure
 * @return 0 for success, -errno for failures.
 */
static inline int device_uart_flush_transmitter(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->flush_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->flush_transmitter(dev);

    return -ENOSYS;
}

#endif /* __INCLUDE_NUTTX_DEVICE_UART_H */
