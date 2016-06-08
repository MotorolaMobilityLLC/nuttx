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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/device_uart.h>

#include "up_arch.h"
#include "tsb_scm.h"

#define UART_CLOCK  48000000

/* Registers and address offset */
#define UA_RBR_THR_DLL      0x00
#define UA_IER_DLH          0x04
#define UA_FCR_IIR          0x08
#define UA_LCR              0x0C
#define UA_MCR              0x10
#define UA_LSR              0x14
#define UA_MSR              0x18
#define UA_SCR              0x1C
#define UA_FAR              0x70
#define UA_TFR              0x74
#define UA_RFW              0x78
#define UA_USR              0x7C
#define UA_TFL              0x80
#define UA_RFL              0x84
#define UA_HTX              0xA4
#define UA_DMASA            0xA8
#define UA_CPR              0xF4
#define UA_UCV              0xF8
#define UA_CTR              0xFC

/*
 * UA_IER_DLH
 */
/* DLAB = 0 */
#define UA_IER_ERBFI        BIT(0)  /* Enable Received Data Available */
#define UA_IER_ETBEI        BIT(1)  /* Enable Transmit Holding Register Empty */
#define UA_IER_ELSI         BIT(2)  /* Enable Receiver Line Status Interrupt */
#define UA_IER_EDSSI        BIT(3)  /* Enable Modem Status Interrupt */
#define UA_IER_PTIME        BIT(7)  /* Programmable THRE Interrupt Mode */

/* DLAB = 1 */
#define UA_DLH_MASK         0x7F    /* Divisor Latch register */

/*
 * UA_FCR_IIR
 */
#define UA_RX_FIF0_TRIGGER_SHIFT    6
#define UA_RX_FIFO_TRIGGER_1_CHAR   0x00
#define UA_RX_FIFO_TRIGGER_1_4      0x01
#define UA_RX_FIFO_TRIGGER_1_2      0x02
#define UA_RX_FIFO_TRIGGER_2        0x03

#define UA_TX_FIF0_TRIGGER_SHIFT    4
#define UA_TX_FIFO_TRIGGER_EMPTY    0x00
#define UA_TX_FIFO_TRIGGER_2_CHAR   0x01
#define UA_TX_FIFO_TRIGGER_1_4      0x02
#define UA_TX_FIFO_TRIGGER_1_2      0x03

/*
 * UA_FCR_IIR
 */
#define UA_INTERRUPT_ID_MASK        0x0f
#define UA_INTERRUPT_ID_MS          0x00    /* modem status */
#define UA_INTERRUPT_ID_NO          0x01    /* no interrupt pending */
#define UA_INTERRUPT_ID_TX          0x02    /* THR empty */
#define UA_INTERRUPT_ID_RX          0x04    /* received data available */
#define UA_INTERRUPT_ID_LS          0x06    /* receiver line status */
#define UA_INTERRUPT_ID_BUSY        0x07    /* busy detect */
#define UA_INTERRUPT_ID_TO          0x0C    /* character timeout */

#define UA_FIFO_ENABLE              BIT(0)
#define UA_RX_FIFO_RESET            BIT(1)
#define UA_TX_FIFO_RESET            BIT(2)

/*
 * UA_LCR
 */
#define UA_LCR_MASK                 0x7C
#define UA_LCR_STOP                 BIT(2)
#define UA_LCR_PEN                  BIT(3)
#define UA_LCR_EPS                  BIT(4)
#define UA_LCR_SP                   BIT(5)
#define UA_LCR_BREAK                BIT(6)

#define UA_DLAB                     BIT(7)

#define UA_DLS_MASK                 0x03
#define UA_DLS_5_BITS               0x00
#define UA_DLS_6_BITS               0x01
#define UA_DLS_7_BITS               0x02
#define UA_DLS_8_BITS               0x03

/*
 * UA_MCR
 */
#define UA_CTS                      BIT(0)
#define UA_RTS                      BIT(1)
#define UA_OUT1                     BIT(2)
#define UA_OUT2                     BIT(3)
#define UA_LOOP_BACK                BIT(4)
#define UA_AUTO_FLOW_ENABLE         BIT(5)
#define UA_AUTO_FLOW_CTRL_N         BIT(6)

/*
 * UA_LSR
 */
#define UA_LSR_DR                   BIT(0)
#define UA_LSR_OE                   BIT(1)
#define UA_LSR_PE                   BIT(2)
#define UA_LSR_FE                   BIT(3)
#define UA_LSR_BI                   BIT(4)
#define UA_LSR_MASK                 0x1f

#define UA_LSR_THRE                 BIT(5)  /* Xmit Holding Register Empty */
#define UA_LSR_TEMT                 BIT(6)  /* Transmitter Empty bit */
#define UA_LSR_REF                  BIT(7)  /* Receive FIFO Error */

/*
 * UA_MSR
 */
#define UA_MSR_DCTS                 BIT(0)
#define UA_MSR_DDSR                 BIT(1)
#define UA_MSR_TERI                 BIT(2)
#define UA_MSR_DDCD                 BIT(3)
#define UA_MSR_CTS                  BIT(4)
#define UA_MSR_DSR                  BIT(5)
#define UA_MSR_RI                   BIT(6)
#define UA_MSR_DCD                  BIT(7)

/*
 * UA_USR
 */
#define UA_USR_BUSY                 BIT(0)  /* UART Busy */
#define UA_USR_TFNF                 BIT(1)  /* Transmit FIFO Not Full */
#define UA_USR_TFE                  BIT(2)  /* Transmit FIFO Empty */
#define UA_USR_RFNE                 BIT(3)  /* Receive FIFO Not Empty */
#define UA_USR_REF                  BIT(4)  /* Receive FIFO Full */

/* uart driver status flag */
#define TSB_UART_FLAG_OPEN          BIT(0)
#define TSB_UART_FLAG_XMIT          BIT(1)
#define TSB_UART_FLAG_RECV          BIT(2)

/* buffer structure */
struct uart_buffer
{
  int16_t head;    /* Index to the head [IN] index in the buffer */
  int16_t tail;    /* Index to the tail [OUT] index in the buffer */
  uint8_t          *buffer; /* Pointer to the allocated buffer memory */
};

/**
 * struct tsb_uart_info - the driver internal information.
 */
struct tsb_uart_info {
    /** device driver handler */
    struct device   *dev;
    /** UART driver flags */
    uint32_t        flags;
    /** UART register base */
    uint32_t        reg_base;
    /** IRQ number */
    int             uart_irq;
    /** The last line errors */
    uint32_t        line_err;
    /** Keeping FCR value, FCR is write-only register. */
    uint8_t         fcr;
    /** Transmit buffer structure */
    struct uart_buffer xmit;
    /** Receive buffer structure */
    struct uart_buffer recv;
    /** transmit semaphore for blocking mode */
    sem_t           tx_sem;
    /** Receive semaphore for blocking mode */
    sem_t           rx_sem;
    /** Modem status callback function */
    void            (*ms_callback)(uint8_t ms);
    /** Line status callback function */
    void            (*ls_callback)(uint8_t ls);
    /** Receive callback function */
    void            (*rx_callback)(uint8_t *buffer, int length, int error);
    /** Transmit callback function */
    void            (*tx_callback)(uint8_t *buffer, int length, int error);
};

/** device structure for interrupt handler */
static struct device *saved_dev;

/**
 * @brief Get register value.
 *
 * @param base The UART register base address.
 * @param offset The UART register offset address.
 * @return Register value.
 */
static uint32_t ua_getreg(uint32_t base, uint32_t offset)
{
    return getreg32(base + offset);
}

/**
 * @brief Put value to register.
 *
 * @param base The UART register base address.
 * @param offset The UART register offset address.
 * @param value The value to put.
 * @return None.
 */
static void ua_putreg(uint32_t base, uint32_t offset, uint32_t value)
{
    putreg32(value, base + offset);
}

/**
 * @brief Set bit or bits in the register.
 *
 * @param base The UART register base address.
 * @param offset The UART register offset address.
 * @param bitmask The bit mask.
 * @return None.
 */
static void ua_reg_bit_set(uint32_t reg, uint32_t offset, uint8_t bitmask)
{
    uint8_t regvalue = ua_getreg(reg, offset);
    regvalue |= bitmask;
    ua_putreg(reg, offset, regvalue);
}

/**
 * @brief Clean bit in the register.
 *
 * @param base The UART register base address.
 * @param offset The UART register offset address.
 * @param bitmask The bit mask.
 * @return None.
 */
static void ua_reg_bit_clr(uint32_t reg, uint32_t offset, uint8_t bitmask)
{
    uint8_t regvalue = ua_getreg(reg, offset);
    regvalue &= ~bitmask;
    ua_putreg(reg, offset, regvalue);
}

#ifdef DEBUG_ON
/**
 * @brief Dump UART registers value.
 *
 * @param base The UART register base address.
 * @return None.
 */
static void dump_regs(uint32_t base)
{
    /* divisor latch */
    ua_reg_bit_set(base, UA_LCR, UA_DLAB);
    lldbg("DLL reg: 0x%08p\n", ua_getreg(base, UA_RBR_THR_DLL));
    lldbg("DLH reg: 0x%08p\n", ua_getreg(base, UA_IER_DLH));
    ua_reg_bit_clr(base, UA_LCR, UA_DLAB);
    /* interrupts enable */
    lldbg("IER reg: 0x%08p\n", ua_getreg(base, UA_IER_DLH));
    /* interrupt id */
    lldbg("IIR reg: 0x%08p\n", ua_getreg(base, UA_FCR_IIR));
    /* line control */
    lldbg("LCR reg: 0x%08p\n", ua_getreg(base, UA_LCR));
    /* modem control */
    lldbg("MCR reg: 0x%08p\n", ua_getreg(base, UA_MCR));
    /* line status */
    lldbg("LSR reg: 0x%08p\n", ua_getreg(base, UA_LSR));
    /* modem status */
    lldbg("MSR reg: 0x%08p\n", ua_getreg(base, UA_MSR));
}
#endif

/**
 * @brief Set divisor register.
 *
 * @param base The UART register base address.
 * @param divisor The divisor value.
 * @return None.
 */
static void ua_set_divisor(uint32_t base, uint32_t divisor)
{
    /* Set DLAB */
    ua_reg_bit_set(base, UA_LCR, UA_DLAB);
    /* Set devisor */
    ua_putreg(base, UA_RBR_THR_DLL, divisor & 0xff);
    ua_putreg(base, UA_IER_DLH, (divisor >> 8) & 0xff);
    /* Clear DLAB */
    ua_reg_bit_clr(base, UA_LCR, UA_DLAB);
}

/**
 * @brief Get interrupt id from controller.
 *
 * @param base The UART register base address.
 * @return The interrupt id.
 */
static uint8_t ua_get_interrupt_id(uint32_t base)
{
    return (ua_getreg(base, UA_FCR_IIR) & UA_INTERRUPT_ID_MASK);
}

/**
 * @brief Check the transmit FIFO empty.
 *
 * @param base The UART register base address.
 * @return 0 for not empty, 1 for empty.
 */
static uint8_t ua_is_tx_fifo_empty(uint32_t base)
{
    return (ua_getreg(base, UA_USR) & UA_USR_TFE) ? 1 : 0;
}

/**
 * @brief Check the transmit FIFO full.
 *
 * @param base The UART register base address.
 * @return 0 for not full, 1 for full.
 */
static uint8_t ua_is_tx_fifo_full(uint32_t base)
{
    return (ua_getreg(base, UA_USR) & UA_USR_TFNF) ? 0 : 1;
}

/**
 * @brief Check the receive FIFO empty.
 *
 * @param base The UART register base address.
 * @return 0 for not empty, 1 for empty.
 */
static uint8_t ua_is_rx_fifo_empty(uint32_t base)
{
    return (ua_getreg(base, UA_USR) & UA_USR_RFNE) ? 0 : 1;
}

/**
 * @brief Transmit characters from buffer to FIFO.
 *
 * This function put the character from buffer to FIFO until the buffer is
 * empty or FIFO is full. If buffer is empty, it calls the up layer callback
 * function.
 *
 * @param uart_info The UART driver info structure.
 * @return None.
 */
static void ua_xmitchars(struct tsb_uart_info *uart_info)
{
    /* TX is not enabled, nothing to transmit */
    if (!(uart_info->flags & TSB_UART_FLAG_XMIT)) {
        return;
    }

    while (!ua_is_tx_fifo_full(uart_info->reg_base)) {
        if (uart_info->xmit.head == uart_info->xmit.tail) {
            /* Disable transmit interrupt */
            ua_reg_bit_clr(uart_info->reg_base, UA_IER_DLH, UA_IER_ETBEI);
            if (uart_info->tx_callback) {
                uart_info->flags &= ~TSB_UART_FLAG_XMIT;
                uart_info->tx_callback(uart_info->xmit.buffer,
                                       uart_info->xmit.head, 0);
            }
            else {
                sem_post(&uart_info->tx_sem);
            }
            return;
        }
        ua_putreg(uart_info->reg_base, UA_RBR_THR_DLL,
                  uart_info->xmit.buffer[uart_info->xmit.head]);
        uart_info->xmit.head++;
    }
}

/**
 * @brief Receive characters from FIFO to buffer.
 *
 * This function get the character from FIFO to buffer until the buffer is full
 * or FIFO is empty. If buffer is full, it calls the up layer callback function.
 *
 * tsb_uart_start_receiver() get one buffer from protocol, it pass the buffer
 * information to uart_info and enable the RX interrupt. When there is data in
 * FIFO, this function is called by interrupt handler for processing data.
 *
 * @param uart_info The UART driver info structure.
 * @param int_id The interrupt ID from register.
 * @return None.
 */
static void ua_recvchars(struct tsb_uart_info *uart_info, uint8_t int_id)
{
    /* RX is not enabled, ignore received characters */
    if (!(uart_info->flags & TSB_UART_FLAG_RECV)) {
        while (!ua_is_rx_fifo_empty(uart_info->reg_base)) {
            ua_getreg(uart_info->reg_base, UA_RBR_THR_DLL);
        }

        return;
    }

    while (!ua_is_rx_fifo_empty(uart_info->reg_base)) {
        uart_info->recv.buffer[uart_info->recv.head] =
                        ua_getreg(uart_info->reg_base, UA_RBR_THR_DLL);
        uart_info->recv.head++;
        if (uart_info->recv.head == uart_info->recv.tail) {
            break;
        }
    }

    /* Disable receive interrupt */
    ua_reg_bit_clr(uart_info->reg_base, UA_IER_DLH,
                           UA_IER_ERBFI | UA_IER_ELSI);

    if (uart_info->rx_callback) {
        uart_info->flags &= ~TSB_UART_FLAG_RECV;
        uart_info->rx_callback(uart_info->recv.buffer,
                               uart_info->recv.head,
                               uart_info->line_err);
    } else {
        sem_post(&uart_info->rx_sem);
    }
}

/**
 * @brief The UART interrupt handler.
 *
 * This function is attached as interrupt service when driver init.
 *
 * @param irq The IRQ number from OS.
 * @param context The context for this interrupt.
 * @return None.
 */
static int ua_irq_handler(int irq, void *context)
{
    struct tsb_uart_info *uart_info = device_get_private(saved_dev);
    uint8_t interrupt_id = 0;
    uint8_t status = 0;

    while (1) {
        interrupt_id = ua_get_interrupt_id(uart_info->reg_base);
        if (interrupt_id == UA_INTERRUPT_ID_NO) {
            /* no interrupt pending */
            break;
        }

        switch (interrupt_id) {
        case UA_INTERRUPT_ID_MS:
            status = ua_getreg(uart_info->reg_base, UA_MSR);
            if (uart_info->ms_callback) {
                uart_info->ms_callback(status);
            }
            break;
        case UA_INTERRUPT_ID_LS:
            status = ua_getreg(uart_info->reg_base, UA_LSR);
            uart_info->line_err = status & (UA_LSR_OE | UA_LSR_PE | UA_LSR_FE |
                                            UA_LSR_BI);
            if (uart_info->ls_callback) {
                uart_info->ls_callback(status);
            }
            break;
        case UA_INTERRUPT_ID_TX:
            ua_xmitchars(uart_info);
            break;
        case UA_INTERRUPT_ID_TO:
        case UA_INTERRUPT_ID_RX:
            ua_recvchars(uart_info, interrupt_id);
            break;
        }
    }

    return 0;
}

/**
 * @brief Retrieve UART resource from driver core.
 *
 * This function get the UART register base and irq number form driver core
 * infrastructure.
 *
 * @param dev The pointer to device structure.
 * @param uart_info The UART driver information.
 * @return 0 for success, -errno for failures.
 */
static int tsb_uart_extract_resources(struct device *dev,
                                     struct tsb_uart_info * uart_info)
{
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS, "reg_base");
    if (!r) {
        return -EINVAL;
    }
    uart_info->reg_base = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_IRQ, "irq_uart");
    if (!r) {
        return -EINVAL;
    }
    uart_info->uart_irq = (int)r->start;

    return 0;
}

/**
 * @brief Low level uart init
 *
 * This function is to enable clock and pinshare when driver bring up.
 *
 * @param baud The baud rate.
 * @return 0 for success, -errno for failures.
 */
int tsb_uart_lowsetup(int baud)
{
    int i;
    uint32_t divisor;
    int retval;

    if (!baud) {
        return -EINVAL;
    }

    retval = tsb_request_pinshare(TSB_PIN_GPIO9 | TSB_PIN_UART_RXTX | TSB_PIN_UART_CTSRTS);
    if (retval) {
        lowsyslog("UART: cannot get ownership of UART pin.\n");
        return retval;
    }

    /* enable UART RX/TX pins */
    tsb_set_pinshare(TSB_PIN_UART_RXTX);

    /* enable UART CTS/RTS pins */
    tsb_clr_pinshare(TSB_PIN_GPIO9);
    tsb_set_pinshare(TSB_PIN_UART_CTSRTS);

    /* enable UART clocks */
    tsb_clk_enable(TSB_CLK_UARTP);
    tsb_clk_enable(TSB_CLK_UARTS);

    /* reset UART module */
    tsb_reset(TSB_RST_UARTP);
    tsb_reset(TSB_RST_UARTS);

    /*
     * The controller requires "several cycles" after reset to stabilize before
     * register writes will work. Try this a few times.
     */
    /* baud rate = serial clock freq / (16 * divider) */
    divisor = (UART_CLOCK >> 4) / baud;
    for (i = 0; i < 3; i++) {
        ua_set_divisor(UART_BASE, divisor);

        /* Set no parity, 8 data bits and 1 stop bit as default setting. */
        ua_reg_bit_clr(UART_BASE, UA_LCR, 0xff);
        ua_reg_bit_set(UART_BASE, UA_LCR, UA_DLS_8_BITS);
        ua_reg_bit_set(UART_BASE, UA_FCR_IIR,
                       UA_FIFO_ENABLE | UA_RX_FIFO_RESET | UA_TX_FIFO_RESET);
    }

    return 0;
}

/**
 * @brief Set UART configurations for baudrate, parity, etc.
 *
 * This function is used to set the baud rate, parity, data bit and stop bit
 * settings in the UART controller.
 *
 * @param dev The pointer to the UART device structure.
 * @param baud The baud rate definition in Baud rate definition.
 * @param parity The value of parity defined in Parity definition.
 * @param databits The number of data bits between 5 to 8 bits.
 * @param stopbit The value stop bit defined in Stopbits definition
 * @param flow 0 for disable flow control, 1 for enable flow control.
 * @return 0 for success, -errno for failures.
 */
static int tsb_uart_set_configuration(struct device *dev, int baud, int parity,
                                      int databits, int stopbit, int flow)
{
    struct tsb_uart_info *uart_info;
    uint32_t divisor;
    uint8_t lcr;

    if (!dev) {
        return -EINVAL;
    }

    if (!baud) {
         return -EINVAL;
    }

    uart_info = device_get_private(dev);

    /* TX and RX FIFO reset */
    uart_info->fcr = 0;

    /* Clean all FIFO setting */
    ua_putreg(uart_info->reg_base,
              UA_FCR_IIR, UA_TX_FIFO_RESET | UA_RX_FIFO_RESET);

    /* Set TX and RX FIFO to 1/2 */
    uart_info->fcr = UA_RX_FIFO_TRIGGER_1_2 << UA_RX_FIF0_TRIGGER_SHIFT |
                     UA_TX_FIFO_TRIGGER_1_2 << UA_TX_FIF0_TRIGGER_SHIFT;
    ua_putreg(uart_info->reg_base, UA_FCR_IIR, uart_info->fcr);

    /* Set data bits */
    lcr = 0;    /* stopbit setting depend on databit */
    switch (databits) {
    case 5:
        lcr = UA_DLS_5_BITS;
        break;
    case 6:
        lcr = UA_DLS_6_BITS;
        break;
    case 7 :
        lcr = UA_DLS_7_BITS;
        break;
    case 8 :
        lcr = UA_DLS_8_BITS;
        break;
    default:
        return -EINVAL;
    }
    ua_reg_bit_clr(uart_info->reg_base, UA_LCR, UA_DLS_MASK);
    ua_reg_bit_set(uart_info->reg_base, UA_LCR, lcr);

    /* Set stop bit */
    switch (stopbit) {
    case ONE_STOP_BIT:
        ua_reg_bit_clr(uart_info->reg_base, UA_LCR, UA_LCR_STOP);
        break;
    case ONE5_STOP_BITS:
        if (lcr != UA_DLS_5_BITS) {
            return -EINVAL;
        }
        ua_reg_bit_set(uart_info->reg_base, UA_LCR, UA_LCR_STOP);
        break;
    case TWO_STOP_BITS:
        if (lcr == UA_DLS_5_BITS) {
            return -EINVAL;
        }
        ua_reg_bit_set(uart_info->reg_base, UA_LCR, UA_LCR_STOP);
        break;
    default:
        return -EINVAL;
    }

    /* Set parity setting */
    lcr = 0;
    if (parity == ODD_PARITY) {
        lcr |= UA_LCR_PEN;
    } else if (parity == EVEN_PARITY) {
        lcr |= (UA_LCR_PEN|UA_LCR_EPS);
    }
    ua_reg_bit_clr(uart_info->reg_base, UA_LCR, (UA_LCR_PEN|UA_LCR_EPS));
    ua_reg_bit_set(uart_info->reg_base, UA_LCR, lcr);

    /* set baud rate divisor */
    /* baud rate = serial clock freq / (16 * divider) */
    divisor = (UART_CLOCK >> 4) / baud;
    ua_set_divisor(uart_info->reg_base, divisor);

    /* Enable TX and RX FIFO */
    uart_info->fcr |= UA_FIFO_ENABLE;
    ua_putreg(uart_info->reg_base, UA_FCR_IIR, uart_info->fcr);
    /* Programmable THRE interrupt mode enable */
    ua_reg_bit_set(uart_info->reg_base, UA_IER_DLH, UA_IER_PTIME);

    /* Set auto flow control */
    if (flow) {
        ua_reg_bit_set(uart_info->reg_base, UA_MCR, UA_AUTO_FLOW_ENABLE|UA_RTS);
    } else {
        ua_reg_bit_clr(uart_info->reg_base, UA_MCR, UA_AUTO_FLOW_ENABLE|UA_RTS);
    }

    return 0;
}

/**
* @brief Get Modem control state.
*
* This function is to get modem control state from the UART controller.
*
* @param dev The pointer to the UART device structure.
* @param modem_ctrl The output value as bitmask of Modem control definition.
* @return O for success, -errno for failures.
*/
static int tsb_uart_get_modem_ctrl(struct device *dev, uint8_t *modem_ctrl)
{
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL || modem_ctrl == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    *modem_ctrl = ua_getreg(uart_info->reg_base, UA_MCR);

    return 0;
}

/**
 * @brief Set Modem control state
 *
 * This function is to write modem control settings to UART controller.
 *
 * @param dev The pointer to the UART device structure.
 * @param modem_ctrl The value as bitmask of Modem control definition.
 * @return 0 for success, -errno for failures.
 */
static int tsb_uart_set_modem_ctrl(struct device *dev, uint8_t *modem_ctrl)
{
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL || modem_ctrl == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    ua_putreg(uart_info->reg_base, UA_MCR, *modem_ctrl);

    return 0;
}

/**
* @brief Get modem status
*
* This function is to get modem status from the UART controller.
*
* @param dev The pointer to the UART device structure.
* @param modem_status The output value as bitmask of Modem status definition.
* @return 0 for success, -errno for failures.
*/
static int tsb_uart_get_modem_status(struct device *dev, uint8_t *modem_status)
{
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL || modem_status == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    *modem_status = ua_getreg(uart_info->reg_base, UA_MSR);

    return 0;
}

/**
* @brief Get line status.
*
* The function is to get line status from the UART controller.
*
* @param dev The pointer to the UART device structure.
* @param line_status The output value as bitmask of Line status definition.
* @return 0 for success, -errno for failures.
*/
static int tsb_uart_get_line_status(struct device *dev, uint8_t *line_status)
{
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL || line_status == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    /*
     * UART_LSR is read & clean register, so this funtion return saved value
     * to user, and only the error bits. The FIFO operation bits only for
     * driver internal use.
     */
    *line_status = uart_info->line_err;
    uart_info->line_err = 0;

    return 0;
}

/**
* @brief Control break state
*
* The function is to control break state of the UART controller.
*
* @param dev The pointer to the UART device structure.
* @param break_on The break state value, it should be 0 or 1.
* @return 0 for success, -errno for failures.
*/
static int tsb_uart_set_break(struct device *dev, uint8_t break_on)
{
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    if (break_on) {
        ua_reg_bit_set(uart_info->reg_base, UA_LCR, UA_LCR_BREAK);
    } else {
        ua_reg_bit_clr(uart_info->reg_base, UA_LCR, UA_LCR_BREAK);
    }

    return 0;
}

/**
* @brief Attach the modem status change callback.
*
* This function registers a modem status (ms) callback function into the driver.
*
* @param dev The pointer to the UART device structure.
* @param callback Null means caller doesn’t need this event.
* @return 0 for success, -errno for failures.
*/
static int tsb_uart_attach_ms_callback(struct device *dev,
                                       void (*callback)(uint8_t ms))
{
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    if (callback == NULL) {
        /* Disable modem status interrupt. */
        ua_reg_bit_clr(uart_info->reg_base, UA_IER_DLH, UA_IER_EDSSI);
        uart_info->ms_callback = NULL;
        return 0;
    }

    /* Enable modem status interrupt. */
    ua_reg_bit_set(uart_info->reg_base, UA_IER_DLH, UA_IER_EDSSI);
    uart_info->ms_callback = callback;
    return 0;
}

/**
* @brief Attach the line status change callback.
*
* The function is to register a line status (ls) callback function into the
* driver.
*
* @param dev The pointer to the UART device structure.
* @param callback Null means caller doesn’t need this event.
* @return 0 for success, -errno for failures.
*/
static int tsb_uart_attach_ls_callback(struct device *dev,
                                       void (*callback)(uint8_t ls))
{
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    uart_info->ls_callback = callback;

    return 0;
}

/**
* @brief Start the transmitter.
*
* This function is to transmit data through the UART controller.
* It could be blocking or non-blocking and through DMA or PIO mode.
*
* @param dev The pointer to the UART device structure.
* @param buffer The pointer of the buffer to send data to UART port.
* @param length The length of data.
* @param dma The DMA handle.
* @param sent The length of transmitted data in block mode.
* @param callback A callback function called when transmitting finished, timeout
*                 or errors.
* @return 0 for success, -errno for failures.
*/
static int tsb_uart_start_transmitter(struct device *dev, uint8_t *buffer,
                                      int length, void *dma, int *sent,
                                      void (*callback)(uint8_t *buffer,
                                                       int length, int error))
{
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    if (uart_info->flags & TSB_UART_FLAG_XMIT) {
        return -EBUSY;
    }

    uart_info->flags |= TSB_UART_FLAG_XMIT;

    uart_info->xmit.buffer = buffer;
    uart_info->xmit.head = 0;
    uart_info->xmit.tail = length;
    uart_info->tx_callback = callback;

    /* Enable transmit interrupt */
    ua_reg_bit_set(uart_info->reg_base, UA_IER_DLH, UA_IER_ETBEI);

    if (!uart_info->tx_callback) {
        sem_wait(&uart_info->tx_sem);
        if (sent) {
            *sent = uart_info->xmit.head;
        }
        uart_info->flags &= ~TSB_UART_FLAG_XMIT;
    }

    return 0;
}

/**
* @brief Stop the transmitter.
*
* This function is to stop the data transmit in blocking or non-blocking mode.
*
* @param dev The pointer to the UART device structure.
* @return 0 for success, -errno for failures.
*/
static int tsb_uart_stop_transmitter(struct device *dev)
{
    struct tsb_uart_info *uart_info = NULL;
    irqstate_t flags;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    if (!(uart_info->flags & TSB_UART_FLAG_XMIT)) {
        return -EINVAL;
    }

    flags = irqsave();

    /* Disable transmit interrupt. */
    ua_reg_bit_clr(uart_info->reg_base, UA_IER_DLH, UA_IER_ETBEI);
    /* Clean FIFO */
    uart_info->fcr |= UA_TX_FIFO_RESET;
    ua_putreg(uart_info->reg_base, UA_FCR_IIR, uart_info->fcr);
    uart_info->fcr &= ~UA_TX_FIFO_RESET;

    irqrestore(flags);

    if (uart_info->tx_callback) {
        uart_info->flags &= ~TSB_UART_FLAG_XMIT;
        uart_info->tx_callback(uart_info->xmit.buffer,
                               uart_info->xmit.head, 0);
    } else {
        sem_post(&uart_info->tx_sem);
    }

    return 0;
}

/**
* @brief Start the receiver.
*
* The function is to receive data from UART controller. It could be
* blocking or non-blocking and through DMA or PIO mode.
*
* @param dev The pointer to the UART device structure
* @param buffer The pointer of the buffer to receive data from UART port.
* @param length The length of data.
* @param dma The DMA handle.
* @param got The length of received data in blocking mode.
* @param callback A callback function called when receiving finished, timeout
*                 or errors.
* @return 0 for success, -errno for failures.
*/
static int tsb_uart_start_receiver(struct device *dev, uint8_t *buffer,
                                   int length, void *dma, int *got,
                                   void (*callback)(uint8_t *buffer, int length,
                                                    int error))
{
    struct tsb_uart_info *uart_info = NULL;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    if (uart_info->flags & TSB_UART_FLAG_RECV) {
        return -EBUSY;
    }

    uart_info->flags |= TSB_UART_FLAG_RECV;

    uart_info->recv.buffer = buffer;
    uart_info->recv.head = 0;
    uart_info->recv.tail = length;
    uart_info->rx_callback = callback;
    uart_info->line_err = 0;

    /* Enable receive interrupt */
    ua_reg_bit_set(uart_info->reg_base, UA_IER_DLH, UA_IER_ERBFI | UA_IER_ELSI);

    if (!uart_info->rx_callback) {
        sem_wait(&uart_info->rx_sem);
        if (got) {
            *got = uart_info->recv.head;
        }
        uart_info->flags &= ~TSB_UART_FLAG_RECV;
    }

    return 0;
}

/**
* @brief Stop the receiver.
*
* The function is to stop the data receiving in blocking or non-blocking mode.
*
* @param dev The pointer to the UART device structure.
* @return 0 for success, -errno for failure.
*/
static int tsb_uart_stop_receiver(struct device *dev)
{
    struct tsb_uart_info *uart_info = NULL;
    irqstate_t flags;

    if (dev == NULL) {
        return -EINVAL;
    }

    uart_info = device_get_private(dev);

    if (!(uart_info->flags & TSB_UART_FLAG_RECV)) {
        return -EINVAL;
    }

    flags = irqsave();

    /* Disable receive interrupt. */
    ua_reg_bit_clr(uart_info->reg_base, UA_IER_DLH, UA_IER_ERBFI | UA_IER_ELSI);
    /* Clean FIFO */
    uart_info->fcr |= UA_RX_FIFO_RESET;
    ua_putreg(uart_info->reg_base, UA_FCR_IIR, uart_info->fcr);
    uart_info->fcr &= ~UA_RX_FIFO_RESET;

    irqrestore(flags);

    if (uart_info->rx_callback) {
        uart_info->flags &= ~TSB_UART_FLAG_RECV;
        uart_info->rx_callback(uart_info->recv.buffer,
                               uart_info->recv.head, uart_info->line_err);
    } else {
        sem_post(&uart_info->rx_sem);
    }

    return 0;
}

/**
* @brief Flush the transmitter.
*
* The function ensures that data in the TX queue has been sent out
* of the UART.
*
* @param dev The pointer to the UART device structure.
* @return 0 for success, -errno for failure.
*/
static int tsb_uart_flush_transmitter(struct device *dev)
{
    struct tsb_uart_info *uart_info = NULL;
    int i;

    if (dev == NULL)
        return -EINVAL;

    uart_info = device_get_private(dev);

    if (!(uart_info->flags & TSB_UART_FLAG_OPEN))
        return -EBUSY;

    /* Ensure that the TX FIFO is emptied.
     * At an assumed baud rate of 115200, 8-N-1 configuration,
     * a FIFO depth of 32 characters would take 2.7 ms to empty.
     */
    for (i = 0; i < 6; i++) {
        if (ua_is_tx_fifo_empty(uart_info->reg_base))
            return 0;
        usleep(500);
    }
    return -EAGAIN;
}

/**
* @brief The device open function.
*
* This function is called when protocol preparing to use the driver ops
* in initial stage. It is called after probe() was invoked by the system.
* The function checks whether the driver is already open or not. If it was
* opened, it returns driver busy error number, otherwise it keeps a flag to
* identify the driver was opened and returns success.
*
* @param dev The pointer to the UART device structure.
* @return 0 for success, -errno for failures.
*/
static int tsb_uart_dev_open(struct device *dev)
{
    struct tsb_uart_info *uart_info = device_get_private(dev);
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    if (uart_info->flags & TSB_UART_FLAG_OPEN) {
        ret = -EBUSY;
        goto err_irqrestore;
    }

    uart_info->flags = TSB_UART_FLAG_OPEN;

err_irqrestore:
    irqrestore(flags);

    return ret;
}

/**
* @brief The device close function.
*
* This function is called when protocol no longer using this driver.
* The driver must be opened before calling this function.
*
* @param dev The pointer to the UART device structure.
* @return None.
*/
static void tsb_uart_dev_close(struct device *dev)
{
    struct tsb_uart_info *uart_info = device_get_private(dev);
    irqstate_t flags;

    flags = irqsave();

    if (!(uart_info->flags & TSB_UART_FLAG_OPEN)) {
        goto err_irqrestore;
    }

    if (uart_info->flags & TSB_UART_FLAG_XMIT) {
        tsb_uart_stop_transmitter(dev);
    }

    if (uart_info->flags & TSB_UART_FLAG_RECV) {
        tsb_uart_stop_receiver(dev);
    }

    uart_info->flags = 0;

err_irqrestore:
    irqrestore(flags);
}

/**
* @brief The device probe function.
*
* This function is called by the system to register this driver when the
* system boots up.This function allocates memory for saving driver internal
* information data, attaches the interrupt handlers to IRQs of the controller
* and configures the pin settings of the UART controller.
*
* @param dev The pointer to the UART device structure.
* @return 0 for success, -errno for failure.
*/
static int tsb_uart_dev_probe(struct device *dev)
{
    struct tsb_uart_info *uart_info;
    irqstate_t flags;
    int ret;

    /* Configure the UART clock and pinshare */
    ret = tsb_uart_lowsetup(BAUD_115200);
    if (ret) {
        return ret;
    }

    uart_info = zalloc(sizeof(*uart_info));
    if (!uart_info)
        goto err_malloc_info;

    llvdbg("LL uart info struct: 0x%08p\n", uart_info);

    ret = tsb_uart_extract_resources(dev, uart_info);
    if (ret)
        goto err_free_info;

    ret = sem_init(&uart_info->tx_sem, 0, 0);
    if (ret) {
        goto err_free_info;
    }

    ret = sem_init(&uart_info->rx_sem, 0, 0);
    if (ret) {
        goto err_destroy_tx_sem;
    }

    flags = irqsave();

    ret = irq_attach(uart_info->uart_irq, ua_irq_handler);
    if (ret) {
        goto err_destroy_rx_sem;
    }

    /* Disable all interrupts */
    ua_reg_bit_clr(uart_info->reg_base,
                   UA_IER_DLH, UA_IER_ERBFI | UA_IER_ETBEI | UA_IER_ELSI |
                   UA_IER_EDSSI | UA_IER_PTIME);

    up_enable_irq(uart_info->uart_irq);

    uart_info->dev = dev;
    device_set_private(dev, uart_info);
    saved_dev = dev;

    irqrestore(flags);

    return ret;

err_destroy_rx_sem:
    irqrestore(flags);
    sem_destroy(&uart_info->rx_sem);
err_destroy_tx_sem:
    sem_destroy(&uart_info->tx_sem);
err_free_info:
    free(uart_info);
err_malloc_info:
    tsb_release_pinshare(TSB_PIN_UART_RXTX | TSB_PIN_UART_CTSRTS);

    return ret;
}

/**
* @brief The device remove function.
*
* This function is called by the system to unregister this driver. It
* must be called after probe() and open(). It detaches IRQ handlers and frees
* the internal information memory space.
*
* @param dev The pointer to the UART device structure.
* @return None.
*/
static void tsb_uart_dev_remove(struct device *dev)
{
    struct tsb_uart_info *uart_info = device_get_private(dev);
    irqstate_t flags;

    flags = irqsave();

    sem_destroy(&uart_info->rx_sem);

    sem_destroy(&uart_info->tx_sem);

    irq_detach(uart_info->uart_irq);

    saved_dev = NULL;
    device_set_private(dev, NULL);

    irqrestore(flags);

    tsb_release_pinshare(TSB_PIN_UART_RXTX | TSB_PIN_UART_CTSRTS);

    free(uart_info);
}

static struct device_uart_type_ops tsb_uart_type_ops = {
    .set_configuration  = tsb_uart_set_configuration,
    .get_modem_ctrl     = tsb_uart_get_modem_ctrl,
    .set_modem_ctrl     = tsb_uart_set_modem_ctrl,
    .get_modem_status   = tsb_uart_get_modem_status,
    .get_line_status    = tsb_uart_get_line_status,
    .set_break          = tsb_uart_set_break,
    .attach_ms_callback = tsb_uart_attach_ms_callback,
    .attach_ls_callback = tsb_uart_attach_ls_callback,
    .start_transmitter  = tsb_uart_start_transmitter,
    .stop_transmitter   = tsb_uart_stop_transmitter,
    .start_receiver     = tsb_uart_start_receiver,
    .stop_receiver      = tsb_uart_stop_receiver,
    .flush_transmitter  = tsb_uart_flush_transmitter,
};

static struct device_driver_ops tsb_uart_driver_ops = {
    .probe          = tsb_uart_dev_probe,
    .remove         = tsb_uart_dev_remove,
    .open           = tsb_uart_dev_open,
    .close          = tsb_uart_dev_close,
    .type_ops       = &tsb_uart_type_ops,
};

struct device_driver tsb_uart_driver = {
    .type      = DEVICE_TYPE_UART_HW,
    .name       = "tsb_uart",
    .desc       = "TSB UART Driver",
    .ops        = &tsb_uart_driver_ops,
};

