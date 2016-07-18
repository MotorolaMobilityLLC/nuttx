/*
 * Copyright (c) 2016 Motorola Mobility, LLC
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
 *
 * Driver to tunnel I2S to Unipro or Unipro to I2S.  This is different
 * from the general purpose I2S driver since it only moves data to and
 * from Unipro.  It also uses DMA for most operations.
 *
 * Instead of using a single circular buffer, it uses two buffers in
 * a ping pong configuration.  The fist buffer is receiving data while
 * the second is being used to transmit the data.  If the end of the
 * transmit buffer is reached before the next buffer is ready, the last
 * sample is repeated.  If a receive buffer is filled before the transmit
 * is completed then a sample is removed.
 *
 * In the case of I2S data being transmitted, the buffer will be empty once the
 * TSB I2S hardware buffer is filled.  This buffer is 256 bytes.  The hardware
 * itself has no way to monitor the buffer to determine how may samples remain
 * before it will be empty.  In order to provide some measurable method of
 * determining how many characters have been sent the software buffer must be
 * larger than 256 bytes and a multiple of the sample size.
 *
 *    AP            APBA          APBE         Device
 * +------+       +------+      +------+      +------+
 * |i2s tx|------>|i2s rx|      |i2s tx|----->|i2s rx|
 * |i2s rx|<------|i2s tx|      |i2s rx|<-----|i2s tx|
 * |i2s ck|------>|i2s ck|      |i2s ck|----->|i2s ck|
 * |i2s wc|------>|i2s wc|      |i2s wc|----->|i2s wc|
 * |      |       |unipro|<====>|unipro|      |      |
 * +------+       +------+      +------+      +------+
 *
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <reglog.h>
#include <up_arch.h>

#include <arch/chip/unipro_p2p.h>
#include <nuttx/arch.h>
#include <nuttx/bufram.h>
#include <nuttx/device_dma.h>
#include <nuttx/device_pll.h>
#include <nuttx/irq.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/i2s_tunnel/i2s_tunnel.h>
#include <nuttx/i2s_tunnel/i2s_unipro.h>

#include "tsb_dma.h"
#include "tsb_fr_tmr.h"
#include "tsb_i2s_regs.h"
#include "tsb_scm.h"
#include "tsb_tmr.h"

/* Allow for local debugging without all of the clutter of global debugging. */
#ifndef dbg
# include <stdio.h>
# define EXTRA_FMT "%s: "
# define EXTRA_ARG ,__FUNCTION__
# define dbg(format, ...) \
  syslog(EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)
#endif

/**
 * @brief The cport to be used for all I2S tunneling.
 */
#define TSB_I2S_UNIPRO_TUNNEL_CPORTID CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL_CPORT_ID

/**
 * @brief The number of bytes in each of the software buffers.
 *
 * The bigger the buffers are the more robust the drivers handling of jitter
 * will be, but more audio latency will be introduced to the system.  The total
 * memory used is this number times 4 due to the ping and pong buffers for both
 * RX and TX.
 *
 * It is recommended to define TSB_I2S_UNIPRO_TUNNEL_QUEUE_UNIPRO_SEND if this
 * value exceeds 512.
 *
 * @note This buffer must be bigger than the hardware I2S FIFO for the
 *       code to operate correctly.  This is due to a lack of a FIFO level
 *       register and/or the ability to set the DMA/IRQ level.  It is also best
 *       if it is a multipe of the largest sample size times 2 (for stereo).
 */
#define TSB_I2S_UNIPRO_TUNNEL_BUF_SZ (504)
#if (CPORT_BUF_SIZE < TSB_I2S_UNIPRO_TUNNEL_BUF_SZ)
# error "TSB_I2S_UNIPRO_TUNNEL_BUF_SZ must be less than or equal to CPORT_BUF_SIZE."
#endif

/**
 * @brief The number of bytes at which an adjustment in the number of bytes in
 *        a buffer is made.
 *
 * See tsb_i2s_unipro_cmd_handle_data() for the precise operation of this
 * constant.  It is set to be 25% above or below the hardware fifo level.   The
 * 256 is the TSB I2S hardware FIFO size in bytes.
 */
#define TSB_I2S_UNIPRO_TUNNEL_BUF_ADJUST_THRESHOLD ((TSB_I2S_UNIPRO_TUNNEL_BUF_SZ - 256) / 4)

/**
 * @brief Select the next buffer to be used.
 *
 * This marco is designed to execute quickly since it is called out of an
 * interrupt it relies on the fact that the number of buffers is a power of two.
 */
#define TSB_I2S_UNIPRO_TUNNEL_NEXT_BUF(buf) ((buf + 1) & (TSB_I2S_UNIPRO_TUNNEL_BUF_NUM - 1))

/** @brief Value used to show that the system has been initialized. */
#define TSB_I2S_UNIPRO_TUNNEL_INIT 0xa5

/**
 * @brief Define a type used to index the ping pong buffers.
 *
 * The number of entries in this enumeration must be a power of two.  Currently
 * this is not an issue since the buffers are in a ping pong configuration.
 * Should this change the macro TSB_I2S_UNIPRO_TUNNEL_NEXT_BUF will need to be
 * updated.
 */
typedef enum
{
    TSB_I2S_UNIPRO_TUNNEL_BUF_PING,
    TSB_I2S_UNIPRO_TUNNEL_BUF_PONG,
    TSB_I2S_UNIPRO_TUNNEL_BUF_NUM
} TSB_I2S_UNIPRO_TUNNEL_BUF_T;

/**
 * @brief Commands sent between the APBA and APBE via unipro.
 */
typedef enum
{
    /**
     * @brief Sent from the APBE to the APBA to measure the unipro transfer times.
     *
     * At init time the measure commands are sent back and forth to determine
     * the round trip time required for Uinpro messages.
     */
    TSB_I2S_UNIPRO_CMD_MEASURE_APBA,
    /**
     * @brief Sent from the APBA to the APBE in response to
     * TSB_I2S_UNIPRO_CMD_MEASURE_APBA.
     */
    TSB_I2S_UNIPRO_CMD_MEASURE_APBE,
    /** @brief Sent from the APBE to the APBA to let it know the roundtrip delay. */
    TSB_I2S_UNIPRO_CMD_MEASURE_APBA_ACK,
    /** @brief Message sent to the APBA to start the I2S data. */
    TSB_I2S_UNIPRO_CMD_I2S_START_APBA,
    /** @brief Message sent to the APBE to confim the start of the I2S bus. */
    TSB_I2S_UNIPRO_CMD_I2S_START_APBE,
    /** @brief Message sent from either the APBA or APBE containing raw I2S data. */
    TSB_I2S_UNIPRO_CMD_I2S_DATA,
    /**
     * @brief Not a message at all, just marking the last entry in the
     * enumeration.  This MUST remain the last item in the list.
     */
    TSB_I2S_UNIPRO_CMD_END
} TSB_I2S_UNIPRO_CMD_T;

/**
 * @brief Structure used to hold all messages sent between the APBA and APBE.
 *
 * The length is not tracked as part of the message, since Unipro handles the
 * length of the message.
 */
struct tsb_i2s_unipro_msg_s
{
    /**
     * @brief The actual command.
     *
     * This is of type TSB_I2S_UNIPRO_CMD, but set as uint32_t here to keep the
     * size fixed.
     */
    uint32_t cmd;
    union tsb_i2s_unipro_cmd_data_u
    {
        /** @brief TSB_I2S_UNIPRO_CMD_I2S_DATA Data buffer with sample data. */
        uint8_t buf[0];
        /** @brief TSB_I2S_UNIPRO_CMD_START_APBE/A true for start false for stop. */
        bool start;
        /** @brief TSB_I2S_UNIPRO_CMD_MEASURE_* time stamp. */
        uint32_t time;
    } data;
};

/**
 * @brief The data needed on a per RX/TX buffer basis.
 */
struct tsb_i2s_unipro_tunnel_buf_s
{
    /** @brief The length of the transfer. */
    size_t len;
    /**
     * @brief true if the buffer was allocated from unipro memory.
     *
     * This is used since Unipro memory buffers need to be freed and those
     * which use malloc do not.
     */
    bool unipro_allocated;
    /** @brief The dma ops needed for the transfer.
     *
     * These are only setup once to save runtime in the interrupt handler.
     */
    struct device_dma_op *dma_op;
    /**
     * @brief Pointer to the message to be sent or which has been received.
     */
    struct tsb_i2s_unipro_msg_s *msg;
};

/**
 * @brief Global storage for the I2S tunneled over Unipro driver.
 */
struct tsb_i2s_unipro_tunnel_s
{
    /** @brief Set when the clocks are enabled. */
    unsigned int enabled:1;
    /**
     * @brief Set when running and cleared when the system should stop.   It
     *        will stop after the next TX DMA interrupt and the TX FIFO is empty.
     */
    unsigned int running:1;
    /**
     * @brief Set when the clocks are initialized, but the I2S hardware has yet to
     *        be enabled.
     *
     * This state is necessary to allow start to enable the I2S hardware as quickly
     * as possible.
     */
    unsigned int armed:1;
    /**
      * @brief Will be set while data reception is in process.
      *
      * It will be cleared on the next RX DMA interrupt after running is cleared.
      * This is necessary to make sure all DMA has stopped before shutting down.
      */
    unsigned int receiving:1;
    /**
      * @brief Will be set while data transmission is in process.
      *
      * It will be cleared when on the next TX DMA interrupt after running is
      * cleared.  This is necessary to make sure all DMA has stopped before
      * shutting down.
      */
    unsigned int transmitting:1;
    /** @brief Set when Unipro is registered. */
    unsigned int unipro_registered:1;
    /** @brief Set when configured as a master device, clear when a slave. */
    unsigned int is_master:1;
    /** @brief Set to TSB_I2S_UNIPRO_TUNNEL_INIT once the system has been initialized. */
    uint8_t initialized;
    /** @brief The number of bytes per sample.
     *
     * For mono data this is 1 for 8 bit data, 2 for 16 bit data, 4 for 24 or 32
     * bit data.  For stereo these numbers are doubled to cover the extra
     * channel.  This value if valid after the configure function is called.
     */
    uint8_t bytes_per_sample;
    /** @brief ID used for reserving and freeing the free running timer used for
     *         measurements.
     *
     * This timer is allocated when the driver is enabled with
     * i2s_unipro_tunnel_enable(true) and freed when i2s_unipro_tunnel_enable(false)
     * is called.  This is done to save power by allowing the timer to be
     * disabled.
     */
    int fr_tmr_id;
    /**
     * @brief This is the current bit clock rate.  It is valid only after the
     *        system has been configured via i2s_unipro_tunnel_i2s_config().
     */
    unsigned int bclk_rate;
    /**
     * @brief The audio clock pll device.  Valid after i2s_unipro_tunnel_enable(true)
     *        is called.
     */
    struct device *pll_dev;
    /**
     * @brief The dma device for both the RX and TX channels.  Valid after
     * i2s_unipro_tunnel_init() is called.
     */
    struct device *dma_dev;
    /** @brief The RX DMA channel.  Valid after i2s_unipro_tunnel_init() is called. */
    void *i2s_rx_dma_channel;
    /** @brief The TX DMA channel.  Valid after i2s_unipro_tunnel_init() is called. */
    void *i2s_tx_dma_channel;
    /**
     * @brief The I2S RX buffers.
     *
     * These buffers are allocated in i2s_unipro_tunnel_init() and are allocated
     * for the rest of the run.  They are not freed, since this would slow down
     * execution.
     */
    struct tsb_i2s_unipro_tunnel_buf_s i2s_rx_unipro_tx_buf[TSB_I2S_UNIPRO_TUNNEL_BUF_NUM];
    /**
     * @brief The I2S TX buffers.
     *
     * These buffers are allocated in i2s_unipro_tunnel_arm(true) since they
     * are in Unipro memory.  This is done to avoid copying the memory to a new
     * DMA buffer before sending it to the I2S FIFO.  Once all data is in the
     * FIFO the buffer will be freed, since it must be returned to the Unipro
     * cport buffer pool.
     */
    struct tsb_i2s_unipro_tunnel_buf_s i2s_tx_unipro_rx_buf[TSB_I2S_UNIPRO_TUNNEL_BUF_NUM];
    /** @brief Index of the next RX buffer to be sent to the APBA/APBE. */
    TSB_I2S_UNIPRO_TUNNEL_BUF_T i2s_tx_buf;
    /**
     * @brief The number of APBE 48MHz timer ticks it takes to send a unipro
     *        message to the APBA and back.
     */
    uint32_t roundtrip_timer_ticks;
    /** @brief The number of timer ticks the local timer is from the remote timer. */
    int32_t timer_offset;
    /**
     * @brief The number of packets transfered over Unipro.  This is used to
     *        calculate clock drift based on the samples retransmitted or dropped.
     */
    unsigned int i2s_unipro_rx_packets;
    /**
     * @brief The number of samples which are retransmitted due to a lack of
     *        data from the other side.
     */
    unsigned int i2s_tx_samples_retransmitted;
    /**
     * @brief The number of samples which are dropped due to receiving samples
     *        over Unipro faster than they can be sent out.
     */
    unsigned int i2s_tx_samples_dropped;
    /**
     * @brief The number of buffers which had to be dropped when received over
     *        Unipro.  A non zero value indicates a serious problem with the
     *        driver operation.
     */
    unsigned int i2s_tx_buffers_dropped;
} g_i2s_unipro_tunnel;

/*
 * Clear all pending interrupts on an I2S block.
 *
 * base  The base address of the block to act upon (see SB_I2S_REG_S*_BASE).
 */
static void tsb_i2s_unipro_clr_irq(uint32_t base)
{
    putreg32(TSB_I2S_REG_INT_ERROR_MASK | TSB_I2S_REG_INT_INT, base + TSB_I2S_REG_INTCLR);
}

/*
 * Wait for a mask to become clear in a register.
 *
 * reg  The address of the register to read.
 * mask The mask of bits to be clear before returning.
 */
static int tsb_i2s_unipro_i2s_wait_clear(uint32_t reg, uint32_t mask)
{
    unsigned int failsafe;
    bool in_irq;

    in_irq = up_interrupt_context();
    failsafe = TSB_I2S_POLL_LIMIT;
    if (in_irq)
    {
        failsafe *= 10;
    }
    while ((getreg32(reg) & mask) && (--failsafe))
    {
        if (!in_irq)
        {
            usleep(100);
        }
    }
    return (failsafe == 0);
}

/*
 * Wait for a mask to become set in a register.
 *
 * reg  The address of the register to read.
 * mask The mask of bits to be set before returning.
*/
static int tsb_i2s_unipro_i2s_wait_set(uint32_t reg, uint32_t mask)
{
    unsigned int failsafe;
    bool in_irq;

    in_irq = up_interrupt_context();
    failsafe = TSB_I2S_POLL_LIMIT;
    if (in_irq)
    {
        failsafe *= 10;
    }
    while (((getreg32(reg) & mask) != mask) && (--failsafe))
    {
        if (!in_irq)
        {
            usleep(100);
        }
    }
    return (failsafe == 0);
}

/*
 * Restart the I2S data after an error.  This is risky as it could restart out
 * of sync with the other side.
 *
 * base  The base address of the block to act upon (see SB_I2S_REG_S*_BASE).
 */
static void tsb_i2s_unipro_i2s_restart(uint32_t i2s_base)
{
    /* Wait for the current transaction to complete. */
    tsb_i2s_unipro_i2s_wait_clear(i2s_base + TSB_I2S_REG_BUSY,
                                  TSB_I2S_REG_BUSY_SPK_MIC_BUSY);
    putreg32(TSB_I2S_REG_START_SPK_MIC_START|TSB_I2S_REG_START_START,
             i2s_base + TSB_I2S_REG_START);
    putreg32(TSB_I2S_REG_MUTE_MUTEN, i2s_base + TSB_I2S_REG_MUTE);
}

/*
 * Start the I2S TX DMA channel on the PING buffer.
 *
 * tx_prime_bytes The number of bytes to copy into the tx buffer before
 *                DMA is started.
 */
static int tsb_i2s_unipro_start_i2s_tx_dma(size_t tx_prime_bytes)
{
    int error;
    struct tsb_i2s_unipro_tunnel_buf_s *buf_p;
    uint32_t *sample_p;

    /*
     * Prime the TX buffer.  It is assumed the value passed in is not greater
     * than the FIFO size.  The size is rounded down to the nearest multiple of
     * 4 as FIFO memory access must be done in multiples of 4.
     */
    tx_prime_bytes /= 4;
    buf_p = &g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[TSB_I2S_UNIPRO_TUNNEL_BUF_PING];
    buf_p->len = TSB_I2S_UNIPRO_TUNNEL_BUF_SZ - (tx_prime_bytes * 4);
    sample_p = (uint32_t *)buf_p->msg->data.buf;
    while (tx_prime_bytes--)
    {
        putreg32(*sample_p++, TSB_I2S_REG_SO_BASE + TSB_I2S_REG_LMEM00);
    }

    buf_p->dma_op->sg[0].len = buf_p->len;
    buf_p->dma_op->sg[0].src_addr = (off_t)sample_p;
    buf_p->dma_op->callback_arg = (void *)TSB_I2S_UNIPRO_TUNNEL_BUF_PING;
    g_i2s_unipro_tunnel.transmitting = true;
    error = device_dma_enqueue(g_i2s_unipro_tunnel.dma_dev,
                               g_i2s_unipro_tunnel.i2s_tx_dma_channel,
                               buf_p->dma_op);
    if (error)
    {
        lldbg("I2S Unipro Tunnel: Unable to enqueue dma request.\n");
    }
    return error;
}

/*
 * Start the I2S RX DMA channel on the PING buffer.
 */
static int tsb_i2s_unipro_start_i2s_rx_dma(void)
{
    int error;
    struct tsb_i2s_unipro_tunnel_buf_s *buf_p;

    g_i2s_unipro_tunnel.i2s_tx_buf = TSB_I2S_UNIPRO_TUNNEL_BUF_PING;

    buf_p = &g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[TSB_I2S_UNIPRO_TUNNEL_BUF_PING];
    buf_p->len = TSB_I2S_UNIPRO_TUNNEL_BUF_SZ;
    buf_p->dma_op->sg[0].len = buf_p->len;
    buf_p->dma_op->sg[0].dst_addr = (off_t)buf_p->msg->data.buf;
    buf_p->dma_op->callback_arg = (void *)TSB_I2S_UNIPRO_TUNNEL_BUF_PING;
    buf_p->msg->cmd = TSB_I2S_UNIPRO_CMD_I2S_DATA;
    g_i2s_unipro_tunnel.receiving = true;
    error = device_dma_enqueue(g_i2s_unipro_tunnel.dma_dev,
                               g_i2s_unipro_tunnel.i2s_rx_dma_channel,
                               buf_p->dma_op);

    return error;
}
/*
 * Free the unipro allocated I2S DMA buffers.
 */
static void tsb_i2s_unipro_tunnel_free_dma_bufs(void)
{
    struct tsb_i2s_unipro_tunnel_buf_s *curr_buf_p;
    unsigned int i;
    int error;

    /* Free any allocated Unipro buffers. */
    for (i = 0; i < TSB_I2S_UNIPRO_TUNNEL_BUF_NUM; i++)
    {
        curr_buf_p = &g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i];
        error = 0;
        if (!device_dma_op_is_complete(g_i2s_unipro_tunnel.dma_dev,
                                       g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].dma_op))
        {
            error |= device_dma_dequeue(g_i2s_unipro_tunnel.dma_dev,
                                        g_i2s_unipro_tunnel.i2s_rx_dma_channel,
                                        g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].dma_op);
        }
        if (!device_dma_op_is_complete(g_i2s_unipro_tunnel.dma_dev,
                                       g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].dma_op))
        {
            error |= device_dma_dequeue(g_i2s_unipro_tunnel.dma_dev,
                                        g_i2s_unipro_tunnel.i2s_tx_dma_channel,
                                        g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].dma_op);
        }
        if (curr_buf_p->unipro_allocated)
        {
            unipro_rxbuf_free(TSB_I2S_UNIPRO_TUNNEL_CPORTID, curr_buf_p->msg);
            curr_buf_p->msg = NULL;
            curr_buf_p->len = 0;
        }
        if (error)
        {
            lldbg("Error: %d when dequeuing DMA ops for buffer %u.\n", error, i);
        }
    }
}

/*
 * Handle shutting down I2S and DMA.
 */
static void tsb_i2s_unipro_tunnel_stop_i2s_dma(void)
{
    size_t failsafe;
    int error;

    /* Disable the error interrupts. */
    putreg32(TSB_I2S_REG_INT_ERROR_MASK | TSB_I2S_REG_INT_INT,
             TSB_I2S_REG_SO_BASE + TSB_I2S_REG_INTMASK);
    putreg32(TSB_I2S_REG_INT_ERROR_MASK | TSB_I2S_REG_INT_INT,
             TSB_I2S_REG_SI_BASE + TSB_I2S_REG_INTMASK);

    /* Stop all DMA activity.  This will call the TX and RX DMA interrupts. */
    gdmac_abort_chan(g_i2s_unipro_tunnel.dma_dev, g_i2s_unipro_tunnel.i2s_tx_dma_channel);

    /* Set mute. */
    tsb_i2s_unipro_i2s_wait_set(TSB_I2S_REG_SO_BASE + TSB_I2S_REG_BUSY,
                                TSB_I2S_REG_BUSY_SPK_MIC_BUSY);
    putreg32(0, TSB_I2S_REG_SO_BASE + TSB_I2S_REG_MUTE);

    tsb_i2s_unipro_i2s_wait_set(TSB_I2S_REG_SI_BASE + TSB_I2S_REG_BUSY,
                                TSB_I2S_REG_BUSY_SPK_MIC_BUSY);
    putreg32(0, TSB_I2S_REG_SI_BASE + TSB_I2S_REG_MUTE);

    /*
     * Wait for transmitting to stop and then stop receiving, to keep the RX
     * fifo empty.
     */
    failsafe = 100;  /* Wait up to 1ms. */
    while (g_i2s_unipro_tunnel.transmitting &&
           (--failsafe > 0))
    {
        usleep(10);
    }

    /* Stop RX if it has not already stopped. */
    gdmac_abort_chan(g_i2s_unipro_tunnel.dma_dev, g_i2s_unipro_tunnel.i2s_rx_dma_channel);

    /* Wait for receive DMA to stop. */
    failsafe = 100;  /* Wait up to 1ms. */
    while (g_i2s_unipro_tunnel.receiving &&
           (--failsafe > 0))
    {
        usleep(10);
    }

    /* Disable the input and output channels. */
    putreg32(TSB_I2S_REG_STOP_I2S_STOP, TSB_I2S_REG_SO_BASE + TSB_I2S_REG_STOP);

    putreg32(TSB_I2S_REG_STOP_I2S_STOP, TSB_I2S_REG_SI_BASE + TSB_I2S_REG_STOP);

    /* Disable the I2S controller. */
    putreg32(TSB_I2S_REG_STOP_I2S_STOP, TSB_I2S_REG_SC_BASE + TSB_I2S_REG_STOP);

    /* Wait for the last of the data to be output. */
    tsb_i2s_unipro_i2s_wait_clear(TSB_I2S_REG_SC_BASE + TSB_I2S_REG_BUSY,
                                  TSB_I2S_REG_BUSY_BUSY);

    /* Clean up DMA. */
    tsb_i2s_unipro_tunnel_free_dma_bufs();

    /* Disable all interrupts and DMA requests. */
    putreg32(TSB_I2S_REG_INT_ERROR_MASK | TSB_I2S_REG_INT_INT | TSB_I2S_REG_INT_DMACMSK,
             TSB_I2S_REG_SO_BASE + TSB_I2S_REG_INTMASK);
    putreg32(TSB_I2S_REG_INT_ERROR_MASK | TSB_I2S_REG_INT_INT | TSB_I2S_REG_INT_DMACMSK,
             TSB_I2S_REG_SI_BASE + TSB_I2S_REG_INTMASK);

    /* Disable the clock . */
    error = device_pll_stop(g_i2s_unipro_tunnel.pll_dev);
    if (error != OK)
    {
        lldbg("Error: Unable to stop audio PLL: %d\n", error);
    }
    tsb_clk_disable(TSB_CLK_I2SBIT);
    g_i2s_unipro_tunnel.running = false;
}

/*
 * Start the TSB I2S hardware.
 */
void tsb_i2s_unipro_tunnel_start_i2s(void)
{
    reglog_log(0xb8, (uint32_t)g_i2s_unipro_tunnel.running);
    if (!g_i2s_unipro_tunnel.running)
    {
        /*
         * Enable the RX DMA request and error interrupts and enable the RX
         * interrupt if in slave mode.
         */
        putreg32(g_i2s_unipro_tunnel.is_master ? TSB_I2S_REG_INT_INT : 0,
                 TSB_I2S_REG_SI_BASE+TSB_I2S_REG_INTMASK);

        putreg32(TSB_I2S_REG_START_START, TSB_I2S_REG_SC_BASE+TSB_I2S_REG_START);

        putreg32(TSB_I2S_REG_START_START, TSB_I2S_REG_SI_BASE+TSB_I2S_REG_START);
        putreg32(TSB_I2S_REG_MUTE_MUTEN, TSB_I2S_REG_SI_BASE+TSB_I2S_REG_MUTE);
        putreg32(TSB_I2S_REG_START_SPK_MIC_START, TSB_I2S_REG_SI_BASE+TSB_I2S_REG_START);

        putreg32(TSB_I2S_REG_START_START, TSB_I2S_REG_SO_BASE+TSB_I2S_REG_START);
        putreg32(TSB_I2S_REG_MUTE_MUTEN, TSB_I2S_REG_SO_BASE+TSB_I2S_REG_MUTE);
        putreg32(TSB_I2S_REG_START_SPK_MIC_START, TSB_I2S_REG_SO_BASE+TSB_I2S_REG_START);
        g_i2s_unipro_tunnel.running = true;
    }
}

/*
 * Send the command to start the APBE.  This version can be called from an
 * interrupt context.
 */
static int tsb_i2s_unipro_send_start_cmd(TSB_I2S_UNIPRO_CMD_T cmd, bool enable)
{
    struct tsb_i2s_unipro_msg_s msg;

    msg.cmd = cmd;
    msg.data.start = enable;
    return unipro_send(TSB_I2S_UNIPRO_TUNNEL_CPORTID, &msg, sizeof(msg));
}

/*
 * Handle any errors on the I2S output channel.
 *
 * irq     The interrupt number.
 * context The standard nuttx interrupt context (register store).
 */
static int tsb_i2s_unipro_i2s_out_error_handler(int irq, void *context)
{
    uint32_t reg_intstat;

    (void)irq;
    (void)context;

    reg_intstat = getreg32(TSB_I2S_REG_SO_BASE + TSB_I2S_REG_INTSTAT);
    reglog_log(0x02, reg_intstat);
    if (reg_intstat & TSB_I2S_REG_INT_LRCK)
    {
        /* Do nothing, this should only happen in slave mode. */
        tsb_i2s_unipro_i2s_restart(TSB_I2S_REG_SO_BASE);
    }
    if (reg_intstat & TSB_I2S_REG_INT_UR)
    {
        /*
         * TODO: Handle the output under run error by inserting a sample in the
         * output FIFO, unmutting and restarting the channel.
         */
        tsb_i2s_unipro_i2s_restart(TSB_I2S_REG_SO_BASE);
    }
    if (reg_intstat & TSB_I2S_REG_INT_OR)
    {
        /* TODO: Handle an overrun by simply restarting and unmutting. */
        tsb_i2s_unipro_i2s_restart(TSB_I2S_REG_SO_BASE);
    }
    putreg32(TSB_I2S_REG_INT_ERROR_MASK, TSB_I2S_REG_SO_BASE + TSB_I2S_REG_INTCLR);
    reglog_log(0x2, 0xff);
    return OK;
}

/*
 * Handle any errors on the i2s input channel.
 *
 * irq     The interrupt number.
 * context The standard nuttx interrupt context (register store).
 */
static int tsb_i2s_unipro_i2s_in_error_handler(int irq, void *context)
{
    uint32_t reg_intstat;

    (void)irq;
    (void)context;

    reg_intstat = getreg32(TSB_I2S_REG_SI_BASE + TSB_I2S_REG_INTSTAT);
    reglog_log(0x01, reg_intstat);
    if (reg_intstat & TSB_I2S_REG_INT_LRCK)
    {
        /* TODO: Do nothing, no idea under which cases this would happen. */
        tsb_i2s_unipro_i2s_restart(TSB_I2S_REG_SI_BASE);
    }
    if (reg_intstat & TSB_I2S_REG_INT_UR)
    {
        /* Handle the input under run error by restarting and waiting for more data. */
        tsb_i2s_unipro_i2s_restart(TSB_I2S_REG_SI_BASE);
    }
    if (reg_intstat & TSB_I2S_REG_INT_OR)
    {
        /* TODO: Handle an overrun by simply restarting and unmutting. */
        tsb_i2s_unipro_i2s_restart(TSB_I2S_REG_SI_BASE);
    }
    putreg32(TSB_I2S_REG_INT_ERROR_MASK, TSB_I2S_REG_SI_BASE + TSB_I2S_REG_INTCLR);
    reglog_log(0x01, 0xFF);
   return OK;
}

/*
 * Trigger the master portion of the tunnel.  This is done on the first character
 * received to minimize the delay until valid input data is flowing.
 *
 * irq     The interrupt number.
 * context The standard nuttx interrupt context (register store).
 */
static int tsb_i2s_unipro_i2s_in_data(int irq, void *context)
{
    (void)irq;
    (void)context;

    reglog_log(0x03, 0);
    /* Disable the interrupt. */
    putreg32(TSB_I2S_REG_INT_INT, TSB_I2S_REG_SI_BASE+TSB_I2S_REG_INTMASK);
    /* Trigger the other size. */
#if defined(CONFIG_UNIPRO_P2P_APBA)
    tsb_i2s_unipro_send_start_cmd(TSB_I2S_UNIPRO_CMD_I2S_START_APBE, true);
#else
    tsb_i2s_unipro_send_start_cmd(TSB_I2S_UNIPRO_CMD_I2S_START_APBA, true);
#endif
    return OK;
}

/*
 * Called when a Unipro packet has been received.  The function then handles
 * checking to see if the packet can be transmitted over I2S based on the
 * current status of the previous buffers DMA.  Adjustments are made to the
 * number of characters in this buffer based on the status of the previous
 * buffer in order to try to stay ahead of any potential data shortages or
 * surpluses.
 *
 * The buffer passed in must be freed with a call to unipro_rxbuf_free().  This
 * is done in the DMA complete callback.
 *
 * cportid  The cport the data was received on.
 * msg      Pointer to the message received from the other side.
 * len      The length of the message received.
 * rsp      A pointer to the response from this function.  See
 *          tsb_i2s_unipro_tunnel_unipro_rx() for more details.
 */
#if !defined(CONFIG_I2S_TUNNEL_LOCAL_LOOPBACK)
static int tsb_i2s_unipro_cmd_handle_data(
    unsigned int cportid,
    struct tsb_i2s_unipro_msg_s *msg,
    size_t len,
    struct tsb_i2s_unipro_msg_s *rsp)
{
    void *dma_dst;
    void *dma_src;
    size_t prev_bytes_sent;
    struct tsb_i2s_unipro_tunnel_buf_s *i2s_tx_buf_p =
        &g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[g_i2s_unipro_tunnel.i2s_tx_buf];
    struct tsb_i2s_unipro_tunnel_buf_s *unipro_rx_buf_p =
        &g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[TSB_I2S_UNIPRO_TUNNEL_NEXT_BUF(g_i2s_unipro_tunnel.i2s_tx_buf)];

    reglog_log(0x30, len);
    /* Do not send a response to the data command. */
    rsp->cmd = TSB_I2S_UNIPRO_CMD_END;
    len -= offsetof(struct tsb_i2s_unipro_msg_s, data.buf);

    g_i2s_unipro_tunnel.i2s_unipro_rx_packets++;
    /*
     * TODO: In the long run it might be better to simply deal with an incorrect
     * sized packet.  However, this should not happen under normal operation.
     */
    if (len == TSB_I2S_UNIPRO_TUNNEL_BUF_SZ)
    {
        if (unipro_rx_buf_p->msg != NULL)
        {
            /*
             * All buffers are are full, so all that can be done is to take note
             * of this and overwrite the last buffer.
             *
             * TODO: Wait a little while before tossing the packet.  Tossing a
             * packet unless well on the way to the next one will stop
             * everything.  This is an interrupt context, so do not wait to long.
             * A better design might be to have a stack of incoming packets and
             * toss one when 3 are on the stack.  However, this should be a
             * rare event if the thresholds below are good.
             */
            reglog_log(0x3e, g_i2s_unipro_tunnel.i2s_tx_buf);
            unipro_rxbuf_free(TSB_I2S_UNIPRO_TUNNEL_CPORTID, msg);
            g_i2s_unipro_tunnel.i2s_tx_buffers_dropped++;
            return 0;
        }
        unipro_rx_buf_p->msg = msg;
        unipro_rx_buf_p->len = TSB_I2S_UNIPRO_TUNNEL_BUF_SZ;
        unipro_rx_buf_p->unipro_allocated = true;
        prev_bytes_sent = TSB_I2S_UNIPRO_TUNNEL_BUF_SZ;
        /*
         * If still transmitting, then figure out how many characters have gone out.
         */
        if (i2s_tx_buf_p->msg != NULL)
        {
            tsb_dma_get_chan_addr(g_i2s_unipro_tunnel.i2s_tx_dma_channel, &dma_src, &dma_dst);
            prev_bytes_sent = (size_t)((uint8_t *)dma_src - i2s_tx_buf_p->msg->data.buf);
            /* Adjust for the case when the length has been adjusted for repeated samples. */
            prev_bytes_sent += TSB_I2S_UNIPRO_TUNNEL_BUF_SZ - i2s_tx_buf_p->len;
            reglog_log(0x31, (uint32_t)prev_bytes_sent);
        }
        /*
         * If the number of samples sent so far is greater than the threshold,
         * insert a sample from the data which will be sent out.
         */
        if (prev_bytes_sent > TSB_I2S_UNIPRO_TUNNEL_BUF_SZ - TSB_I2S_UNIPRO_TUNNEL_BUF_ADJUST_THRESHOLD)
        {
            memcpy(&unipro_rx_buf_p->msg->data.buf[unipro_rx_buf_p->len],
                   &unipro_rx_buf_p->msg->data.buf[unipro_rx_buf_p->len-g_i2s_unipro_tunnel.bytes_per_sample],
                   g_i2s_unipro_tunnel.bytes_per_sample);
            unipro_rx_buf_p->len = unipro_rx_buf_p->len + g_i2s_unipro_tunnel.bytes_per_sample;
            g_i2s_unipro_tunnel.i2s_tx_samples_retransmitted++;
            reglog_log(0x32, unipro_rx_buf_p->len);
        }
        /*
         * If the number of samples left in the previous buffer is not within the
         * threshold or fully copied into the FIFO by now, then a backlog of
         * samples is starting.  Leave one sample off of the packet to allow the
         * backlog to dissipate.
         */
        else if (prev_bytes_sent < TSB_I2S_UNIPRO_TUNNEL_BUF_ADJUST_THRESHOLD)
        {
            unipro_rx_buf_p->len -= g_i2s_unipro_tunnel.bytes_per_sample;
            g_i2s_unipro_tunnel.i2s_tx_samples_dropped++;
            reglog_log(0x33, unipro_rx_buf_p->len);
        }
    }
    else
    {
        lldbg("Error: Incorrect size (%d) received from Unipro.\n", len);
    }
    reglog_log(0x3F, g_i2s_unipro_tunnel.i2s_tx_buf);
    return len;
}
#endif

/*
 * I2S DMA RX callback.  This function is called when the DMA RX buffer is full.
 * As a result the following needs to be done:
 *
 *   1. Switch the DMA buffer to the PING or PONG buffer.
 *   2. Re-enable DMA for the data coming in.
 *   3. Send the data received over Unipro.
 *
 *  dev   The DMA device structure pointer.
 *  chan  The channel which cause the event causing this to be called.
 *  op    A pointer to the DMA ops used for the completed operation.
 *  event The DMA event which caused this function to be called (not used).
 *  arg   The buffer index which is being operated on (not a pointer to it).
 *
 * Note: Interrupts are not disabled when this function is called since it runs
 *       in a non interrupt context.
 */
static int tsb_i2s_unipro_i2s_rx_dma_callback(
    struct device *dev,
    void *chan,
    struct device_dma_op *op,
    unsigned int event,
    void *arg)
{
    irqstate_t flags;
    TSB_I2S_UNIPRO_TUNNEL_BUF_T curr_buf = (TSB_I2S_UNIPRO_TUNNEL_BUF_T)arg;
    TSB_I2S_UNIPRO_TUNNEL_BUF_T next_buf = TSB_I2S_UNIPRO_TUNNEL_NEXT_BUF(curr_buf);
    struct tsb_i2s_unipro_tunnel_buf_s *curr_buf_p =
        &g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[curr_buf];
    struct tsb_i2s_unipro_tunnel_buf_s *next_buf_p =
        &g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[next_buf];
    int error = 0;

    (void)dev;
    (void)chan;
    (void)op;
    (void)event;

    reglog_log(0x10, curr_buf);
    if (g_i2s_unipro_tunnel.armed)
    {
        /* Restart DMA. */
        flags = irqsave();
        next_buf_p->dma_op->sg[0].dst_addr = (off_t)next_buf_p->msg->data.buf;
        next_buf_p->dma_op->sg[0].len = next_buf_p->len;
        next_buf_p->dma_op->callback_arg = (void *)next_buf;
        error = device_dma_enqueue(g_i2s_unipro_tunnel.dma_dev,
                                   g_i2s_unipro_tunnel.i2s_rx_dma_channel,
                                   next_buf_p->dma_op);
        irqrestore(flags);
        reglog_log(0x11, error);
#if defined(CONFIG_I2S_TUNNEL_LOCAL_LOOPBACK)
        /*
         * In local I2S loopback mode simply copy the received data to the next
         * transmit buffer.
         */
        memcpy(g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[next_buf].msg->data.buf,
               curr_buf_p->msg->data.buf,
               curr_buf_p->len);
        g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[next_buf].len = curr_buf_p->len;
#else
    curr_buf_p->msg->cmd = TSB_I2S_UNIPRO_CMD_I2S_DATA;
# if defined(TSB_I2S_UNIPRO_TUNNEL_QUEUE_UNIPRO_SEND)
        error = unipro_send_async(TSB_I2S_UNIPRO_TUNNEL_CPORTID,
                                  curr_buf_p->msg,
                                  curr_buf_p->len+offsetof(struct tsb_i2s_unipro_msg_s, data.buf),
                                  NULL,
                                  NULL);
# else
        error = unipro_send(TSB_I2S_UNIPRO_TUNNEL_CPORTID,
                            curr_buf_p->msg,
                            curr_buf_p->len+offsetof(struct tsb_i2s_unipro_msg_s, data.buf));
# endif
        reglog_log(0x12, error);
#endif
    }
    else
    {
        g_i2s_unipro_tunnel.receiving = false;
    }
    if (error)
    {
        /* All may well now stop, this should not happen. */
        lldbg("Error: %d received in RX DMA interrupt.\n", error);
    }
    reglog_log(0x1F, curr_buf);
    return OK;
}

/*
 * I2S DMA TX callback.  This function is called when the DMA TX buffer is full.
 * As a result the following needs to be done:
 *
 *   1. Check to see if the next buffer has been received from Unipro.
 *   1.1. If not or if too few samples are still in the previous buffer then
 *        repeat last sample in the previous buffer.
 *   1.2. If it has:
 *   1.2.1. Switch the DMA buffer to the PING or PONG buffer.
 *   1.2.2. Check to see if too few samples from the previous buffer were sent.
 *          If this is the case, remove a sample from the buffer.
 *   2. In either case re-enable DMA.
 *
 *  dev   The DMA device structure pointer.
 *  chan  The channel which cause the event causing this to be called.
 *  op    A pointer to the DMA ops used for the completed operation.
 *  event The DMA event which caused this function to be called (not used).
 *  arg   The buffer index which is being operated on (not a pointer to it).
 *
 * Note: Interrupts are not disabled when this function is called since it runs
 *       in a non interrupt context.
 */
static int tsb_i2s_unipro_i2s_tx_dma_callback(
    struct device *dev,
    void *chan,
    struct device_dma_op *op,
    unsigned int event,
    void *arg)
{
    irqstate_t flags;
    TSB_I2S_UNIPRO_TUNNEL_BUF_T curr_buf = (TSB_I2S_UNIPRO_TUNNEL_BUF_T)arg;
    TSB_I2S_UNIPRO_TUNNEL_BUF_T next_buf = TSB_I2S_UNIPRO_TUNNEL_NEXT_BUF(curr_buf);
    struct tsb_i2s_unipro_tunnel_buf_s *curr_buf_p =
        &g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[curr_buf];
    struct tsb_i2s_unipro_tunnel_buf_s *next_buf_p =
        &g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[next_buf];
    int error = 0;
    bool free_buf;

    (void)dev;
    (void)chan;
    (void)op;
    (void)event;

    /*
     * If the next buffer has not yet been received, resend the last character
     * which was transmitted.  It if has been received send the next buffer.
     */
    reglog_log(0x20, curr_buf);
    free_buf = curr_buf_p->unipro_allocated;
    if (g_i2s_unipro_tunnel.armed)
    {
        flags = irqsave();
        if (next_buf_p->len != 0)
        {
            reglog_log(0x21, (uint32_t)next_buf_p->len);
            next_buf_p->dma_op->sg[0].src_addr = (off_t)next_buf_p->msg->data.buf;
            next_buf_p->dma_op->callback_arg = (void *)next_buf;
            next_buf_p->dma_op->sg[0].len = next_buf_p->len;
            g_i2s_unipro_tunnel.i2s_tx_buf = next_buf;
            error = device_dma_enqueue(g_i2s_unipro_tunnel.dma_dev,
                                       g_i2s_unipro_tunnel.i2s_tx_dma_channel,
                                       next_buf_p->dma_op);
        }
        else
        {
            /*
             * Resend the last sample again.  Cannot simply re-queue with the
             * end of the buffer, since the next handler has to free the buffer
             * and thus needs a pointer to the start of the buffer.  To leave
             * the pointer intact simply copy the last sample to the start of
             * the buffer.
             *
             * This commonly happens on startup if the remote or local side is
             * delayed for some reason.  If this happens more than a few times
             * in a row the TX interrupt may not be able to keep up with the
             * consumption of the samples in the FIFO and the entire system may
             * halt.
             */
            /* TODO: Update to repeat the last n samples based on the time it takes
             * to send a sample. */
            memcpy(curr_buf_p->msg->data.buf,
                   &curr_buf_p->msg->data.buf[curr_buf_p->len-g_i2s_unipro_tunnel.bytes_per_sample],
                   g_i2s_unipro_tunnel.bytes_per_sample*10);
            /*
             * The src_addr and callback_arg should already be set, so they are
             * not reset here to save execution time.
             */
            curr_buf_p->dma_op->sg[0].len = g_i2s_unipro_tunnel.bytes_per_sample*10;
            error = device_dma_enqueue(g_i2s_unipro_tunnel.dma_dev,
                                       g_i2s_unipro_tunnel.i2s_tx_dma_channel,
                                       curr_buf_p->dma_op);
            free_buf = false;
            g_i2s_unipro_tunnel.i2s_tx_samples_retransmitted += 10;
            reglog_log(0x2e, (uint32_t)curr_buf_p->msg);
        }
        irqrestore(flags);
    }
    else
    {
        reglog_log(0x22, 0);
        g_i2s_unipro_tunnel.transmitting = false;
    }
    if (free_buf)
    {
        unipro_rxbuf_free(TSB_I2S_UNIPRO_TUNNEL_CPORTID, curr_buf_p->msg);
        curr_buf_p->msg = NULL;
        curr_buf_p->len = 0;
    }

    if (error)
    {
        /* All will now stop, this should not happen. */
        lldbg("Error: %d in tx DMA handler.\n", error);
    }
    reglog_log(0x2F, curr_buf);
    return OK;
}

#if !defined(CONFIG_I2S_TUNNEL_LOCAL_LOOPBACK)
# if defined(CONFIG_UNIPRO_P2P_APBE)

#  if defined(CONFIG_I2S_TUNNEL_ROUNDTRIP_LOOPBACK)
/*
 * Wait for the number of ticks to elapse before returning.
 *
 * ticks The number of ticks to wait.
 *
 * returns 0 upon success non zero if the timeout did not happen.
 */
static int tsb_i2s_unipro_wait_ticks(uint32_t ticks)
{
    unsigned int failsafe;
    uint32_t curr_tick;

    curr_tick= tsb_fr_tmr_get();
    failsafe = ticks;
    while (((tsb_fr_tmr_get() - curr_tick) < ticks) &&
           (failsafe-- > 0));
    return failsafe != 0;
}

/*
 * Send the command to start the APBA I2S.
 *
 * enable True when I2S should be started and false to stop.
 */
static int tsb_i2s_unipro_cmd_i2s_start_loopback(bool enable)
{
    tsb_i2s_unipro_send_start_cmd(TSB_I2S_UNIPRO_CMD_I2S_START_APBA, true);
    /* Wait half of the roundtrip time to allow the message to get to the APBA. */
    tsb_i2s_unipro_wait_ticks(g_i2s_unipro_tunnel.roundtrip_timer_ticks / 2);
    return OK;
}
#  else
#   define tsb_i2s_unipro_cmd_i2s_start_loopback(enable) (0)
#  endif

/*
 * Wait for the measurement process to complete before returning.
 *
 * returns 0 upon the measurement process completing and non zero upon timeout.
 */
static int tsb_i2s_unipro_wait_for_measruement(void)
{
    unsigned int failsafe;

    /* Wait for the measurement to be done, before starting. */
    failsafe = 1000;
    while (((volatile uint32_t)g_i2s_unipro_tunnel.roundtrip_timer_ticks == 0) &&
           (--failsafe > 0))
    {
        usleep(1000);
    }
    /* If a roundtrip time has been established retrun OK. */
    if (g_i2s_unipro_tunnel.roundtrip_timer_ticks > 0)
    {
        return OK;
    }
    return -EAGAIN;
}

/*
 * Send the start measure command from the APBE to the APBA.  This starts the
 * measurement sequence.
 */
static int tsb_i2s_unipro_cmd_measure_start(void)
{
    struct tsb_i2s_unipro_msg_s msg;

    msg.cmd = TSB_I2S_UNIPRO_CMD_MEASURE_APBA;
    msg.data.time = tsb_fr_tmr_get();
    g_i2s_unipro_tunnel.roundtrip_timer_ticks = msg.data.time;
    return unipro_send(TSB_I2S_UNIPRO_TUNNEL_CPORTID, &msg, sizeof(msg));
}
# else
#  define tsb_i2s_unipro_wait_for_measruement()         (0)
#  define tsb_i2s_unipro_cmd_measure_start()            (0)
# endif


/*
 * Handle the measure APBA command.
 *
 * cportid  The cport id for which the command was received on.
 * msg      The full message received.
 * len      The length of the message.
 * rsp      Pointer to the response which will be sent by
 *          tsb_i2s_unipro_tunnel_unipro_rx().
 */
static int tsb_i2s_unipro_cmd_handle_measure_apba(
    unsigned int cportid,
    struct tsb_i2s_unipro_msg_s *msg,
    size_t len,
    struct tsb_i2s_unipro_msg_s *rsp)
{
    uint32_t curr_time;

    (void)cportid;
    (void)len;

    curr_time = tsb_fr_tmr_get();
    g_i2s_unipro_tunnel.timer_offset = curr_time - msg->data.time;
    rsp->data.time = curr_time;
    rsp->cmd = TSB_I2S_UNIPRO_CMD_MEASURE_APBE;
    return OK;
}

/*
 * Handle the measure APBE command.
 *
 * cportid  The cport id for which the command was received on.
 * msg      The full message received.
 * len      The length of the message.
 * rsp      Pointer to the response which will be sent by
 *          tsb_i2s_unipro_tunnel_unipro_rx().
 */
static int tsb_i2s_unipro_cmd_handle_measure_apbe(
    unsigned int cportid,
    struct tsb_i2s_unipro_msg_s *msg,
    size_t len,
    struct tsb_i2s_unipro_msg_s *rsp)
{
    uint32_t curr_time;

    (void)cportid;
    (void)len;

    curr_time = tsb_fr_tmr_get();
    g_i2s_unipro_tunnel.roundtrip_timer_ticks = curr_time - g_i2s_unipro_tunnel.roundtrip_timer_ticks;
    g_i2s_unipro_tunnel.timer_offset = curr_time - msg->data.time -
                                       (g_i2s_unipro_tunnel.roundtrip_timer_ticks / 2);
    rsp->data.time = g_i2s_unipro_tunnel.roundtrip_timer_ticks;
    rsp->cmd = TSB_I2S_UNIPRO_CMD_MEASURE_APBA_ACK;
    return OK;
}

/*
 * Handle the measure APBA_ACK command.
 *
 * cportid  The cport id for which the command was received on.
 * msg      The full message received.
 * len      The length of the message.
 * rsp      Pointer to the response which will be sent by
 *          tsb_i2s_unipro_tunnel_unipro_rx().
 */
static int tsb_i2s_unipro_cmd_handle_measure_apba_ack(
    unsigned int cportid,
    struct tsb_i2s_unipro_msg_s *msg,
    size_t len,
    struct tsb_i2s_unipro_msg_s *rsp)
{
    (void)cportid;
    (void)len;

    g_i2s_unipro_tunnel.roundtrip_timer_ticks = msg->data.time;
    /*
     * Update the offset to account for the time it took to receive the first
     * measure message.
     */
    g_i2s_unipro_tunnel.timer_offset -= g_i2s_unipro_tunnel.roundtrip_timer_ticks / 2;
    /* No response, the end of the transfer chain. */
    rsp->cmd = TSB_I2S_UNIPRO_CMD_END;
    return OK;
}

/*
 * Handle the I2S start apba command.
 *
 * cportid  The cport id for which the command was received on.
 * msg      The full message received.
 * len      The length of the message.
 * rsp      Pointer to the response which will be sent by
 *          tsb_i2s_unipro_tunnel_unipro_rx().
 */
static int tsb_i2s_unipro_cmd_handle_i2s_start_apba(
    unsigned int cportid,
    struct tsb_i2s_unipro_msg_s *msg,
    size_t len,
    struct tsb_i2s_unipro_msg_s *rsp)
{
    tsb_i2s_unipro_tunnel_start_i2s();
    /* Respond with the start APBE if not in loopback mode. */
#if defined(CONFIG_I2S_TUNNEL_ROUNDTRIP_LOOPBACK)
    rsp->cmd = TSB_I2S_UNIPRO_CMD_I2S_END;
#else
    rsp->cmd = TSB_I2S_UNIPRO_CMD_I2S_START_APBE;
#endif
    return OK;
}

/*
 * Handle the I2S start APBE command.
 *
 * cportid  The cport id for which the command was received on.
 * msg      The full message received.
 * len      The length of the message.
 * rsp      Pointer to the response which will be sent by
 *          tsb_i2s_unipro_tunnel_unipro_rx().
 */
static int tsb_i2s_unipro_cmd_handle_i2s_start_apbe(
    unsigned int cportid,
    struct tsb_i2s_unipro_msg_s *msg,
    size_t len,
    struct tsb_i2s_unipro_msg_s *rsp)
{
    tsb_i2s_unipro_tunnel_start_i2s();
    /* No respone on the APBE since the APBA sent the command. */
    rsp->cmd = TSB_I2S_UNIPRO_CMD_END;
    return OK;
}
/*
 * RX handler for all Unipro received packets.
 *
 * cportid The cport the message was received on.
 * data    The message received via Unipro.
 * len     The length of the Unipro message received.
 */
static int tsb_i2s_unipro_tunnel_unipro_rx(
    unsigned int cportid,
    void *data,
    size_t len)
{
    static struct
    {
        int (* const fcn)(unsigned int cportid,
                          struct tsb_i2s_unipro_msg_s *msg,
                          size_t len,
                          struct tsb_i2s_unipro_msg_s *rsp);
        bool free_msg;
    } cmd_handler_tb[TSB_I2S_UNIPRO_CMD_END] =
    {
        { tsb_i2s_unipro_cmd_handle_measure_apba,     true  }, /* TSB_I2S_UNIPRO_CMD_MEASURE_APBA     */
        { tsb_i2s_unipro_cmd_handle_measure_apbe,     true  }, /* TSB_I2S_UNIPRO_CMD_MEASURE_APBE     */
        { tsb_i2s_unipro_cmd_handle_measure_apba_ack, true  }, /* TSB_I2S_UNIPRO_CMD_MEASURE_APBA_ACK */
        { tsb_i2s_unipro_cmd_handle_i2s_start_apba,   true  }, /* TSB_I2S_UNIPRO_CMD_I2S_START_APBA   */
        { tsb_i2s_unipro_cmd_handle_i2s_start_apbe,   true  }, /* TSB_I2S_UNIPRO_CMD_I2S_START_APBE   */
        { tsb_i2s_unipro_cmd_handle_data,             false }  /* TSB_I2S_UNIPRO_CMD_I2S_DATA         */
    };
    struct tsb_i2s_unipro_msg_s *msg;
    struct tsb_i2s_unipro_msg_s rsp;
    int ret;

    /* Assume command is not recognized. */
    msg = (struct tsb_i2s_unipro_msg_s *)data;
    reglog_log(0xa0, (uint32_t)msg->cmd);
    rsp.cmd = TSB_I2S_UNIPRO_CMD_END;
    ret = OK;
    if (len < sizeof(struct tsb_i2s_unipro_msg_s))
    {
        lldbg("Error: Unipro packet too small.\n");
    }
    else if (msg->cmd >= ARRAY_SIZE(cmd_handler_tb))
    {
        lldbg("Error: Invalid command %d received.\n", msg->cmd);
    }
    else
    {
        /* The command is valid, call the handler. */
        cmd_handler_tb[msg->cmd].fcn(cportid, msg, len, &rsp);
    }
    /* Send the response if required. */
    if (rsp.cmd < TSB_I2S_UNIPRO_CMD_END)
    {
        unipro_send(cportid, &rsp, sizeof(rsp));
    }
    /* Free the message if required. */
    if (cmd_handler_tb[msg->cmd].free_msg)
    {
        unipro_rxbuf_free(TSB_I2S_UNIPRO_TUNNEL_CPORTID, msg);
    }
    reglog_log(0xaf, (uint32_t)rsp.cmd);
    return ret;
}
#endif

/*
 * Compare function for bsearch.  It assumes the key and the element are pointers
 * to unsigned 8 bit integers.
 *
 * pv_key     Pointer to the key to be compared to the element.
 * pv_element Pointer to the element to be compared to the key.
 */
static int tsb_i2s_unipro_cmp_uint8(const void *pv_key, const void *pv_element)
{
    uint8_t *pn_key = (uint8_t *)pv_key;
    uint8_t *pn_element = (uint8_t *)pv_element;

    return (*pn_key - *pn_element);
}

/*
 * Compare function for bsearch.  It assumes the key and the element are pointers
 * to unsigned integers.
 *
 * pv_key     Pointer to the key to be compared to the element.
 * pv_element Pointer to the element to be compared to the key.
 */
static int tsb_i2s_unipro_cmp_uint(const void *pv_key, const void *pv_element)
{
    unsigned int *pn_key = (unsigned int *)pv_key;
    unsigned int *pn_element = (unsigned int *)pv_element;

    return (*pn_key - *pn_element);
}

/*
 * Find an element in an array given a compare function.
 *
 * Calls bsearch to find an element containing parameter what in an array of
 * where using cmp_fcn.  Please note what must be a pointer, as bsearch needs
 * a pointer to the value, not an actual value.
 *
 * what    A pointer to the value to search for in the array.
 * where   The array to search for the value in.
 * cmp_fcn Pointer to the function to use for the compare.
 *
 * Returns a pointer to the value found in the table or NULL if it is not found.
 */
#define TSB_I2S_TSB_FIND_ELEMENT(what, where, cmp_fcn) \
            bsearch(what, where, ARRAY_SIZE(where), sizeof(where[0]), cmp_fcn)

/**
  * @brief Setup the Unipro hardware for transmit and receive.
  *
  * @param sample_rate      The sample rate for all transactions in samples per second.
  *                         The currently supported rates are: 8000, 16000, 44100, 48000.
  * @param sample_size_bits The number of bits per sample.  Supported sizes are: 16, 24 and 32.
  * @param mode             The type of transfer.  See 2S_UNIPRO_I2S_MODE_T for details.
  * @param flags            The edge information for TX, RX, Word Clock and
  *                         master/slave.  See I2S_TUNNEL_I2S_FLAGS_* for more information.
  */
int i2s_unipro_tunnel_i2s_config(
    unsigned int sample_rate,
    I2S_TUNNEL_I2S_MODE_T mode,
    uint8_t sample_size_bits,
    uint8_t flags)
{
    const unsigned int rate_validation_tb[] =
    {
        8000,
        16000,
        44100,
        48000,
        88200,
        96000
    };
    const struct tsb_i2s_unipro_bits_to_reg_s
    {
        uint8_t bits;
        uint8_t bytes_per_sample;
        uint8_t reg_audioset_wordlen;
    } size_to_reg_tb[] =
    {
        {16, 2, 0b010000},
        {24, 4, 0b011000},
        {32, 4, 0b100000}
    };
    const struct tsb_i2s_unipro_mode_to_reg_s
    {
        uint8_t reg_modeset_ws;
        uint8_t num_channels;
    } mode_to_reg_ws_tb[I2S_TUNNEL_I2S_MODE_END] =
    {
        {0b100, 1 }, /* I2S_TUNNEL_I2S_MODE_PCM_MONO      */
        {0b101, 1 }, /* I2S_TUNNEL_I2S_MODE_PCM_MONO_REV  */
        {0b000, 2 }, /* I2S_TUNNEL_I2S_MODE_I2S_STEREO    */
        {0b010, 2 }, /* I2S_TUNNEL_I2S_MODE_LR_STEREO     */
        {0b011, 2 }  /* I2S_TUNNEL_I2S_MODE_LR_STEREO_REV */
    };
    void *p;
    unsigned int bytes_per_sample;
    uint32_t reg_audioset;
    uint32_t reg_modeset;

    if (!g_i2s_unipro_tunnel.enabled)
    {
        return -EIO;
    }
    /* Validate the rate. */
    p = TSB_I2S_TSB_FIND_ELEMENT(&sample_rate, rate_validation_tb, tsb_i2s_unipro_cmp_uint);
    if (p == NULL)
    {
        lldbg("Invalid sample rate: %d\n", sample_rate);
        return -EINVAL;
    }
    /* Validate and find the bit size. */
    p = TSB_I2S_TSB_FIND_ELEMENT(&sample_size_bits, size_to_reg_tb, tsb_i2s_unipro_cmp_uint8);
    if (p == NULL)
    {
        lldbg("Invalid bit size: %d\n", sample_size_bits);
        return -EINVAL;
    }
    bytes_per_sample = ((struct tsb_i2s_unipro_bits_to_reg_s *)p)->bytes_per_sample;
    reg_audioset = ((struct tsb_i2s_unipro_bits_to_reg_s *)p)->reg_audioset_wordlen <<
                   TSB_I2S_REG_AUDIOSET_WORDLEN_POS;
    reg_audioset |= (bytes_per_sample == 2 ? 0 : 1) << TSB_I2S_REG_AUDIOSET_SCLKTOWS_POS;
    reg_audioset |= (flags & I2S_TUNNEL_I2S_FLAGS_LR_EDGE_RISING) ? TSB_I2S_REG_AUDIOSET_EDGE : 0;
    reg_audioset |= TSB_I2S_REG_AUDIOSET_DTFMT;
    /* Validate the mode. */
    if (mode >= I2S_TUNNEL_I2S_MODE_END)
    {
        lldbg("Invalid mode: %d\n", mode);
        return -EINVAL;
    }
    reg_modeset = mode_to_reg_ws_tb[mode].reg_modeset_ws;
    /*
     * Calculate the BCLK rate needed for the settings.  The clock rate will be
     * setup when I2S is enabled to save power.
     */
    bytes_per_sample *= mode_to_reg_ws_tb[mode].num_channels;
    g_i2s_unipro_tunnel.bytes_per_sample = bytes_per_sample;
    g_i2s_unipro_tunnel.bclk_rate = sample_rate*bytes_per_sample * 8;
    g_i2s_unipro_tunnel.is_master = ((flags & I2S_TUNNEL_I2S_FLAGS_MASTER) == I2S_TUNNEL_I2S_FLAGS_MASTER);

    tsb_i2s_unipro_i2s_wait_clear(TSB_I2S_REG_SC_BASE + TSB_I2S_REG_BUSY,
                                  TSB_I2S_REG_REGBUSY_AUDIOSETBUSY);
    putreg32(reg_audioset & TSB_I2S_REG_AUDIOSET_SC_MASK,
             TSB_I2S_REG_SC_BASE + TSB_I2S_REG_AUDIOSET);
    tsb_i2s_unipro_i2s_wait_clear(TSB_I2S_REG_SO_BASE + TSB_I2S_REG_BUSY,
                                  TSB_I2S_REG_REGBUSY_AUDIOSETBUSY);
    putreg32(reg_audioset| ((flags & I2S_TUNNEL_I2S_FLAGS_TX_EDGE_RISING) ? TSB_I2S_REG_AUDIOSET_SDEDGE : 0),
             TSB_I2S_REG_SO_BASE + TSB_I2S_REG_AUDIOSET);
    tsb_i2s_unipro_i2s_wait_clear(TSB_I2S_REG_SI_BASE + TSB_I2S_REG_BUSY,
                                  TSB_I2S_REG_REGBUSY_AUDIOSETBUSY);
    putreg32(reg_audioset | ((flags & I2S_TUNNEL_I2S_FLAGS_RX_EDGE_RISING) ? TSB_I2S_REG_AUDIOSET_SDEDGE : 0),
             TSB_I2S_REG_SI_BASE + TSB_I2S_REG_AUDIOSET);

    tsb_i2s_unipro_i2s_wait_clear(TSB_I2S_REG_SC_BASE + TSB_I2S_REG_BUSY,
                                  TSB_I2S_REG_REGBUSY_MODESETBUSY);
    putreg32(reg_modeset, TSB_I2S_REG_SC_BASE + TSB_I2S_REG_MODESET);
    tsb_i2s_unipro_i2s_wait_clear(TSB_I2S_REG_SO_BASE + TSB_I2S_REG_BUSY,
                                  TSB_I2S_REG_REGBUSY_MODESETBUSY);
    putreg32(reg_modeset, TSB_I2S_REG_SO_BASE + TSB_I2S_REG_MODESET);
    tsb_i2s_unipro_i2s_wait_clear(TSB_I2S_REG_SI_BASE + TSB_I2S_REG_BUSY,
                                  TSB_I2S_REG_REGBUSY_MODESETBUSY);
    putreg32(reg_modeset, TSB_I2S_REG_SI_BASE + TSB_I2S_REG_MODESET);
    return OK;
}

/**
 * @brief Start the flow of I2S data.
 *
 * @param start true when the data flow must be started or false when it must be
 *              stopped.
 */
void i2s_unipro_tunnel_start(bool start)
{
    reglog_log(0xb0, (uint32_t)start);
    if (start)
    {
        /*
         * Start the process if in loopback mode.  If in normal mode the slave will be started
         * in the arm function and the master will be started when the first packet is received.
         */
#if defined(CONFIG_I2S_TUNNEL_ROUNDTRIP_LOOPBACK)
        (void)tsb_i2s_unipro_cmd_i2s_start_loopback(start);
        reglog_log(0xb1, 0);
        tsb_i2s_unipro_tunnel_start_i2s();
#endif
    }
    else
    {
        g_i2s_unipro_tunnel.running = false;
    }
    reglog_log(0xbf, 0);
}


 /**
  * @brief Arms or disarms the tunneling of I2S data over unipro.
  *
  * @param enable Non zero enables tunneling zero disables.
  */
int i2s_unipro_tunnel_arm(bool enable)
{
    int error = 0;
    unsigned int i;
    uint32_t reg_clk_sel;

    reglog_log(0x50, (uint32_t)enable);
    if ((!g_i2s_unipro_tunnel.enabled) || (!g_i2s_unipro_tunnel.bclk_rate))
    {
        return -EIO;
    }
    /* If the state is not changing exit. */
    if ((enable ^ g_i2s_unipro_tunnel.armed) == false)
    {
        return OK;
    }
    if (enable)
    {
        reglog_log(0x51, (uint32_t)g_i2s_unipro_tunnel.armed);
        /* Reset the statistics. */
        g_i2s_unipro_tunnel.i2s_unipro_rx_packets = 0;
        g_i2s_unipro_tunnel.i2s_tx_samples_retransmitted = 0;
        g_i2s_unipro_tunnel.i2s_tx_samples_dropped = 0;
        g_i2s_unipro_tunnel.i2s_tx_buffers_dropped = 0;
        reg_clk_sel = 0;

#if !defined(CONFIG_I2S_TUNNEL_LOCAL_LOOPBACK)
        /*
         * If necessary allocate a Unipro buffer for the initial transmit, since
         * it will be freed in the TX DMA handler.
         */
        for (i = 0; i < TSB_I2S_UNIPRO_TUNNEL_BUF_NUM; i++)
        {
            if (g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].msg == NULL)
            {
                g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].msg =
                    (struct tsb_i2s_unipro_msg_s *)unipro_rxbuf_alloc(TSB_I2S_UNIPRO_TUNNEL_CPORTID);
                if (g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].msg == NULL)
                {
                    while (--i)
                    {
                        unipro_rxbuf_free(TSB_I2S_UNIPRO_TUNNEL_CPORTID,
                                          g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].msg);
                        g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].unipro_allocated = false;
                    }
                    return -ENOMEM;
                }
                memset(g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].msg,
                       0x00,
                       TSB_I2S_UNIPRO_TUNNEL_BUF_SZ + offsetof(struct tsb_i2s_unipro_msg_s, data.buf));
                g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].unipro_allocated = true;
                g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].len = TSB_I2S_UNIPRO_TUNNEL_BUF_SZ;
                reglog_log(0x52, (uint32_t)g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].msg);
            }
            else
            {
                lldbg("Unipro buffer already allocated.\n");
            }
        }
#endif
#if (defined(CONFIG_I2S_TUNNEL_LOCAL_LOOPBACK) || defined(CONFIG_I2S_TUNNEL_ROUNDTRIP_LOOPBACK))
        /* Prime the TX buffer with some data which can be monitored. */
        for (i = 0; i < TSB_I2S_UNIPRO_TUNNEL_BUF_SZ; i += 2)
        {
            g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[TSB_I2S_UNIPRO_TUNNEL_BUF_PING].msg->data.buf[i + 1] =
                (uint8_t)((i>>8)&0xff);
            g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[TSB_I2S_UNIPRO_TUNNEL_BUF_PONG].msg->data.buf[i + 1] =
                (uint8_t)((i>>8)&0xff);
            g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[TSB_I2S_UNIPRO_TUNNEL_BUF_PING].msg->data.buf[i] =
                (uint8_t)(i&0xff);
            g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[TSB_I2S_UNIPRO_TUNNEL_BUF_PONG].msg->data.buf[i] =
                (uint8_t)(i&0xff);
        }

#endif
       /*
        * Setup the clock.  The I2S controller always divides MCLK by four and optionally
        * by two again to get the BCLK.
        */
        /*
         * TODO: Need to update the PLL code to handle an offset for clock
         * differences between the APBA and the APBE.
         */
        error = device_pll_set_frequency(g_i2s_unipro_tunnel.pll_dev,
                                         g_i2s_unipro_tunnel.bclk_rate * 4);
        if (!error)
        {
            error = device_pll_start(g_i2s_unipro_tunnel.pll_dev);
        }
        if (error)
        {
            return error;
        }
        if (!g_i2s_unipro_tunnel.is_master)
        {
            /*
             * The MCLK is not currently externally connected, thus the internal
             * clock must be used.  This may change on other hardware.
             */
            reg_clk_sel = TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_LR_BCLK_SEL;
        }
        putreg32(reg_clk_sel, TSB_SYSCTL_BASE + TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR);

        /* Set the start threshold. */
        putreg32(TSB_I2S_TX_START_THRESHOLD, TSB_I2S_REG_SO_BASE + TSB_I2S_REG_TX_SSIZE);

        /* Clear any pending interrupts. */
        tsb_i2s_unipro_clr_irq(TSB_I2S_REG_SO_BASE);
        tsb_i2s_unipro_clr_irq(TSB_I2S_REG_SI_BASE);

        /* Start the bit clock. */
        tsb_clk_enable(TSB_CLK_I2SBIT);
        tsb_reset(TSB_RST_I2SBIT);

        g_i2s_unipro_tunnel.armed = true;
        error = tsb_i2s_unipro_start_i2s_tx_dma(128);

        /* Enable the TX DMA request and error interrupts. */
        putreg32(TSB_I2S_REG_INT_INT, TSB_I2S_REG_SO_BASE + TSB_I2S_REG_INTMASK);

        /* Setup RX DMA. */
        (void)tsb_i2s_unipro_start_i2s_rx_dma();

        /* Wait for the measurement process to complete. */
        (void)tsb_i2s_unipro_wait_for_measruement();

        /*
         * If in slave mode, go ahead and start the I2S hardware, since nothing
         * will happen until a clock is received.
         */
        if (!g_i2s_unipro_tunnel.is_master)
        {
            tsb_i2s_unipro_tunnel_start_i2s();
        }
    }
    else
    {
        reglog_log(0x5e, 0);
        g_i2s_unipro_tunnel.armed = false;
        /* If running stop. */
        if (g_i2s_unipro_tunnel.running)
        {
            /*
             * Clear the armed bit and let the TX DMA interrupt shut things down once
             * it fires.
             */
            tsb_i2s_unipro_tunnel_stop_i2s_dma();
        }
    }
    reglog_log(0x5f, 0);
    return error;
}

/**
 * @brief Enable or disable the i2s hardware if not already enabled.
 *
 * Enables and disables the hardware.  It can be left enabled for short periods
 * of time, but should be disabled to save power if it will not be used for
 * a while.
 *
 * @param enable true if the hardware must be enabled or false for disable.
 */
int i2s_unipro_tunnel_enable(bool enable)
{
    int error;

    reglog_log(0xc0, (uint32_t)enable|(g_i2s_unipro_tunnel.enabled<<1));
    /* If the state has not changed, do nothing. */
    if ((enable ^ g_i2s_unipro_tunnel.enabled) == 0)
    {
        return OK;
    }
    if (enable)
    {
        /* Make sure Unipro has been registered. */
        if (!g_i2s_unipro_tunnel.unipro_registered)
        {
            reglog_log(0xc2, 0);
            lldbg("Unipro not registered.\n");
            return -EIO;
        }
        /* Open the PLL. */
        reglog_log(0xc3, 0);
        g_i2s_unipro_tunnel.pll_dev = device_open(DEVICE_TYPE_PLL_HW, TSB_I2S_PLLA_ID);
        if (!g_i2s_unipro_tunnel.pll_dev)
        {
            reglog_log(0xc4, 0);
            lldbg("Unable to open audio pll.\n");
            return -EIO;
        }
        /* Enable the free running timer for use by the measurement code. */
        g_i2s_unipro_tunnel.fr_tmr_id = tsb_fr_tmr_reserve();
        if (g_i2s_unipro_tunnel.fr_tmr_id < 0)
        {
            return g_i2s_unipro_tunnel.fr_tmr_id;
        }
        tsb_fr_tmr_start(g_i2s_unipro_tunnel.fr_tmr_id);

        /* Start the measurement process.  This should only run on the APBE. */
        error = tsb_i2s_unipro_cmd_measure_start();
        if (error != OK)
        {
            return error;
        }

        /* Enable the system clock. */
        reglog_log(0xc5, (uint32_t)g_i2s_unipro_tunnel.pll_dev);
        tsb_clk_enable(TSB_CLK_I2SSYS);
        tsb_reset(TSB_RST_I2SSYS);

        /* Enable the interrupt handlers. */
        reglog_log(0xc6, 0);
        up_enable_irq(TSB_IRQ_I2SOERR);
        up_enable_irq(TSB_IRQ_I2SIERR);
        up_enable_irq(TSB_IRQ_I2SI);

        g_i2s_unipro_tunnel.enabled = true;
    }
    else
    {
        /* Disable all I2S interrupts. */
        up_disable_irq(TSB_IRQ_I2SOERR);
        up_disable_irq(TSB_IRQ_I2SIERR);
        up_disable_irq(TSB_IRQ_I2SI);

        /* Close the PLL. */
        device_close(g_i2s_unipro_tunnel.pll_dev);

        /* Disable the system clock. */
        tsb_clk_disable(TSB_CLK_I2SSYS);

        /* Setup to remeasure at the next enable. */
        g_i2s_unipro_tunnel.roundtrip_timer_ticks = 0;

        /* Release the free running timer. */
        tsb_fr_tmr_stop(g_i2s_unipro_tunnel.fr_tmr_id);
        tsb_fr_tmr_release(g_i2s_unipro_tunnel.fr_tmr_id);

        g_i2s_unipro_tunnel.enabled = false;
    }
    reglog_log(0xcf, 0);
    return OK;
}

/*
 * Define the Unipro driver for receiving data from the remote side.
 */
static const struct unipro_driver i2s_unipro_tunnel_unipro_driver =
{
    .name = "I2S Tunneling",
    .rx_handler = tsb_i2s_unipro_tunnel_unipro_rx
};

/**
 * @brief Register the Unipro driver for I2S tunneling.
 *
 * This is not done as part of the init process on the APBA, since it cannot
 * be done until after Unipro is up.  For the APBA this means it must be done
 * once Unipro is running.  However on the APBE it can be started as part of the
 * init process since Unipro will be up and running.
 */
int i2s_unipro_tunnel_unipro_register(void)
{
    int ret;

    ret = unipro_driver_register((struct unipro_driver *)&i2s_unipro_tunnel_unipro_driver,
                                 TSB_I2S_UNIPRO_TUNNEL_CPORTID);
    if (ret == 0)
    {
        /* Only one side needs to make the point to point connection. */
#if defined(CONFIG_UNIPRO_P2P_APBA)
        unipro_p2p_setup_connection(TSB_I2S_UNIPRO_TUNNEL_CPORTID);
#endif
        g_i2s_unipro_tunnel.unipro_registered = true;
    }
    return ret;
}

int i2s_unipro_tunnel_unipro_unregister(void)
{
    g_i2s_unipro_tunnel.unipro_registered = false;
    (void)unipro_driver_unregister(TSB_I2S_UNIPRO_TUNNEL_CPORTID);
    return 0;
}

/*
 * Initialize a buffer structure.
 *
 * buf_p Pointer to the buffer to be initialized.
 * tx    true if the buffer is a transmit buffer and false if it is a receive buffer.
 */
static int tsb_i2s_unipro_tunnel_dma_op_alloc(
    struct tsb_i2s_unipro_tunnel_buf_s *buf_p,
    bool tx)
{
    int error;

    error = device_dma_op_alloc(g_i2s_unipro_tunnel.dma_dev, 1, 0, &buf_p->dma_op);
    if (error != OK)
    {
        lowsyslog("I2S Unipro Tunnel: Unable to allocate DMA ops.\n");
        return error;
    }
    buf_p->dma_op->callback_arg = (void *)TSB_I2S_UNIPRO_TUNNEL_BUF_PING;
    buf_p->dma_op->callback_events = DEVICE_DMA_CALLBACK_EVENT_COMPLETE;
    buf_p->dma_op->sg_count = 1;
    if (tx)
    {
        buf_p->dma_op->callback = (void *)tsb_i2s_unipro_i2s_tx_dma_callback;
        buf_p->dma_op->sg[0].src_addr = (off_t)buf_p->msg->data.buf;
        buf_p->dma_op->sg[0].dst_addr = (off_t)(TSB_I2S_REG_SO_BASE + TSB_I2S_REG_LMEM00);
    }
    else
    {
        buf_p->dma_op->callback = (void *)tsb_i2s_unipro_i2s_rx_dma_callback;
        buf_p->dma_op->sg[0].src_addr = (off_t)(TSB_I2S_REG_SI_BASE + TSB_I2S_REG_LMEM00);
        buf_p->dma_op->sg[0].dst_addr = (off_t)buf_p->msg->data.buf;
    }
    /* These will be overwritten later and do not necessarily need to be initialized. */
    buf_p->dma_op->sg[0].len = buf_p->len;
    return OK;
}

/**
 * @brief Get data on the operational status of the tunneling driver.
 *
 * @param info Pointer to be populated with the driver data of interest.
 */
int i2s_unipro_tunnel_info(struct i2s_tunnel_info_s *info)
{
    info->enabled = g_i2s_unipro_tunnel.enabled;
    info->is_master = g_i2s_unipro_tunnel.is_master;
    info->bclk_rate = g_i2s_unipro_tunnel.bclk_rate;
    info->bytes_per_sample = g_i2s_unipro_tunnel.bytes_per_sample;
    info->roundtrip_timer_ticks = g_i2s_unipro_tunnel.roundtrip_timer_ticks;
    info->timer_offset = g_i2s_unipro_tunnel.timer_offset;
    info->i2s_tunnel_rx_packets =  g_i2s_unipro_tunnel.i2s_unipro_rx_packets;
    info->i2s_tunnel_packet_size = TSB_I2S_UNIPRO_TUNNEL_BUF_SZ;
    info->i2s_tx_samples_dropped = g_i2s_unipro_tunnel.i2s_tx_samples_dropped;
    info->i2s_tx_samples_retransmitted = g_i2s_unipro_tunnel.i2s_tx_samples_retransmitted;
    info->i2s_tx_buffers_dropped = g_i2s_unipro_tunnel.i2s_tx_buffers_dropped;
    return OK;
}

/**
 * @brief Cleans up and releases the resources assigned in i2s_unipro_tunnel_init().
 */
void i2s_unipro_tunnel_deinit(void)
{
    size_t i;

    if (g_i2s_unipro_tunnel.initialized == TSB_I2S_UNIPRO_TUNNEL_INIT)
    {
        for (i = 0; i < TSB_I2S_UNIPRO_TUNNEL_BUF_NUM; i++)
        {
            if (g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].msg)
            {
                bufram_free(g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].msg);
            }
            if (g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].dma_op)
            {
                device_dma_op_free(g_i2s_unipro_tunnel.dma_dev,
                                   g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].dma_op);
            }
            if (g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].dma_op)
            {
                device_dma_op_free(g_i2s_unipro_tunnel.dma_dev,
                                   g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i].dma_op);
            }
        }
        if (g_i2s_unipro_tunnel.i2s_rx_dma_channel)
        {
            device_dma_chan_free(g_i2s_unipro_tunnel.dma_dev,
                                 &g_i2s_unipro_tunnel.i2s_rx_dma_channel);
        }
        if (g_i2s_unipro_tunnel.i2s_tx_dma_channel)
        {
            device_dma_chan_free(g_i2s_unipro_tunnel.dma_dev,
                                 &g_i2s_unipro_tunnel.i2s_tx_dma_channel);
        }
        (void)irq_detach(TSB_IRQ_I2SOERR);
        (void)irq_detach(TSB_IRQ_I2SIERR);
        (void)irq_detach(TSB_IRQ_I2SI);
        (void)tsb_release_pinshare(TSB_PIN_ETM | TSB_PIN_GPIO16 | TSB_PIN_GPIO18 |
                                   TSB_PIN_GPIO19 | TSB_PIN_GPIO20);
        g_i2s_unipro_tunnel.initialized = 0;
    }
}

/**
 * @brief Initialize the I2S Unipro tunneling driver.
 *
 * This must be called at startup to allocate and initialize the necessary buffers
 * and hardware drivers used by the I2S Unipro tunneling driver.  Failure to call
 * this before using the drive will cause bad things to happen.
 */
int i2s_unipro_tunnel_init(void)
{
    int error;
    size_t i;
    const struct device_dma_params i2s_rx_dma_params =
    {
        .src_dev = DEVICE_DMA_DEV_IO,
        .src_devid = TSB_I2S_SI_DMA_DEVID,
        .src_inc_options = DEVICE_DMA_INC_NOAUTO,
        .dst_dev = DEVICE_DMA_DEV_MEM,
        .dst_devid = 0,
        .dst_inc_options = DEVICE_DMA_INC_AUTO,
        .transfer_size = DEVICE_DMA_TRANSFER_SIZE_32,
        .burst_len = DEVICE_DMA_BURST_LEN_1,
        /*
         * If the data was to be use internally, swap would need to be setup.
         * However, since the data is simply sent out another I2S port it is left
         * in the bus format.
         */
        .swap = DEVICE_DMA_SWAP_SIZE_NONE,
    };
    const struct device_dma_params i2s_tx_dma_params =
    {
        .src_dev = DEVICE_DMA_DEV_MEM,
        .src_devid = 0,
        .src_inc_options = DEVICE_DMA_INC_AUTO,
        .dst_dev = DEVICE_DMA_DEV_IO,
        .dst_devid = TSB_I2S_SO_DMA_DEVID,
        .dst_inc_options = DEVICE_DMA_INC_NOAUTO,
        .transfer_size = DEVICE_DMA_TRANSFER_SIZE_32,
        .burst_len = DEVICE_DMA_BURST_LEN_1,
        /*
         * If the data was to be use internally, swap would need to be setup.
         * However, since the data is simply sent out another I2S port it is left
         * in the bus format.
         */
        .swap = DEVICE_DMA_SWAP_SIZE_NONE,
    };

    if (g_i2s_unipro_tunnel.initialized != TSB_I2S_UNIPRO_TUNNEL_INIT)
    {
        memset(&g_i2s_unipro_tunnel, 0, sizeof(g_i2s_unipro_tunnel));
        g_i2s_unipro_tunnel.initialized = TSB_I2S_UNIPRO_TUNNEL_INIT;

        /*
         * Disable the clock in case it was on from startup.  This is required to
         * make sure RX and TX stay in sync.
         */
        tsb_clk_disable(TSB_CLK_I2SSYS);
        tsb_clk_disable(TSB_CLK_I2SBIT);

        for (i = 0; i < TSB_I2S_UNIPRO_TUNNEL_BUF_NUM; i++)
        {
            /*
             * Allocate the Unipro RX buffers.  The TX buffers are allocated from
             * Unipro memory when the transfer is started since they are freed after
             * each packet is transmitted, unlike the receive buffers which are kept.
             */
            size_t alloc_size = TSB_I2S_UNIPRO_TUNNEL_BUF_SZ +
                offsetof(struct tsb_i2s_unipro_msg_s, data.buf);

            g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].msg =
                (struct tsb_i2s_unipro_msg_s *)bufram_alloc(alloc_size);

            g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].len = TSB_I2S_UNIPRO_TUNNEL_BUF_SZ;
            if (g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].msg == NULL)
            {
                lldbg("I2S Unipro Tunnel: Insufficient memory to allocate buffers.\n");
                error = -ENOMEM;
                goto i2s_tunnel_init_error;
            }
            memset(g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i].msg, 0, alloc_size);
        }

        /* Allocate the I2S DMA channels. */
        g_i2s_unipro_tunnel.dma_dev = device_open(DEVICE_TYPE_DMA_HW, 0);
        if (!g_i2s_unipro_tunnel.dma_dev)
        {
            lldbg("I2S Unipro Tunnel: Unable to open DMA device.\n");
            error = -ENODEV;
            goto i2s_tunnel_init_error;
        }
        device_dma_chan_alloc(g_i2s_unipro_tunnel.dma_dev,
                              (struct device_dma_params *)&i2s_rx_dma_params,
                              &g_i2s_unipro_tunnel.i2s_rx_dma_channel);
        device_dma_chan_alloc(g_i2s_unipro_tunnel.dma_dev,
                              (struct device_dma_params *)&i2s_tx_dma_params,
                              &g_i2s_unipro_tunnel.i2s_tx_dma_channel);
        if (((uint32_t)g_i2s_unipro_tunnel.i2s_rx_dma_channel &
            (uint32_t)g_i2s_unipro_tunnel.i2s_tx_dma_channel) == (uint32_t)NULL)
        {
            lldbg("I2S Unipro Tunnel: Unable to allocate I2S DMA channels.\n");
            error = -ENODEV;
            goto i2s_tunnel_init_error;
        }

        /* Allocate the I2S DMA ops. */
        error = 0;
        for (i = 0; i < TSB_I2S_UNIPRO_TUNNEL_BUF_NUM; i++)
        {
            error |= tsb_i2s_unipro_tunnel_dma_op_alloc(&g_i2s_unipro_tunnel.i2s_rx_unipro_tx_buf[i],
                                                        false);
            error |= tsb_i2s_unipro_tunnel_dma_op_alloc(&g_i2s_unipro_tunnel.i2s_tx_unipro_rx_buf[i],
                                                        true);
        }
        if (error != OK)
        {
            goto i2s_tunnel_init_error;
        }

        /* Register the error handlers. */
        error = irq_attach(TSB_IRQ_I2SOERR, tsb_i2s_unipro_i2s_out_error_handler);
        error |= irq_attach(TSB_IRQ_I2SIERR, tsb_i2s_unipro_i2s_in_error_handler);
        error |= irq_attach(TSB_IRQ_I2SI, tsb_i2s_unipro_i2s_in_data);
        if (error != OK)
        {
            lldbg("I2S Unipro Tunnel: Unable to register I2S error handlers.\n");
            goto i2s_tunnel_init_error;
        }

        /* Assign the correct pins to I2S. */
        error = tsb_request_pinshare(TSB_PIN_ETM | TSB_PIN_GPIO16 | TSB_PIN_GPIO18 |
                                     TSB_PIN_GPIO19 | TSB_PIN_GPIO20);
        if (error)
        {
            lldbg("I2S Unipro Tunnel: Unable to gain ownership of I2S pins.\n");
            goto i2s_tunnel_init_error;
        }
        tsb_clr_pinshare(TSB_PIN_ETM);
        tsb_clr_pinshare(TSB_PIN_GPIO16);
        tsb_clr_pinshare(TSB_PIN_GPIO18);
        tsb_clr_pinshare(TSB_PIN_GPIO19);
        tsb_clr_pinshare(TSB_PIN_GPIO20);
        lldbg("I2S Unipro Tunnel Initialized\n");
    }
    return OK;

i2s_tunnel_init_error:
    i2s_unipro_tunnel_deinit();
    return error;
}
