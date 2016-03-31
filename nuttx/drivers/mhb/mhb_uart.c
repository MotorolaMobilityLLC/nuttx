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

#include <nuttx/config.h>

#include <crc16_poly8005.h>
#include <errno.h>
#include <string.h>
#include <queue.h>
#include <unistd.h>

#include <arch/atomic.h>
#include <arch/byteorder.h>

#include <nuttx/util.h>
#include <nuttx/clock.h>
#include <nuttx/device.h>
#include <nuttx/device_uart.h>
#include <nuttx/gpio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/time.h>

#include <nuttx/mhb/device_mhb.h>

#define MHB_UART_SYNC_FLAG   (0x7f)
#define MHB_UART_SYNC_LENGTH (32)

#define MHB_SIG_STOP (9)

struct mhb_queue_item {
    sq_entry_t entry;
    size_t size;
    int flags;
    uint8_t buffer[0];
};

struct mhb {
    atomic_t ref_count;
    struct device *self;

    unsigned int uart_dev_id;
    struct device *dev;
    /* gpio */
    uint32_t local_wake;
    uint32_t peer_wake;
    /* rx */
    pthread_t rx_thread;
    /* tx */
    sem_t tx_sem;
    sq_queue_t tx_queue;
    pthread_t tx_thread;
    /* callbacks */
    /* IMPORTANT: Dependent on the implementation of _mhb_addr_to_index() */
    mhb_receiver receivers[MHB_FUNC_MAX + 1];
    /* pm */
    atomic_t pm_state_peer;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    /* uart */
    uint32_t current_baud;
    uint32_t new_baud;
};

static struct mhb *g_mhb;

static int mhb_send(struct device *dev, struct mhb_hdr *hdr,
    uint8_t *payload, size_t payload_length, int flags);

#define PM_HANDSHAKE_TIMEOUT 1000 /* ms */

#define MHB_INVALID_GPIO (0xffffffff)

static void mhb_assert_peer_wake(void) {
    if (gpio_get_value(g_mhb->peer_wake) == 0) {
        gpio_set_value(g_mhb->peer_wake, 1);
        usleep(10000);
    }

    gpio_set_value(g_mhb->peer_wake, 0);
}

static void mhb_assert_peer_wake_ack(void) {
    if (g_mhb->peer_wake == MHB_INVALID_GPIO) {
        return;
    }

    gpio_set_value(g_mhb->peer_wake, 1);
}

/*
 * Called from wake isr handler. Wake Ack is sent back
 * and PM control will be performed in TX thread context
 * according to the FLAG value.
 */
static void mhb_handle_pm_wake_req(void)
{
    /* If we are responding to wake up request, consider the remote is awake. */
    atomic_init(&g_mhb->pm_state_peer, 1);

    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_PM;
    hdr.type = MHB_TYPE_PM_WAKE_RSP;
    hdr.result = MHB_RESULT_SUCCESS;

    if (mhb_send(g_mhb->self, &hdr, NULL, 0, 0)) {
        lldbg("ERROR: Failed to send wake rsp\n");
    }
}

static void mhb_handle_pm_wake_rsp(void)
{
    atomic_init(&g_mhb->pm_state_peer, 1);

    mhb_assert_peer_wake_ack();

    pthread_mutex_lock(&g_mhb->mutex);
    pthread_cond_signal(&g_mhb->cond);
    pthread_mutex_unlock(&g_mhb->mutex);
}

static void mhb_handle_pm_sleep_req(void)
{
    struct mhb_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.addr = MHB_ADDR_PM;
    hdr.type = MHB_TYPE_PM_SLEEP_RSP;
    hdr.result = MHB_RESULT_SUCCESS;

    if (mhb_send(g_mhb->self, &hdr, NULL, 0, 0)) {
        lldbg("ERROR: Failed to send sleep rsp\n");
    }
}

static void mhb_handle_pm_status_not(struct mhb_hdr *hdr,
    uint8_t *payload, size_t payload_length)
{
    struct mhb_pm_status_not *not = (struct mhb_pm_status_not *)payload;

    if (payload_length != sizeof(*not)) {
        lldbg("ERROR: Invalid PM status notify\n");
        return;
    }

    lldbg("PM status: %d\n", not->status);
}

static int mhb_local_wake_irq(int irq, FAR void *context) {
    gpio_clear_interrupt(irq);
    mhb_handle_pm_wake_req();
    return 0;
}

/* Utility function to wait pthread_cond with timeout */
static bool mhb_wait_or_timeout(long long timeout) {
    struct timespec expires;

    if (clock_gettime(CLOCK_REALTIME, &expires)) {
        return false;
    }

    uint64_t new_ns = timespec_to_nsec(&expires);
    new_ns += timeout * NSEC_PER_MSEC;
    nsec_to_timespec(new_ns, &expires);

    int ret = pthread_cond_timedwait(&g_mhb->cond,
                                     &g_mhb->mutex,
                                     &expires);
    if (ret) {
        /* timeout or other erros */
        lldbg("ERROR: wait error %d\n", ret);
        return false;
    }

    return true;
}

/*
 * Function called in TX thread context to wake up AP.
 * The function will block until receiving WAKE_ACK from AP
 * or timeout.
 */
static void mhb_wake_peer(void)
{
    if (atomic_get(&g_mhb->pm_state_peer)) {
        return;
    }

    if (g_mhb->peer_wake == MHB_INVALID_GPIO) {
        /* The peer does not support a wake interrupt.
           Assume the peer is always awake. */
        atomic_init(&g_mhb->pm_state_peer, 1);
        return;
    }

    pthread_mutex_lock(&g_mhb->mutex);

    /* Assert Wake INT */
    mhb_assert_peer_wake();

    if (!mhb_wait_or_timeout(PM_HANDSHAKE_TIMEOUT)) {
        lldbg("ERROR: Timed out for Wake Handshake\n");
    }

    pthread_mutex_unlock(&g_mhb->mutex);
}

static int mhb_handle_pm(struct mhb_hdr *hdr,
    uint8_t *payload, size_t payload_length)
{
    switch (hdr->type) {
    case MHB_TYPE_PM_WAKE_REQ:
        mhb_handle_pm_wake_req();
        break;
    case MHB_TYPE_PM_WAKE_RSP:
        mhb_handle_pm_wake_rsp();
        break;
    case MHB_TYPE_PM_SLEEP_RSP:
        /* no specific action */
        break;
    case MHB_TYPE_PM_SLEEP_REQ:
        mhb_handle_pm_sleep_req();
        break;
    case MHB_TYPE_PM_STATUS_NOT:
        mhb_handle_pm_status_not(hdr, payload, payload_length);
        break;
    default:
        lldbg("ERROR: unknown pm event\n");
        break;
    }

    return 0;
}

static void mhb_complete_baud_change(uint32_t new_baud)
{
#if CONFIG_MHB_UART_FLOWCONTROL
    const int flow = 1;
#else
    const int flow = 0;
#endif
    int ret;

    /* Ensure UART TX FIFO is emptied so the ACK is fully sent out */
    device_uart_flush_transmitter(g_mhb->dev);

    ret = device_uart_set_configuration(g_mhb->dev, new_baud,
                                        NO_PARITY, 8 /* bits */, ONE_STOP_BIT,
                                        flow);
    if (ret) {
        lldbg("error switching baud: %d\n", ret);
    }
}

static void mhb_handle_uart_config_req(uint8_t *payload, size_t payload_length)
{
    struct mhb_uart_config_req *req = (struct mhb_uart_config_req *)payload;

    if (payload_length != sizeof(*req)) {
        return;
    }

    req->baud = le32_to_cpu(req->baud);
    if (req->baud != g_mhb->current_baud) {
        g_mhb->new_baud = req->baud;
    }

    struct mhb_hdr rsp_hdr;
    memset(&rsp_hdr, 0, sizeof(rsp_hdr));
    rsp_hdr.addr = MHB_ADDR_UART;
    rsp_hdr.type = MHB_TYPE_UART_CONFIG_RSP;
    rsp_hdr.result =
        (req->baud == g_mhb->new_baud ? MHB_RESULT_SUCCESS : MHB_RESULT_RETRY);

    struct mhb_uart_config_rsp rsp;
    rsp.baud = cpu_to_le32(g_mhb->new_baud);

    if (mhb_send(g_mhb->self, &rsp_hdr, (uint8_t *)&rsp, sizeof(rsp), 0)) {
        lldbg("ERROR: Failed to send rsp\n");
    }
}

static int mhb_handle_uart(struct mhb_hdr *hdr,
    uint8_t *payload, size_t payload_length)
{
    switch (hdr->type) {
    case MHB_TYPE_UART_CONFIG_REQ:
        mhb_handle_uart_config_req(payload, payload_length);
        break;
    default:
        lldbg("ERROR: unknown uart event\n");
        break;
    }

    return 0;
}

static ssize_t _mhb_addr_to_index(uint8_t addr)
{
    ssize_t index;

    if (addr & MHB_PEER_MASK) {
        lldbg("ERROR: peer callbacks not supported\n");
        return -ENOSYS;
    }

    /* Compress the valid addresses into a contiguous array.
       Only MHB_FUNC_CDSI supports more than one instance.
       Map the first instance of MHB_FUNC_* to the start of the array.
       Map the second instance of MHB_FUNC_CDSI to the next entry.
     */
    if (addr == MHB_ADDR_CDSI1) {
        index = MHB_FUNC_MAX;
    } else {
        index = (addr >> MHB_FUNC_SHIFT) & MHB_FUNC_BIT;
    }

    if (index >= ARRAY_SIZE(g_mhb->receivers)) {
        lldbg("FATAL: Invalid index: %d\n", index);
        return -ENODEV;
    }

    return index;
}

static int mhb_register_receiver(struct device *dev,
    uint8_t addr, mhb_receiver receiver)
{
    int ret;

    irqstate_t flags;
    flags = irqsave();

    ssize_t index = _mhb_addr_to_index(addr);
    if (index < 0) {
        ret = index;
        goto err_irqrestore;
    }

    g_mhb->receivers[index] = receiver;
    ret = 0;

err_irqrestore:
    irqrestore(flags);
    return ret;
}

static int mhb_unregister_receiver(struct device *dev,
    uint8_t addr, mhb_receiver receiver)
{
    int ret;

    irqstate_t flags;
    flags = irqsave();

    ssize_t index = _mhb_addr_to_index(addr);
    if (index < 0) {
        ret = index;
        goto err_irqrestore;
    }

    g_mhb->receivers[index] = NULL;
    ret = 0;

err_irqrestore:
    irqrestore(flags);
    return ret;
}

static int mhb_callback(struct mhb_hdr *hdr,
    uint8_t *payload, size_t payload_length)
{
    uint8_t funcid = (hdr->addr >> MHB_FUNC_SHIFT) & MHB_FUNC_BIT;

    if (hdr->rsvd != 0) {
        /* Require the reserved field to be 0. This provides the ability
           to add forward-compatibility. */
        lldbg("ERROR: invalid reserved field\n");
        return -EINVAL;
    }

    switch (funcid) {
    case MHB_FUNC_PM:
        return mhb_handle_pm(hdr, payload, payload_length);
    case MHB_FUNC_UART:
        return mhb_handle_uart(hdr, payload, payload_length);
    default: {
            ssize_t index = _mhb_addr_to_index(hdr->addr);
            if (index < 0) {
                lldbg("ERROR: invalid addr=%d.\n", hdr->addr);
                return index;
            }

            mhb_receiver receiver = g_mhb->receivers[index];
            if (receiver) {
                return receiver(g_mhb->self, hdr, payload, payload_length);
            } else {
                lldbg("WARNING: no receiver\n");
                return -EINVAL;
            }
        }
    }
}

#if CONFIG_MHB_UART_SEND_SYNC
static int _mhb_send_sync_pattern(void)
{
    struct mhb_queue_item *item;
    size_t length = MHB_UART_SYNC_LENGTH;

    /* Allocate an item */
    uint8_t *buf = kmm_zalloc(sizeof(*item) + length);
    if (!buf) {
        return -ENOMEM;
    }

    /* Populate the queue item */
    item = (struct mhb_queue_item *)buf;
    item->size = length;

    /* Set the sync */
    memset(item->buffer, MHB_UART_SYNC_FLAG, length);

    /* Queue the buffer */
    irqstate_t iflags = irqsave();
    sq_addlast(&item->entry, &g_mhb->tx_queue);
    irqrestore(iflags);

    /* Signal the transmit thread that a buffer is available */
    sem_post(&g_mhb->tx_sem);

    return 0;
}
#endif

#if CONFIG_MHB_UART_WAIT_FOR_SYNC
static int _mhb_wait_for_sync_pattern(void)
{
    int ret = 0;
    size_t n = 0;

    while (!ret) {
        uint8_t rx_buf[1];
        int got = 0;
        ret = device_uart_start_receiver(g_mhb->dev, rx_buf, sizeof(rx_buf),
                      NULL /* dma */,
                      &got, NULL /* blocking */);
        if (ret || !got) {
            if (ret != -EINTR) {
                lldbg("FATAL: Failed to wait for sync: %d\n", ret);
            }
            break;
        }

        n = (rx_buf[0] == MHB_UART_SYNC_FLAG) ? n + 1 : 0;

        if (n == MHB_UART_SYNC_LENGTH) {
            /* sync found */
            break;
        }
    }

    return ret;
}
#endif

static void *mhb_rx_thread(void *data)
{
    int ret = 0;
    static uint8_t rx_buf[CONFIG_MHB_UART_RXBUFSIZE];
    size_t rx_size = 0;

#if CONFIG_MHB_UART_WAIT_FOR_SYNC
    ret = _mhb_wait_for_sync_pattern();
#endif

    while (!ret) {
        int got = 0;
        ret = device_uart_start_receiver(g_mhb->dev, rx_buf + rx_size,
                      sizeof(rx_buf) - rx_size, NULL /* dma */,
                      &got, NULL /* blocking */);
        if (ret) {
            if (ret != -EINTR) {
                lldbg("FATAL: Failed to start the receiver: %d\n", ret);
            }
            break;
        }

        rx_size += got;

        if (rx_size >= MHB_HDR_SIZE) {
            /* Have a complete data link header */
            struct mhb_hdr *rx_hdr = (struct mhb_hdr *)rx_buf;

            uint16_t length = le16_to_cpu(rx_hdr->length);
            if (rx_size >= length) {
                /* Have a complete packet */

                /* Reverse header length */
                rx_hdr->length = length;

                /* Check the CRC */
                uint16_t received_crc =
                    *((uint16_t *)(rx_buf + length - MHB_CRC_SIZE));
                received_crc = le16_to_cpu(received_crc);
                if (received_crc) {
                    /* Only check the CRC if it is non-zero.  Zero is a special
                       case that indicates CRC checking is disabled. */
                    uint16_t calculated_crc = crc16_poly8005(rx_buf,
                                length - sizeof(received_crc), CRC_INIT_VAL);
                    if (received_crc != calculated_crc) {
                        lldbg("ERROR: CRC mismatch: rx=%04x, calc=%04x\n",
                                                received_crc, calculated_crc);

                        // TODO: Tell the peer there is an error

                        /* Reset the rx size */
                        rx_size = 0;
                        continue;
                    }
                }

                /* Find the payload */
                uint8_t *payload = rx_buf + MHB_HDR_SIZE;
                size_t payload_length =
                    rx_hdr->length - MHB_HDR_SIZE - MHB_CRC_SIZE;

                /* Notify listeners */
                mhb_callback(rx_hdr, payload, payload_length);

                /* Reset the rx size */
                rx_size -= length;
                memmove(rx_buf, rx_buf + length, rx_size);
            } else if (length > MHB_MAX_MSG_SIZE) {
                lldbg("ERROR: Invalid length=%04x\n", length);

                 // TODO: Tell the peer there is an error

                 /* Reset the rx size */
                 rx_size = 0;
                 continue;
            }
        }
    }

    vdbg("done: ret=%d\n", ret);
    return (void *)ret;
}

static int mhb_send(struct device *dev, struct mhb_hdr *hdr,
    uint8_t *payload, size_t payload_length, int flags) {
    struct mhb_queue_item *item;

    /* Calculate the packet length */
    const size_t length = MHB_HDR_SIZE + payload_length + MHB_CRC_SIZE;

    /* Populate the packet length field */
    hdr->length = cpu_to_le16(length);

    /* Allocate an item */
    // TODO: Use buffer-pool.
    uint8_t *buf = kmm_zalloc(sizeof(*item) + CONFIG_MHB_UART_TXBUFSIZE);
    if (!buf) {
        return -ENOMEM;
    }

    /* Populate the queue item */
    item = (struct mhb_queue_item *)buf;
    item->size = length;
    item->flags = flags;

    /* Copy the packet */
    uint8_t *p = item->buffer;

    /* Copy the header */
    memcpy(p, hdr, MHB_HDR_SIZE);
    p += MHB_HDR_SIZE;

    /* Copy the payload */
    memcpy(p, payload, payload_length);
    p += payload_length;

    /* Calculate the CRC */
    uint16_t crc =
        crc16_poly8005(item->buffer, p - item->buffer, CRC_INIT_VAL);
    crc = cpu_to_le16(crc);
    memcpy(p, &crc, MHB_CRC_SIZE);

    /* Queue the buffer */
    irqstate_t iflags = irqsave();
    sq_addlast(&item->entry, &g_mhb->tx_queue);
    irqrestore(iflags);

    /* Signal the transmit thread that a buffer is available */
    sem_post(&g_mhb->tx_sem);

    return 0;
}

static void mhb_tx_flush(void)
{
    irqstate_t flags = irqsave();

    while (!sq_empty(&g_mhb->tx_queue)) {
        struct mhb_queue_item *item =
                        (struct mhb_queue_item *)sq_remfirst(&g_mhb->tx_queue);
        kmm_free(item);
    }

    irqrestore(flags);
}

static void *mhb_tx_thread(void *data)
{
    int ret = 0;

#if CONFIG_MHB_UART_SEND_SYNC
    ret = _mhb_send_sync_pattern();
#endif

    while (!ret) {
        /* Block until a buffer is available */
        ret = sem_wait(&g_mhb->tx_sem);
        if (ret < 0) {
            ret = -get_errno();
            if (ret != -EINTR) {
                lldbg("ERROR: sem wait failed: %d\n", ret);
            }
            break;
        }

        /* Get the next buffer */
        irqstate_t flags = irqsave();
        bool empty = sq_empty(&g_mhb->tx_queue);
        irqrestore(flags);

        if (empty) {
            continue;
        }

        mhb_wake_peer();

        struct mhb_queue_item *item =
                        (struct mhb_queue_item *)sq_remfirst(&g_mhb->tx_queue);

        /* Send the buffer */
        int sent = 0;
        ret = device_uart_start_transmitter(g_mhb->dev, item->buffer,
                      item->size, NULL /* dma */, &sent,
                      NULL /* callback, blocking without callback */);
        if (ret) {
            lldbg("ERROR: failed to write: %d\n", ret);
            kmm_free(item);
            break;
        }

        /* Switch baud rates if requested. */
        if (g_mhb->current_baud != g_mhb->new_baud) {
            mhb_complete_baud_change(g_mhb->new_baud);
            g_mhb->current_baud = g_mhb->new_baud;
        }

        /* Free the item and buffer */
        kmm_free(item);
    }

    mhb_tx_flush();

    vdbg("done: ret=%d\n", ret);
    return (void *)ret;
}

static int mhb_stop(void)
{
    pthread_addr_t join_value;

    if (g_mhb->rx_thread) {
        pthread_kill(g_mhb->rx_thread, MHB_SIG_STOP);
        pthread_join(g_mhb->rx_thread, &join_value);
        g_mhb->rx_thread = 0;
    }

    if (g_mhb->tx_thread) {
        pthread_kill(g_mhb->tx_thread, MHB_SIG_STOP);
        pthread_join(g_mhb->tx_thread, &join_value);
        g_mhb->tx_thread = 0;
    }

    return 0;
}

static int mhb_start(void)
{
    int ret;

    ret = pthread_create(&g_mhb->rx_thread, NULL, mhb_rx_thread, NULL);
    if (ret) {
        lldbg("ERROR: Failed to create rx thread: %s.\n", strerror(errno));
        g_mhb->rx_thread = 0;
        goto rx_thread_error;
    }

    ret = pthread_create(&g_mhb->tx_thread, NULL, mhb_tx_thread, NULL);
    if (ret) {
        lldbg("ERROR: Failed to create tx thread: %s.\n", strerror(errno));
        g_mhb->tx_thread = 0;
        goto tx_thread_error;
    }

rx_thread_error:
done:
    return ret;

tx_thread_error:
    mhb_stop();
    goto done;
}

static int mhb_restart(struct device *dev)
{
    int ret;

    if (!dev) {
        return -ENODEV;
    }

    ret = mhb_stop();
    if (!ret) {
        ret = mhb_start();
    }

    return ret;
}

static int mhb_dev_open(struct device *dev)
{
    int ret;

    irqstate_t flags;
    flags = irqsave();

    atomic_inc(&g_mhb->ref_count);

    if (g_mhb->dev) {
        ret = 0;
        goto err_irqrestore;
    }

    g_mhb->dev = device_open(DEVICE_TYPE_UART_HW, g_mhb->uart_dev_id);
    if (!g_mhb->dev) {
        lldbg("ERROR: Failed to open.\n");
        ret = -ENODEV;
        goto err_irqrestore;
    }

#if CONFIG_MHB_UART_FLOWCONTROL
    const int flow = 1;
#else
    const int flow = 0;
#endif

    ret = device_uart_set_configuration(g_mhb->dev, CONFIG_MHB_UART_BAUD,
                                        NO_PARITY, 8 /* bits */, ONE_STOP_BIT,
                                        flow /* no flow-control */);
    if (ret) {
        lldbg("ERROR: Failed to configure: %d.\n", ret);
        goto config_error;
    }

    ret = mhb_start();
    if (ret) {
        goto start_error;
    }

err_irqrestore:
    irqrestore(flags);

    return ret;

start_error:
config_error:
    device_close(g_mhb->dev);
    g_mhb->dev = NULL;

    atomic_dec(&g_mhb->ref_count);

    goto err_irqrestore;
}

static void mhb_dev_close(struct device *dev) {
    irqstate_t flags;
    flags = irqsave();

    uint32_t value = atomic_dec(&g_mhb->ref_count);

    if (value == 0) {
        mhb_stop();
        device_close(g_mhb->dev);
        g_mhb->dev = NULL;
    }

    irqrestore(flags);
}

int mhb_dev_probe(struct device *dev)
{
    irqstate_t flags;
    flags = irqsave();

    if (g_mhb) {
        irqrestore(flags);
        return 0;
    }

    g_mhb = kmm_zalloc(sizeof(*g_mhb));
    if (!g_mhb) {
        lldbg("ERROR: Failed to allocate UART\n");
        irqrestore(flags);
        return -ENOMEM;
    }

    atomic_init(&g_mhb->ref_count, 0);

    struct device_resource *uart_dev_id = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "uart_dev_id");
    if (uart_dev_id) {
        g_mhb->uart_dev_id = uart_dev_id->start;
    } else {
        g_mhb->uart_dev_id = 0;
        lldbg("WARNING: Failed to get UART dev id\n");
    }

    struct device_resource *local_wake = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "local_wake");
    if (local_wake) {
        g_mhb->local_wake = local_wake->start;

        gpio_direction_in(local_wake->start);
        set_gpio_triggering(local_wake->start, IRQ_TYPE_EDGE_FALLING);
        gpio_irqattach(local_wake->start, mhb_local_wake_irq);
        gpio_clear_interrupt(local_wake->start);
        gpio_unmask_irq(local_wake->start);
    } else {
        g_mhb->local_wake = MHB_INVALID_GPIO;
        lldbg("WARNING: Failed to get local wake\n");
    }

    struct device_resource *peer_wake = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "peer_wake");
    if (peer_wake) {
        g_mhb->peer_wake = peer_wake->start;

        gpio_direction_out(peer_wake->start, 1);
    } else {
        g_mhb->peer_wake = MHB_INVALID_GPIO;
        lldbg("WARNING: Failed to get peer wake\n");
    }

    g_mhb->self = dev;

    sem_init(&g_mhb->tx_sem, 0, 0);
    sq_init(&g_mhb->tx_queue);

    pthread_mutex_init(&g_mhb->mutex, NULL);
    pthread_cond_init(&g_mhb->cond, NULL);

    atomic_init(&g_mhb->pm_state_peer, 0);

    irqrestore(flags);

    return 0;
}

static void mhb_dev_remove(struct device *dev) {
    irqstate_t flags;
    flags = irqsave();

    kmm_free(g_mhb);
    g_mhb = NULL;

    irqrestore(flags);
}

const static struct device_mhb_type_ops mhb_type_ops = {
    .register_receiver = mhb_register_receiver,
    .unregister_receiver = mhb_unregister_receiver,
    .send = mhb_send,
    .restart = mhb_restart,
};

const static struct device_driver_ops mhb_driver_ops = {
    .probe    = mhb_dev_probe,
    .remove   = mhb_dev_remove,
    .open     = mhb_dev_open,
    .close    = mhb_dev_close,
    .type_ops = (struct device_mhb_type_ops *)&mhb_type_ops,
};

struct device_driver mhb_driver = {
    .type = DEVICE_TYPE_MHB,
    .name = "mhb",
    .desc = "mhb",
    .ops  = (struct device_driver_ops *)&mhb_driver_ops,
};
