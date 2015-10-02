/**
 * Copyright (c) 2015 Google Inc.
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
 * @author Mark Greer
 */

#ifndef __INCLUDE_NUTTX_DEVICE_DMA_H
#define __INCLUDE_NUTTX_DEVICE_DMA_H

#include <errno.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define containerof(x, s, f) ((void*) ((char*)(x) - offsetof(s, f)))

#define DEVICE_TYPE_DMA_HW                 "dma"

/* Size of a single transfer in bits */
#define DEVICE_DMA_TRANSFER_SIZE_8          BIT(0)
#define DEVICE_DMA_TRANSFER_SIZE_16         BIT(1)
#define DEVICE_DMA_TRANSFER_SIZE_32         BIT(2)
#define DEVICE_DMA_TRANSFER_SIZE_64         BIT(3)

/* Size of a single burst in bytes */
#define DEVICE_DMA_BURST_LEN_1              BIT(0)
#define DEVICE_DMA_BURST_LEN_2              BIT(1)
#define DEVICE_DMA_BURST_LEN_3              BIT(2)
#define DEVICE_DMA_BURST_LEN_4              BIT(3)
#define DEVICE_DMA_BURST_LEN_5              BIT(4)
#define DEVICE_DMA_BURST_LEN_6              BIT(5)
#define DEVICE_DMA_BURST_LEN_7              BIT(6)
#define DEVICE_DMA_BURST_LEN_8              BIT(7)
#define DEVICE_DMA_BURST_LEN_9              BIT(8)
#define DEVICE_DMA_BURST_LEN_10             BIT(9)
#define DEVICE_DMA_BURST_LEN_11             BIT(10)
#define DEVICE_DMA_BURST_LEN_12             BIT(11)
#define DEVICE_DMA_BURST_LEN_13             BIT(12)
#define DEVICE_DMA_BURST_LEN_14             BIT(13)
#define DEVICE_DMA_BURST_LEN_15             BIT(14)
#define DEVICE_DMA_BURST_LEN_16             BIT(15)

/* In bits */
#define DEVICE_DMA_SWAP_SIZE_NONE           BIT(0)
#define DEVICE_DMA_SWAP_SIZE_16             BIT(1)
#define DEVICE_DMA_SWAP_SIZE_32             BIT(2)
#define DEVICE_DMA_SWAP_SIZE_64             BIT(3)
#define DEVICE_DMA_SWAP_SIZE_128            BIT(4)

/* Options for the specified source and destination devices */
#define DEVICE_DMA_INC_AUTO                 BIT(0)
#define DEVICE_DMA_INC_NOAUTO               BIT(1)

/* Event that the callback should be called for */
#define DEVICE_DMA_CALLBACK_EVENT_START     BIT(0)
#define DEVICE_DMA_CALLBACK_EVENT_COMPLETE  BIT(1)
#define DEVICE_DMA_CALLBACK_EVENT_DEQUEUED  BIT(2)
#define DEVICE_DMA_CALLBACK_EVENT_ERROR     BIT(3)

/* Alignments for the source and destination address */
#define DEVICE_DMA_ALIGNMENT_8              BIT(0)
#define DEVICE_DMA_ALIGNMENT_16             BIT(1)
#define DEVICE_DMA_ALIGNMENT_32             BIT(2)
#define DEVICE_DMA_ALIGNMENT_64             BIT(3)

enum device_dma_error {
    DEVICE_DMA_ERROR_NONE,
    DEVICE_DMA_ERROR_INVALID,
    DEVICE_DMA_ERROR_BAD_DEV,
    DEVICE_DMA_ERROR_BAD_FLAG,
    DEVICE_DMA_ERROR_BAD_TX_SIZE,
    DEVICE_DMA_ERROR_BAD_BURST_LEN,
    DEVICE_DMA_ERROR_BAD_SWAP,
    DEVICE_DMA_ERROR_BAD_ADDR,
    DEVICE_DMA_ERROR_BAD_LEN,
    DEVICE_DMA_ERROR_DMA_FAILED,
    DEVICE_DMA_ERROR_UNKOWN_FAILURE = 0xff,
};

enum device_dma_dev {
    DEVICE_DMA_DEV_INVALID, DEVICE_DMA_DEV_MEM, DEVICE_DMA_DEV_IO,
};

struct device_dma_caps {
    unsigned int addr_alignment;
    unsigned int transfer_sizes; /* DEVICE_DMA_TRANSFER_SIZE_* */
    unsigned int burst_len; /* DEVICE_DMA_BURST_LEN_* */
    unsigned int swap_options; /* DEIVCE_DMA_SWAP_SIZE_* */
    unsigned int inc_options; /* DEVICE_DMA_INC_* */
    unsigned int sg_max;
};

struct device_dma_sg {
    off_t src_addr;
    off_t dst_addr;
    size_t len;
};

struct device_dma_op;

typedef int (*device_dma_op_callback)(struct device *dev, void *chan,
        struct device_dma_op *op, unsigned int event, void *arg);

struct device_dma_op {
    device_dma_op_callback callback;
    void *callback_arg;
    unsigned int callback_events;
    unsigned int sg_count;
    struct device_dma_sg sg[0];
};

struct device_dma_params {
    enum device_dma_dev src_dev;
    unsigned int src_devid;
    unsigned int src_inc_options;
    enum device_dma_dev dst_dev;
    unsigned int dst_devid;
    unsigned int dst_inc_options;
    unsigned int transfer_size;
    unsigned int burst_len;
    unsigned int swap;
};

struct device_dma_type_ops {
    int (*get_caps)(struct device *dev, struct device_dma_caps *caps);
    int (*chan_free_count)(struct device *dev);
    int (*chan_alloc)(struct device *dev, struct device_dma_params *params,
            void **chanp);
    int (*chan_free)(struct device *dev, void *chan);
    int (*op_alloc)(struct device *dev, unsigned int sg_count,
            unsigned int extra, struct device_dma_op **opp);
    int (*op_free)(struct device *dev, struct device_dma_op *op);
    int (*op_is_complete)(struct device *dev, struct device_dma_op *op);
    int (*op_get_error)(struct device *dev, struct device_dma_op *op,
            enum device_dma_error *error);
    int (*enqueue)(struct device *dev, void *chan, struct device_dma_op *op);
    int (*dequeue)(struct device *dev, void *chan, struct device_dma_op *op);
};

/**
 * @brief Get DMA device's capabilites for specified type
 *
 * Implies that the DMA driver has knowledge of the capabilities for
 * each type of DMA type.
 *
 * @param dev DMA device whose capabilities are queried
 * @param type DMA type whose capabilities are queried
 * @param capabilites pointer to capabilities structure that will be filled out
 * @return 0: Query successful
 *         -errno: Cause of failure
 */
static inline int device_dma_get_caps(struct device *dev,
        struct device_dma_caps *caps)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->get_caps)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->get_caps(dev, caps);

    return -ENOSYS;
}

/**
 * @brief Get number of unallocated DMA channels
 * @param dev DMA device whose number of free channels is queried
 * @return 'n': Number of unallocated/free DMA channels
 *         -errno: Cause of failure
 */
static inline int device_dma_chan_free_count(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->chan_free_count)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->chan_free_count(dev);

    return -ENOSYS;
}

/**
 * @brief Allocate a DMA channel
 * @param dev DMA device whose channel is being allocated
 * @param chanp Pointer to DMA channel cookie
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_dma_chan_alloc(struct device *dev,
        struct device_dma_params *params, void **chanp)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->chan_alloc)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->chan_alloc(dev, params, chanp);

    return -ENOSYS;
}

/**
 * @brief Free a DMA channel
 * @param dev DMA device whose channel is being freed
 * @param chan DMA channel cookie
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_dma_chan_free(struct device *dev, void *chan)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->chan_free)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->chan_free(dev, chan);

    return -ENOSYS;
}

/**
 * @brief Allocate a DMA operation structure
 * @param dev DMA device
 * @param opp Pointer to pointer of dma_op structure being allocated
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_dma_op_alloc(struct device *dev, unsigned int sg_count,
        unsigned int extra, struct device_dma_op **opp)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->op_alloc)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->op_alloc(dev, sg_count, extra,
                opp);

    return -ENOSYS;
}

/**
 * @brief Free a DMA operation structure
 * @param dev DMA device
 * @param opp Pointer to dma_op structure being freed
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_dma_op_free(struct device *dev,
        struct device_dma_op *op)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->op_free)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->op_free(dev, op);

    return -ENOSYS;
}

/**
 * @brief Check if DMA operation is complete
 * @param dev DMA device
 * @param op operation being checked
 * @return 0: Operation is not complete
 *         1: Operations is complete
 *         -errno: Cause of failure
 */
static inline int device_dma_op_is_complete(struct device *dev,
        struct device_dma_op *op)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->op_is_complete)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->op_is_complete(dev, op);

    return -ENOSYS;
}

/**
 * @brief Retrieve operation error code
 * @param dev DMA device
 * @param op operation being checked
 * @param error the error code
 * @return 0: Operation successful
 *         -errno: Cause of failure
 */
static inline int device_dma_op_get_error(struct device *dev,
        struct device_dma_op *op, enum device_dma_error *error)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->op_get_error)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->op_get_error(dev, op, error);

    return -ENOSYS;
}

/**
 * @brief Enqueue (and possibly start) DMA operation
 *
 * If the DMA channel is busy, the 'op' will be queued and executed in the
 * order received by the driver.  If caller wishes to block until the DMA
 * is complete, it can block on a semaphore which should be unblocked by
 * the callback routine.
 *
 * @param dev DMA device operation queued is queued for
 * @param chan DMA channel cookie
 * @param op device_dma_op structure describing the entire DMA operation
 * @return 0: Successfully queued
 *         -errno: Cause of failure
 */
static inline int device_dma_enqueue(struct device *dev, void *chan,
        struct device_dma_op *op)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->enqueue)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->enqueue(dev, chan, op);

    return -ENOSYS;
}

/**
 * @brief Dequeue a DMA operation
 * @param dev DMA device whose operation is being dequeued
 * @param chan DMA channel cookie
 * @param op device_dma_op structure being dequeued
 * @return 0: Successfully dequeued
 *         -errno: Cause of failure
 */
static inline int device_dma_dequeue(struct device *dev, void *chan,
        struct device_dma_op *op)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, dma)->dequeue)
        return DEVICE_DRIVER_GET_OPS(dev, dma)->dequeue(dev, chan, op);

    return -ENOSYS;
}

#endif /* __INCLUDE_NUTTX_DEVICE_DMA_H */
