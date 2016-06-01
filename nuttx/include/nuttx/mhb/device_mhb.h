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

#if !defined(mhb_h)
#define mhb_h

#define DEVICE_TYPE_MHB "MHB"

#include <nuttx/mhb/mhb_protocol.h>

#define MHB_SEND_FLAGS_PEER_ASLEEP (0x1000)
#define MHB_SEND_FLAGS_LOCAL_SLEEP (0x2000)
#define MHB_SEND_FLAGS_WAKE_ACK    (0x4000)

typedef int (*mhb_receiver)(struct device *dev,
    struct mhb_hdr *hdr, uint8_t *payload, size_t payload_length);

struct device_mhb_type_ops {
    int (*register_receiver)(struct device *dev,
        uint8_t addr, mhb_receiver receiver);
    int (*unregister_receiver)(struct device *dev,
        uint8_t addr, mhb_receiver receiver);
    int (*send)(struct device *dev,
        struct mhb_hdr *hdr, uint8_t *payload, size_t payload_length, int flags);
    int (*restart)(struct device *dev);
};

/**
 * @brief Register callback
 *
 * Register a callback from the MHB transport using the specified address.
 * Callbacks are executed in the MHB's receiver thread.  The header and
 * payload are 'owned' by the MHB transport.  If they are needed after
 * the listener returns from the callback, then they need to be copied.
 *
 * @param dev MHB device
 * @param addr MHB_ADDR_* value to register for callbacks from.
 * @param receiver Callback function to register
 * @return 0 on success, negative errno on error
 */
static inline int device_mhb_register_receiver(struct device *dev,
    uint8_t addr, mhb_receiver receiver)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, mhb)->register_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, mhb)->register_receiver(dev,
            addr, receiver);

    return -ENOSYS;
}

/**
 * @brief Unregister callback
 *
 * Unregister a previously registered callback using the specified address.
 *
 * @param dev MHB device
 * @param addr MHB_ADDR_* value to register for callbacks from.
 * @param receiver Callback function previously registered.
 * @return 0 on success, negative errno on error
 */
static inline int device_mhb_unregister_receiver(struct device *dev,
    uint8_t addr, mhb_receiver receiver)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, mhb)->unregister_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, mhb)->unregister_receiver(dev,
            addr, receiver);

    return -ENOSYS;
}

/**
 * @brief Send
 *
 * Send a MHB message using the MHB transport.
 *
 * @param dev MHB device
 * @param hdr MHB message header.
 * @param payload Pointer to the payload or NULL if no payload.
 * @param payload_length Length, in bytes, of the payload or 0 if no payload.
 * @param flags Reserved, must be 0.
 * @return 0 on success, negative errno on error
 */
static inline int device_mhb_send(struct device *dev, struct mhb_hdr *hdr,
    const uint8_t *payload, size_t payload_length, int flags)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, mhb)->send)
        return DEVICE_DRIVER_GET_OPS(dev, mhb)->send(dev, hdr,
            payload, payload_length, flags);

    return -ENOSYS;
}

/**
 * @brief Restart
 *
 * Restart the RX and TX threads.  Send or wait for a sync if configured.
 *
 * @param dev MHB device
 * @return 0 on success, negative errno on error
 */
static inline int device_mhb_restart(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, mhb)->restart)
        return DEVICE_DRIVER_GET_OPS(dev, mhb)->restart(dev);

    return -ENOSYS;
}
#endif
