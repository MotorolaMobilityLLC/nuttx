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
#include <nuttx/gpio.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/device_spi.h>

#include "up_arch.h"

#include "chip.h"
#include "tsb_scm.h"

#if defined(CONFIG_TSB_CHIP_REV_ES2)
#include "tsb_spi_es2.h"
#endif

/**
 * SPI device state
 */
enum tsb_spi_state {
    TSB_SPI_STATE_INVALID,
    TSB_SPI_STATE_CLOSED,
    TSB_SPI_STATE_OPEN,
    TSB_SPI_STATE_LOCKED,
};

/**
 * @brief private SPI device information
 */
struct tsb_spi_info {
    /** Driver model representation of the device */
    struct device       *dev;
    /** SPI device base address */
    uint32_t            reg_base;
    /** SPI device state */
    enum tsb_spi_state  state;
    /** bit mask of current SPI protocol mode */
    uint16_t            mode;
    /** gpio index to use as cs instead of hw cs */
    int8_t              cs_gpio[1];

    /** bit masks of supported SPI protocol mode */
    uint16_t            modes;
    /** bit masks of supported SPI protocol flags */
    uint16_t            flags;
    /** number of bits per word supported */
    uint32_t            bpw;
    /** number of chip select pins supported */
    uint16_t            csnum;

    /** Exclusive access for SPI bus */
    sem_t               bus;
    /** Exclusive access for operation */
    sem_t               lock;
};

static uint32_t spi_read(int offset)
{
    return getreg32(SPI_BASE + offset);
}

static void spi_write(int offset, uint32_t b)
{
    putreg32(b, SPI_BASE + offset);
}

/**
 * @brief Lock SPI bus for exclusive access
 *
 * On SPI buses where there are multiple devices, it will be necessary to lock
 * SPI to have exclusive access to the buses for a sequence of transfers.
 * The bus should be locked before the chip is selected. After locking the SPI
 * bus, the caller should then also call the setfrequency(), setbits() , and
 * setmode() methods to make sure that the SPI is properly configured for the
 * device. If the SPI buses is being shared, then it may have been left in an
 * incompatible state.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_lock(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    /* Take the semaphore (perhaps waiting) */
    ret = sem_wait(&info->bus);
    if (ret != OK) {
        /* The sem_wait() call should fail only if we are awakened by
         * a signal.
         */
        return -get_errno();
    }
    info->state = TSB_SPI_STATE_LOCKED;

    return 0;
}

/**
 * @brief unlock SPI bus for exclusive access
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_unlock(struct device *dev)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    info->state = TSB_SPI_STATE_OPEN;
    sem_post(&info->bus);
    return 0;
}

/**
 * @brief Enable the SPI chip select pin
 *
 * The implementation of this method must include handshaking. If a device is
 * selected, it must hold off all the other attempts to select the device
 * until the device is deselected. This function should be called after lock(),
 * if the driver isn’t in lock state, it returns an error code to notify a
 * problem.
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_select(struct device *dev, int devid)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    if (devid >= ARRAY_SIZE(info->cs_gpio)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        sem_post(&info->lock);
        return -EPERM;
    }

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    if (info->cs_gpio[devid] != -1) {
        uint8_t cs_active = info->mode & SPI_MODE_CS_HIGH ? 1 : 0;
        gpio_direction_out(info->cs_gpio[devid], cs_active);
    }
#else
    /* TODO: Implement chip-select code.
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
#endif
    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Disable the SPI chip select pin
 *
 * The implementation of this method must include handshaking. If a device is
 * selected, it must hold off all the other attempts to select the device
 * until the device is deselected. This function should be called after lock(),
 * if the driver isn’t in lock state, it returns an error code to notify a
 * problem.
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_deselect(struct device *dev, int devid)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    if (devid >= ARRAY_SIZE(info->cs_gpio)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        sem_post(&info->lock);
        return -EPERM;
    }

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    if (info->cs_gpio[devid] != -1) {
        uint8_t cs_active = info->mode & SPI_MODE_CS_HIGH ? 1 : 0;
        gpio_direction_out(info->cs_gpio[devid], !cs_active);
    }
#else
    /* TODO: Implement chip-select code.
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
#endif
    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Configure SPI clock.
 *
 * If SPI hardware doesn’t support this frequency value, this function should
 * find the nearest lower frequency in which hardware supported and then
 * configure SPI clock to this value. It will return the actual frequency
 * selected value back to the caller via parameter frequency.
 * This function should be called after lock(), if the driver is not in lock
 * state, it returns an error code to notify a problem.
 *
 * @param dev pointer to structure of device data
 * @param frequency SPI frequency requested (unit: Hz)
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setfrequency(struct device *dev, uint32_t *frequency)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !frequency) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        sem_post(&info->lock);
        return -EPERM;
    }
#if defined(CONFIG_TSB_CHIP_REV_ES2)
    uint32_t v;
    if (*frequency < 12*1000*1000) {
        *frequency = 6*1000*1000;
        v = 0x00;
    } else if (*frequency < 24*1000*1000) {
        *frequency = 12*1000*1000;
        v = 0x01;
    } else {
        *frequency = 24*1000*1000;
        v = 0x10;
    }

    tsb_set_spi_clock(v);
#else
    /* TODO: Change SPI hardware clock
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
#endif
    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Configure SPI mode.
 *
 * To configure SPI configuration such as clock polarity and phase via the mode
 * parameter. Other possible definition of SPI mode can be found in SPI mode
 * definition. If the value of mode parameter is out of SPI mode definition or
 * this mode isn’t supported by the current hardware, this function should
 * return -EOPNOTSUPP error code.
 * This function should be called after lock(), if driver is not in lock state,
 * function returns -EPERM error code.
 *
 * @param dev pointer to structure of device data
 * @param mode SPI protocol mode requested
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setmode(struct device *dev, uint16_t mode)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        sem_post(&info->lock);
        return -EPERM;
    }

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    uint16_t allowed = SPI_MODE_0;
#if defined(CONFIG_ARCH_CHIP_DEVICE_SPI_CS0_GPIO)
    allowed |= SPI_MODE_CS_HIGH;
#endif
    if (mode & ~allowed) {
        return -EINVAL;
    }

    info->mode = mode;
#else
    /* TODO: change SPI mode register
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
#endif
    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Set the number of bits per word in transmission.
 *
 * This function should be called after lock(), if driver is not in lock state,
 * this function returns -EPERM error code.
 *
 * @param dev pointer to structure of device data
 * @param nbits The number of bits requested. The nbits value range is from
 *        1 to 32. The generic nbits value is 8, 16, 32, but this value still
 *        depends on hardware supported.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setbits(struct device *dev, int nbits)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        sem_post(&info->lock);
        return -EPERM;
    }
    /* TODO: Implement setbits function
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
    sem_post(&info->lock);
    return 0;
}

#if defined(CONFIG_TSB_CHIP_REV_ES2)
/**
 * @brief Internal exchange function using PIO and the secondary buffer
 *
 * Perform an exchange using programmed I/O (PIO) using only the secondary
 * buffer.  Block until the exchange is complete.
 *
 * @param dev pointer to structure of device data
 * @param txbuffer pointer to the data to send
 * @param rxbuffer pointer to the buffer to receive data into
 * @param nbytes number of bytes to exchange, up to TSB_SPIB0_PRG_SEC_DAT_SIZE.
 * @return 0 on success, negative errno on error
 */
static int __tsb_spi_exchange_single(struct device *dev, void *txbuffer,
                                          void *rxbuffer, int nbytes)
{
    void *sec_buf = (void *)(SPI_BASE + TSB_SPIB0_PRG_SEC_DAT_BASE);

    /* Clear the status bits */
    spi_write(TSB_SPIB_PRG_ACC_STAT, 0);

    /* Set up the secondary buffer */
    memcpy(sec_buf, txbuffer, nbytes);

    /* Subtract 1 since it is assumed */
    uint32_t v = (nbytes - 1) & TSB_SEC_BUF_DAT_BYTE_CNT_MASK;
    v <<= TSB_SEC_BUF_DAT_BYTE_CNT_SHIFT;
    v |= TSB_SEC_BUF_EN | TSB_SPI_CYC_GO;
    spi_write(TSB_SPIB_PRG_ACC_CTRL_1, v);

    /* Wait for the transaction to complete. */
    while (1) {
        v = spi_read(TSB_SPIB_PRG_ACC_STAT);
        if (v & TSB_SPI_CYC_DONE) {
            break;
        }
    }

    /* Extract the secondary buffer */
    if (rxbuffer) {
        memcpy(rxbuffer, sec_buf, nbytes);
    }

    return 0;
}

/**
 * @brief Internal exchange function using PIO and both the primary and
 * secondary buffers
 *
 * Perform an exchange using programmed I/O (PIO) using both the primary
 * and secondary buffers.  Block until the exchange is complete.
 * This is useful when using HW CS and a single transfer is slightly larger
 * than the secondary buffer (e.g. a 256-byte page write).
 *
 * @param dev pointer to structure of device data
 * @param txbuffer pointer to the data to send
 * @param rxbuffer pointer to the buffer to receive data into
 * @param nbytes number of bytes to exchange, up to TSB_SPIB0_PRG_PRI_DAT_SIZE +
 * TSB_SPIB0_PRG_SEC_DAT_SIZE.
 * @return 0 on success, negative errno on error
 */
static int __tsb_spi_exchange_dual(struct device *dev, void *txbuffer,
                                          void *rxbuffer, int nbytes)
{
    void *pri_buf = (void *)(SPI_BASE + TSB_SPIB0_PRG_PRI_DAT_BASE);
    void *sec_buf = (void *)(SPI_BASE + TSB_SPIB0_PRG_SEC_DAT_BASE);

    /* Clear the status bits */
    spi_write(TSB_SPIB_PRG_ACC_STAT, 0);

    uint32_t v = TSB_SPI_CYC_GO;

    /* Set up the primary buffer */
    ssize_t pri_size = MIN(nbytes, TSB_SPIB0_PRG_PRI_DAT_SIZE);
    memcpy(pri_buf, txbuffer, pri_size);

    /* Subtract 1 since it is assumed */
    v |= ((pri_size - 1) & TSB_PRI_BUF_DAT_BYTE_CNT_MASK)
           << TSB_PRI_BUF_DAT_BYTE_CNT_SHIFT;
    v |= TSB_PRI_BUF_EN;

    /* Set up the secondary buffer */
    ssize_t sec_size = nbytes - pri_size;
    if (sec_size > 0) {
        memcpy(sec_buf, txbuffer + pri_size, sec_size);

        /* Subtract 1 since it is assumed */
        v |= ((sec_size - 1) & TSB_SEC_BUF_DAT_BYTE_CNT_MASK)
              << TSB_SEC_BUF_DAT_BYTE_CNT_SHIFT;
        v |= TSB_SEC_BUF_EN;
    }

    spi_write(TSB_SPIB_PRG_ACC_CTRL_1, v);

    /* Wait for the transaction to complete. */
    while (1) {
        v = spi_read(TSB_SPIB_PRG_ACC_STAT);
        if (v & TSB_SPI_CYC_DONE) {
            break;
        }
    }

    /* Extract the primary buffer */
    if (rxbuffer) {
        memcpy(rxbuffer, pri_buf, pri_size);

        /* Extract the secondary buffer */
        if (sec_size > 0) {
            memcpy(rxbuffer + pri_size, sec_buf, sec_size);
        }
    }

    return 0;
}
#endif

/**
 * @brief Exchange a block of data from SPI
 *
 * Device driver uses this function to transfer and receive data from SPI bus.
 * This function should be called after lock() , if the driver is not in lock
 * state, it returns -EPERM error code.
 * The transfer structure is consists of the read/write buffer,transfer length,
 * transfer flags and callback function.
 *
 * @param dev pointer to structure of device data
 * @param transfer pointer to the spi transfer request
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_exchange(struct device *dev,
                             struct device_spi_transfer *transfer)
{
    struct tsb_spi_info *info = NULL;
    int i = 0, ret = 0;
    uint8_t *txbuf = NULL, *rxbuf = NULL;


    /* check input parameters */
    if (!dev || !device_get_private(dev) || !transfer) {
        return -EINVAL;
    }

    /* check transfer buffer */
    if(!transfer->txbuffer && !transfer->rxbuffer) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_unlock;
    }

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    txbuf = transfer->txbuffer;
    rxbuf = transfer->rxbuffer;

    if (transfer->nwords > TSB_SPIB0_PRG_SEC_DAT_SIZE &&
         transfer->nwords <= TSB_SPIB0_PRG_PRI_DAT_SIZE +
                              TSB_SPIB0_PRG_SEC_DAT_SIZE) {
        /* If the transfer is bigger than the secondary buffer, but can fit
           in both the primary and secondary buffers, then use them both. */
        __tsb_spi_exchange_dual(dev, txbuf, rxbuf, transfer->nwords);
    } else {
        /* If the transfer is bigger than the primary and secondary buffers,
           then do multiple transfers using the secondary buffer.
           NOTE: Each iteration will toggle the HW CS.  Use GPIO CS if CS
           has to be active for the entire transfer. */
        int num_iterations = transfer->nwords / TSB_SPIB0_PRG_SEC_DAT_SIZE;
        for (i=0; i < num_iterations; i++) {
            __tsb_spi_exchange_single(dev, txbuf, rxbuf,
                                          TSB_SPIB0_PRG_SEC_DAT_SIZE);

            txbuf += TSB_SPIB0_PRG_SEC_DAT_SIZE;
            rxbuf += TSB_SPIB0_PRG_SEC_DAT_SIZE;
        }

        int remainder = transfer->nwords % TSB_SPIB0_PRG_SEC_DAT_SIZE;
        if (remainder) {
            __tsb_spi_exchange_single(dev, txbuf, rxbuf, remainder);
        }
    }
#else
    /* TODO: Implement SPI transfer function
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */

    /* for test only */
    txbuf = transfer->txbuffer;
    rxbuf = transfer->rxbuffer;
    for (i=0; i < transfer->nwords; i++) {
        if (txbuf && rxbuf) {
            rxbuf[i] = ~txbuf[i];
        } else if (rxbuf) {
            rxbuf[i] = (uint8_t)i;
        }
    }
#endif
err_unlock:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief SPI interrupt handler
 *
 * @param irq interrupt number
 * @param context argument for interrupt handler
 * @return 0 if successful, negative error code otherise.
 */
static int tsb_spi_irq_handler(int irq, void *context)
{
    /* TODO: Implement IRQ handler code.
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
    return 0;
}

/**
 * @brief Get SPI device driver hardware capabilities information.
 *
 * This function can be called whether lock() has been called or not.
 *
 * @param dev pointer to structure of device data
 * @param caps pointer to the spi_caps structure to receive the capabilities
 *             information.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_getcaps(struct device *dev, struct device_spi_caps *caps)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !caps) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    caps->modes = SPI_MODE_0;
#if defined(CONFIG_ARCH_CHIP_DEVICE_SPI_CS0_GPIO)
    caps->modes |= SPI_MODE_CS_HIGH;
#endif
    caps->flags = 0;
    caps->bpw = BIT(8-1);
    caps->csnum = 1;
#else
    /* TODO: Add query hardware capabilities code.
     *
     * Because SPI master only supported on Toshiba ES3 chip, the hardware
     * isn't ready, so we only add dummy code for testing.
     */
    caps->modes = SPI_MODE_CPHA |
                  SPI_MODE_CPOL |
                  SPI_MODE_CS_HIGH |
                  SPI_MODE_LSB_FIRST |
                  SPI_MODE_LOOP;
    caps->flags = 0;
    caps->bpw = BIT(8-1) | BIT(16-1) | BIT(32-1);
    caps->csnum = 1;
#endif

    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Open SPI device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_dev_open(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    int ret = 0;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    tsb_clk_enable(TSB_CLK_SPIP);
    tsb_clk_enable(TSB_CLK_SPIS);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_CLOSED) {
        ret = -EBUSY;
        goto err_unlock;
    }
    info->state = TSB_SPI_STATE_OPEN;

err_unlock:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Close SPI device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function. If the device
 * is not opened yet, this function should return without any operations.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_close(struct device *dev)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    tsb_clk_disable(TSB_CLK_SPIP);
    tsb_clk_disable(TSB_CLK_SPIS);

    sem_wait(&info->lock);
    info->state = TSB_SPI_STATE_CLOSED;
    sem_post(&info->lock);
}

/**
 * @brief Probe SPI device
 *
 * This function is called by the system to register the driver when the system
 * boot up. This function allocates memory for the private SPI device
 * information, and then setup the hardware resource and interrupt handler.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_dev_probe(struct device *dev)
{
    struct tsb_spi_info *info;
    struct device_resource *r;
    irqstate_t flags;
    int ret = 0;
    int i;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }
    /* get register data from resource block */
    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS, "reg_base");
    if (!r) {
        ret = -EINVAL;
        goto err_freemem;
    }

    flags = irqsave();
    /* register SPI IRQ number */
    ret = irq_attach(TSB_IRQ_SPI, tsb_spi_irq_handler);
    if (ret != OK) {
        ret = -EIO;
        goto err_irqrestore;
    }
#if 0
    // only ES3 chip supported SPI Master, disable interrupt now.
    up_enable_irq(TSB_IRQ_SPI);
#endif

    info->dev = dev;
    info->reg_base = r->start;
    info->state = TSB_SPI_STATE_CLOSED;
    info->mode = 0;
    for (i=0; i < ARRAY_SIZE(info->cs_gpio); i++) {
        info->cs_gpio[i] = -1;
    }
#if defined(CONFIG_TSB_CHIP_REV_ES2)
#if defined(CONFIG_ARCH_CHIP_DEVICE_SPI_CS0_GPIO)
    tsb_set_pinshare(TSB_PIN_GPIO15);
    gpio_direction_out(15, 1);
    info->cs_gpio[0] = 15;
#else
    tsb_clr_pinshare(TSB_PIN_GPIO15);
#endif
#endif
    device_set_private(dev, info);
    sem_init(&info->bus, 0, 1);
    sem_init(&info->lock, 0, 1);
    irqrestore(flags);
    return 0;

err_irqrestore:
    irqrestore(flags);
err_freemem:
    free(info);
    return ret;
}

/**
 * @brief Remove SPI device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_remove(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    irqstate_t flags;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    flags = irqsave();
    irq_detach(TSB_IRQ_SPI);
    info->state = TSB_SPI_STATE_INVALID;
    sem_destroy(&info->lock);
    sem_destroy(&info->bus);
    device_set_private(dev, NULL);
    irqrestore(flags);
    free(info);
}

static struct device_spi_type_ops tsb_spi_type_ops = {
    .lock           = tsb_spi_lock,
    .unlock         = tsb_spi_unlock,
    .select         = tsb_spi_select,
    .deselect       = tsb_spi_deselect,
    .setfrequency   = tsb_spi_setfrequency,
    .setmode        = tsb_spi_setmode,
    .setbits        = tsb_spi_setbits,
    .exchange       = tsb_spi_exchange,
    .getcaps        = tsb_spi_getcaps,
};

static struct device_driver_ops tsb_spi_driver_ops = {
    .probe          = tsb_spi_dev_probe,
    .remove         = tsb_spi_dev_remove,
    .open           = tsb_spi_dev_open,
    .close          = tsb_spi_dev_close,
    .type_ops       = &tsb_spi_type_ops,
};

struct device_driver tsb_spi_driver = {
    .type       = DEVICE_TYPE_SPI_HW,
    .name       = "tsb_spi",
    .desc       = "TSB SPI Driver",
    .ops        = &tsb_spi_driver_ops,
};
