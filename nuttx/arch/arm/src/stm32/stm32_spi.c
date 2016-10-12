/************************************************************************************
 * arm/arm/src/stm32/stm32_spi.c
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 * provided by board-specific logic.  They are implementations of the select
 * and status methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi/spi.h). All other methods (including up_spiinitialize())
 * are provided by common STM32 logic.  To use this common SPI logic on your
 * board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************c********************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_dma.h"
#include "stm32_spi.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3) || \
    defined(CONFIG_STM32_SPI4) || defined(CONFIG_STM32_SPI5) || defined(CONFIG_STM32_SPI6)

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* SPI interrupts */

#ifdef CONFIG_STM32_SPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_STM32_SPI_INTERRUPTS) && defined(CONFIG_STM32_SPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif

/* SPI DMA priority */

#ifdef CONFIG_STM32_SPI_DMA

#  if defined(CONFIG_SPI_DMAPRIO)
#    define SPI_DMA_PRIO  CONFIG_SPI_DMAPRIO
#  elif defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32L15XX) || \
        defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)
#    define SPI_DMA_PRIO  DMA_CCR_PRIMED
#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#    define SPI_DMA_PRIO  DMA_SCR_PRIMED
#  else
#    error "Unknown STM32 DMA"
#  endif

#  if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32L15XX) || \
      defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)
#    if (SPI_DMA_PRIO & ~DMA_CCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_SPI_DMAPRIO"
#    endif
#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#    if (SPI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_SPI_DMAPRIO"
#    endif
#  else
#    error "Unknown STM32 DMA"
#  endif

#endif

/* DMA channel configuration */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32L4X6) || \
    defined(CONFIG_STM32_STM32L4X3)
#  define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_CCR_MSIZE_16BITS|DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC            )
#  define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS |DMA_CCR_MINC            )
#  define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_16BITS                         )
#  define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS                          )
#  define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_CCR_MSIZE_16BITS|DMA_CCR_PSIZE_16BITS|DMA_CCR_MINC|DMA_CCR_DIR)
#  define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS |DMA_CCR_MINC|DMA_CCR_DIR)
#  define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_16BITS             |DMA_CCR_DIR)
#  define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_CCR_MSIZE_8BITS |DMA_CCR_PSIZE_8BITS              |DMA_CCR_DIR)
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define SPI_RXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_P2M)
#  define SPI_RXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_P2M)
#  define SPI_RXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_P2M)
#  define SPI_TXDMA16_CONFIG        (SPI_DMA_PRIO|DMA_SCR_MSIZE_16BITS|DMA_SCR_PSIZE_16BITS|DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8_CONFIG         (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS |DMA_SCR_MINC|DMA_SCR_DIR_M2P)
#  define SPI_TXDMA16NULL_CONFIG    (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_16BITS             |DMA_SCR_DIR_M2P)
#  define SPI_TXDMA8NULL_CONFIG     (SPI_DMA_PRIO|DMA_SCR_MSIZE_8BITS |DMA_SCR_PSIZE_8BITS              |DMA_SCR_DIR_M2P)
#else
#  error "Unknown STM32 DMA"
#endif

#define CALL_CB_IF_SET(priv, func)           \
    if (priv->cb_ops && priv->cb_ops->func)  \
      {                                      \
        priv->cb_ops->func(priv->cb_v);      \
      }

/* The number of error prints before sleeping the prints. A success will reset
 * the count.
 */
#define ERR_CNT_ZZZ 5

/* Debug ****************************************************************************/
/* Check if (non-standard) SPI debug is enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct stm32_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         spibase;    /* SPIn base address */
  uint32_t         spiclock;   /* Clocking for the SPI module */
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  uint8_t          spiirq;     /* SPI IRQ number */
#endif
#ifdef CONFIG_STM32_SPI_DMA
  volatile uint8_t rxresult;   /* Result of the RX DMA */
  volatile uint8_t txresult;   /* Result of the RX DMA */
  uint16_t         rxch;       /* The RX DMA channel number */
  uint16_t         txch;       /* The TX DMA channel number */
  DMA_HANDLE       rxdma;      /* DMA channel handle for RX transfers */
  DMA_HANDLE       txdma;      /* DMA channel handle for TX transfers */
  sem_t            rxsem;      /* Wait for RX DMA to complete */
  sem_t            txsem;      /* Wait for TX DMA to complete */
  uint32_t         txccr;      /* DMA control register for TX transfers */
  uint32_t         rxccr;      /* DMA control register for RX transfers */
#endif
#ifndef CONFIG_SPI_OWNBUS
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  int8_t           nbits;      /* Width of word in bits (8 or 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
#endif
  uint8_t          mode_type;  /* Master or Slave */
#ifdef CONFIG_SPI_SLAVE
  uint8_t          xfering;    /* Flag indicating transfer in progress */
  const struct spi_cb_ops_s *cb_ops; /* spi dev callbacks */
  void            *cb_v;       /* Data pointer for spi dev callbacks */
  uint32_t         err_cnt;    /* Count of errors seen since last success */
#endif
#ifdef CONFIG_SPI_CRC16
  uint8_t          crc16_en;   /* CRC16 enabled */
#endif
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static inline uint16_t spi_getreg(FAR struct stm32_spidev_s *priv, uint8_t offset);
static inline void spi_putreg(FAR struct stm32_spidev_s *priv, uint8_t offset,
                                 uint16_t value);
static inline uint16_t spi_readword(FAR struct stm32_spidev_s *priv);
static inline void spi_writeword(FAR struct stm32_spidev_s *priv, uint16_t byte);
static inline bool spi_16bitmode(FAR struct stm32_spidev_s *priv);
static void spi_modifycr1(FAR struct stm32_spidev_s *priv, uint16_t setbits, uint16_t clrbits);

/* DMA support */

#ifdef CONFIG_STM32_SPI_DMA
static void        spi_dmarxwait(FAR struct stm32_spidev_s *priv);
static void        spi_dmatxwait(FAR struct stm32_spidev_s *priv);
static inline void spi_dmarxwakeup(FAR struct stm32_spidev_s *priv);
static inline void spi_dmatxwakeup(FAR struct stm32_spidev_s *priv);
static void        spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmarxsetup(FAR struct stm32_spidev_s *priv,
                                  FAR void *rxbuffer, FAR void *rxdummy, size_t nwords);
static void        spi_dmatxsetup(FAR struct stm32_spidev_s *priv,
                                  FAR const void *txbuffer, FAR const void *txdummy, size_t nwords);
static inline void spi_dmarxstart(FAR struct stm32_spidev_s *priv);
static inline void spi_dmatxstart(FAR struct stm32_spidev_s *priv);
#ifdef CONFIG_SPI_SLAVE
static void spi_rxtxdmastop_slave(FAR struct spi_dev_s *dev);
static void spi_portreset(FAR struct stm32_spidev_s *priv);
#endif
#endif

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int         spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static uint32_t    spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void        spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void        spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t    spi_send(FAR struct spi_dev_s *dev, uint16_t wd);
static void        spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                size_t nwords);
static void        spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                 size_t nwords);
#endif

#ifdef CONFIG_SPI_SLAVE
static int spi_registercallback(FAR struct spi_dev_s *dev,
                                const struct spi_cb_ops_s *cb_ops, void *v);
#endif

/* Initialization */

static void        spi_portinitialize(FAR struct stm32_spidev_s *priv);

#ifdef CONFIG_SPI_SLAVE
static int stm32_spi_ss_isr(struct stm32_spidev_s *priv, int nss_gpio);

#ifdef CONFIG_STM32_SPI1
static int stm32_spi1_ss_isr(int irq, void *context);
#endif
#ifdef CONFIG_STM32_SPI2
static int stm32_spi2_ss_isr(int irq, void *context);
#endif
#ifdef CONFIG_STM32_SPI3
static int stm32_spi3_ss_isr(int irq, void *context);
#endif
#ifdef CONFIG_STM32_SPI4
static int stm32_spi4_ss_isr(int irq, void *context);
#endif
#ifdef CONFIG_STM32_SPI5
static int stm32_spi5_ss_isr(int irq, void *context);
#endif
#ifdef CONFIG_STM32_SPI6
static int stm32_spi6_ss_isr(int irq, void *context);
#endif

#endif /* CONFIG_SPI_SLAVE */

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI1
static const struct spi_ops_s g_sp1iops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = stm32_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = stm32_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
#ifdef CONFIG_SPI_SLAVE
  .slaveregistercallback = spi_registercallback,
#ifdef CONFIG_STM32_SPI_DMA
  .slave_dma_cancel    = spi_rxtxdmastop_slave,
#endif
#endif
};

static struct stm32_spidev_s g_spi1dev =
{
  .spidev   = { &g_sp1iops },
  .spibase  = STM32_SPI1_BASE,
  .spiclock = STM32_PCLK2_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI1,
#endif
#ifdef CONFIG_STM32_SPI_DMA
  .rxch     = DMACHAN_SPI1_RX,
  .txch     = DMACHAN_SPI1_TX,
#endif
#ifdef CONFIG_SPI_SLAVE
  .cb_ops     = NULL,
  .cb_v       = NULL,
  .mode_type  = SPI1_MODE_TYPE,
#else
  .mode_type  = SPI_MODE_TYPE_MASTER,
#endif
#ifdef CONFIG_SPI_CRC16
  .crc16_en   = SPI1_CRC16_EN,
#endif
};
#endif

#ifdef CONFIG_STM32_SPI2
static const struct spi_ops_s g_sp2iops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = stm32_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = stm32_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi2cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
#ifdef CONFIG_SPI_SLAVE
  .slaveregistercallback = spi_registercallback,
#ifdef CONFIG_STM32_SPI_DMA
  .slave_dma_cancel    = spi_rxtxdmastop_slave,
#endif
#endif
};

static struct stm32_spidev_s g_spi2dev =
{
  .spidev   = { &g_sp2iops },
  .spibase  = STM32_SPI2_BASE,
  .spiclock = STM32_PCLK1_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI2,
#endif
#ifdef CONFIG_STM32_SPI_DMA
  .rxch     = DMACHAN_SPI2_RX,
  .txch     = DMACHAN_SPI2_TX,
#endif
#ifdef CONFIG_SPI_SLAVE
  .cb_ops     = NULL,
  .cb_v       = NULL,
  .mode_type  = SPI2_MODE_TYPE,
#else
  .mode_type  = SPI_MODE_TYPE_MASTER,
#endif
#ifdef CONFIG_SPI_CRC16
  .crc16_en   = SPI2_CRC16_EN,
#endif
};
#endif

#ifdef CONFIG_STM32_SPI3
static const struct spi_ops_s g_sp3iops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = stm32_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = stm32_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi3cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
#ifdef CONFIG_SPI_SLAVE
  .slaveregistercallback = spi_registercallback,
#ifdef CONFIG_STM32_SPI_DMA
  .slave_dma_cancel    = spi_rxtxdmastop_slave,
#endif
#endif
};

static struct stm32_spidev_s g_spi3dev =
{
  .spidev   = { &g_sp3iops },
  .spibase  = STM32_SPI3_BASE,
  .spiclock = STM32_PCLK1_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI3,
#endif
#ifdef CONFIG_STM32_SPI_DMA
  .rxch     = DMACHAN_SPI3_RX,
  .txch     = DMACHAN_SPI3_TX,
#endif
#ifdef CONFIG_SPI_SLAVE
  .cb_ops     = NULL,
  .cb_v       = NULL,
  .mode_type  = SPI3_MODE_TYPE,
#else
  .mode_type  = SPI_MODE_TYPE_MASTER,
#endif
#ifdef CONFIG_SPI_CRC16
  .crc16_en   = SPI3_CRC16_EN,
#endif
};
#endif

#ifdef CONFIG_STM32_SPI4
static const struct spi_ops_s g_sp4iops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = stm32_spi4select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = stm32_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi4cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
#ifdef CONFIG_SPI_SLAVE
  .slaveregistercallback = spi_registercallback,
#ifdef CONFIG_STM32_SPI_DMA
  .slave_dma_cancel    = spi_rxtxdmastop_slave,
#endif
#endif
};

static struct stm32_spidev_s g_spi4dev =
{
  .spidev   = { &g_sp4iops },
  .spibase  = STM32_SPI4_BASE,
  .spiclock = STM32_PCLK1_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI4,
#endif
#ifdef CONFIG_STM32_SPI_DMA
  .rxch     = DMACHAN_SPI4_RX,
  .txch     = DMACHAN_SPI4_TX,
#endif
#ifdef CONFIG_SPI_SLAVE
  .cb_ops     = NULL,
  .cb_v       = NULL,
  .mode_type  = SPI4_MODE_TYPE,
#else
  .mode_type  = SPI_MODE_TYPE_MASTER,
#endif
#ifdef CONFIG_SPI_CRC16
  .crc16_en   = SPI4_CRC16_EN,
#endif
};
#endif

#ifdef CONFIG_STM32_SPI5
static const struct spi_ops_s g_sp5iops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = stm32_spi5select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = stm32_spi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi5cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
#ifdef CONFIG_SPI_SLAVE
  .slaveregistercallback = spi_registercallback,
#ifdef CONFIG_STM32_SPI_DMA
  .slave_dma_cancel    = spi_rxtxdmastop_slave,
#endif
#endif
};

static struct stm32_spidev_s g_spi5dev =
{
  .spidev   = { &g_sp5iops },
  .spibase  = STM32_SPI5_BASE,
  .spiclock = STM32_PCLK1_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI5,
#endif
#ifdef CONFIG_STM32_SPI_DMA
  .rxch     = DMACHAN_SPI5_RX,
  .txch     = DMACHAN_SPI5_TX,
#endif
#ifdef CONFIG_SPI_SLAVE
  .cb_ops     = NULL,
  .cb_v       = NULL,
  .mode_type  = SPI5_MODE_TYPE,
#else
  .mode_type  = SPI_MODE_TYPE_MASTER,
#endif
#ifdef CONFIG_SPI_CRC16
  .crc16_en   = SPI5_CRC16_EN,
#endif
};
#endif

#ifdef CONFIG_STM32_SPI6
static const struct spi_ops_s g_sp6iops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = stm32_spi6select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = stm32_spi6status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = stm32_spi3cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
#ifdef CONFIG_SPI_SLAVE
  .slaveregistercallback = spi_registercallback,
#ifdef CONFIG_STM32_SPI_DMA
  .slave_dma_cancel    = spi_rxtxdmastop_slave,
#endif
#endif
};

static struct stm32_spidev_s g_spi6dev =
{
  .spidev   = { &g_sp6iops },
  .spibase  = STM32_SPI6_BASE,
  .spiclock = STM32_PCLK1_FREQUENCY,
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  .spiirq   = STM32_IRQ_SPI6,
#endif
#ifdef CONFIG_STM32_SPI_DMA
  .rxch     = DMACHAN_SPI6_RX,
  .txch     = DMACHAN_SPI6_TX,
#endif
#ifdef CONFIG_SPI_SLAVE
  .cb_ops     = NULL,
  .cb_v       = NULL,
  .mode_type  = SPI6_MODE_TYPE,
#else
  .mode_type  = SPI_MODE_TYPE_MASTER,
#endif
#ifdef CONFIG_SPI_CRC16
  .crc16_en   = SPI6_CRC16_EN,
#endif
};
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ************************************************************************************/

static inline uint16_t spi_getreg(FAR struct stm32_spidev_s *priv, uint8_t offset)
{
  return getreg16(priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ************************************************************************************/

static inline void spi_putreg(FAR struct stm32_spidev_s *priv, uint8_t offset, uint16_t value)
{
  putreg16(value, priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ************************************************************************************/

static inline uint16_t spi_readword(FAR struct stm32_spidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_RXNE) == 0);

  /* Then return the received byte */

  return spi_getreg(priv, STM32_SPI_DR_OFFSET);
}

/************************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one byte to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_writeword(FAR struct stm32_spidev_s *priv, uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_TXE) == 0);

  /* Then send the byte */

  spi_putreg(priv, STM32_SPI_DR_OFFSET, word);
}

#ifdef CONFIG_SPI_SLAVE
/*******************************************************************************
 * Name: spi_registercallback
 *
 * Description:
 *
 *******************************************************************************/
static int spi_registercallback(FAR struct spi_dev_s *dev,
                                const struct spi_cb_ops_s *cb_ops, void *v)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  priv->cb_ops = cb_ops;
  priv->cb_v = v;
  return OK;
}
#endif

/************************************************************************************
 * Name: spi_16bitmode
 *
 * Description:
 *   Check if the SPI is operating in 16-bit mode
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *
 * Returned Value:
 *   true: 16-bit mode, false: 8-bit mode
 *
 ************************************************************************************/

static inline bool spi_16bitmode(FAR struct stm32_spidev_s *priv)
{
#if defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)
  return ((spi_getreg(priv, STM32_SPI_CR2_OFFSET) & SPI_CR2_DS_MASK) == SPI_CR2_DS_16BIT);
#else
  return ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_DFF) != 0);
#endif
}

/************************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmarxwait(FAR struct stm32_spidev_s *priv)
{
  /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
   * must not really have completed???
   */

  while (sem_wait(&priv->rxsem) != 0 || priv->rxresult == 0)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(errno == EINTR);
    }
}
#endif

/************************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmatxwait(FAR struct stm32_spidev_s *priv)
{
  /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
   * must not really have completed???
   */

  while (sem_wait(&priv->txsem) != 0 || priv->txresult == 0)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(errno == EINTR);
    }
}
#endif

/************************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static inline void spi_dmarxwakeup(FAR struct stm32_spidev_s *priv)
{
  (void)sem_post(&priv->rxsem);
}
#endif

/************************************************************************************
 * Name: spi_dmatxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static inline void spi_dmatxwakeup(FAR struct stm32_spidev_s *priv)
{
  (void)sem_post(&priv->txsem);
}
#endif

/************************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)arg;
#ifdef CONFIG_SPI_CRC16
  uint32_t status = 0;
#endif

  if (priv->mode_type == SPI_MODE_TYPE_MASTER)
    {
      /* Wake-up the SPI driver */

      priv->rxresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
      spi_dmarxwakeup(priv);
    }
#ifdef CONFIG_SPI_SLAVE
  else
    {
      spi_modifycr1(priv, SPI_CR1_SSM, 0);
      priv->xfering = false;

#ifdef CONFIG_SPI_CRC16
      status = spi_getreg(priv, STM32_SPI_SR_OFFSET);

      if (priv->crc16_en && (status & SPI_SR_CRCERR))
        {
          /* clear the CRCERR bit */
          status &= ~SPI_SR_CRCERR;
          spi_putreg(priv, STM32_SPI_SR_OFFSET, status);

          if (priv->err_cnt++ < ERR_CNT_ZZZ)
            {
              spidbg("CRC ERR: 0x%X%s\n", status,
                     priv->err_cnt >= ERR_CNT_ZZZ ? " (zzz)" : "");
            }

          spi_portreset(priv);
          spi_portinitialize(priv);

          CALL_CB_IF_SET(priv, txn_err);
        }
      else
#endif
        {
          /* On success, reset the error count */
          priv->err_cnt = 0;

          CALL_CB_IF_SET(priv, txn_end);
        }
    }
#endif
}
#endif

/************************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)arg;

  if (priv->mode_type == SPI_MODE_TYPE_MASTER)
    {
      /* Wake-up the SPI driver */

      priv->txresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
      spi_dmatxwakeup(priv);
    }
}
#endif

/************************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmarxsetup(FAR struct stm32_spidev_s *priv, FAR void *rxbuffer,
                           FAR void *rxdummy, size_t nwords)
{
  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
    {
      /* 16-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          priv->rxccr = SPI_RXDMA16_CONFIG;
        }
      else
        {
          rxbuffer    = rxdummy;
          priv->rxccr = SPI_RXDMA16NULL_CONFIG;
        }
    }
  else
    {
      /* 8-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          priv->rxccr = SPI_RXDMA8_CONFIG;
        }
      else
        {
          rxbuffer    = rxdummy;
          priv->rxccr = SPI_RXDMA8NULL_CONFIG;
        }
     }

  /* Configure the RX DMA */

  stm32_dmasetup(priv->rxdma, priv->spibase + STM32_SPI_DR_OFFSET,
                 (uint32_t)rxbuffer, nwords, priv->rxccr);
}
#endif

/************************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_dmatxsetup(FAR struct stm32_spidev_s *priv, FAR const void *txbuffer,
                           FAR const void *txdummy, size_t nwords)
{
  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
    {
      /* 16-bit mode -- is there a buffer to transfer data from? */

      if (txbuffer)
        {
          priv->txccr = SPI_TXDMA16_CONFIG;
        }
      else
        {
          txbuffer    = txdummy;
          priv->txccr = SPI_TXDMA16NULL_CONFIG;
        }
    }
  else
    {
      /* 8-bit mode -- is there a buffer to transfer data from? */

      if (txbuffer)
        {
          priv->txccr = SPI_TXDMA8_CONFIG;
        }
      else
        {
          txbuffer    = txdummy;
          priv->txccr = SPI_TXDMA8NULL_CONFIG;
        }
    }

  /* Setup the TX DMA */

  stm32_dmasetup(priv->txdma, priv->spibase + STM32_SPI_DR_OFFSET,
                 (uint32_t)txbuffer, nwords, priv->txccr);
}
#endif

/************************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static inline void spi_dmarxstart(FAR struct stm32_spidev_s *priv)
{
  priv->rxresult = 0;
  stm32_dmastart(priv->rxdma, spi_dmarxcallback, priv, false);
}
#endif

/************************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static inline void spi_dmatxstart(FAR struct stm32_spidev_s *priv)
{
  priv->txresult = 0;
  stm32_dmastart(priv->txdma, spi_dmatxcallback, priv, false);
}
#endif


/************************************************************************************
 * Name: spi_rxtxdmastop_slave
 *
 * Description:
 *   End DMA transaction.
 *
 ************************************************************************************/

#if defined(CONFIG_STM32_SPI_DMA) && defined(CONFIG_SPI_SLAVE)
static void spi_rxtxdmastop_slave(FAR struct spi_dev_s *dev)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;

  spi_modifycr1(priv, SPI_CR1_SSM, 0);
  stm32_dmastop(priv->txdma);
  stm32_dmastop(priv->rxdma);

  priv->xfering = false;

  spi_portreset(priv);
  spi_portinitialize(priv);
}
#endif

/************************************************************************************
 * Name: spi_modifycr1
 *
 * Description:
 *   Clear and set bits in the CR1 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_modifycr1(FAR struct stm32_spidev_s *priv, uint16_t setbits, uint16_t clrbits)
{
  uint16_t cr1;
  cr1 = spi_getreg(priv, STM32_SPI_CR1_OFFSET);
  cr1 &= ~clrbits;
  cr1 |= setbits;
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, cr1);
}

/************************************************************************************
 * Name: spi_modifycr2
 *
 * Description:
 *   Clear and set bits in the CR2 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_modifycr2(FAR struct stm32_spidev_s *priv, uint16_t setbits, uint16_t clrbits)
{
  uint16_t cr2;
  cr2 = spi_getreg(priv, STM32_SPI_CR2_OFFSET);
  cr2 &= ~clrbits;
  cr2 |= setbits;
  spi_putreg(priv, STM32_SPI_CR2_OFFSET, cr2);
}

/************************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&priv->exclsem) != 0)
        {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->exclsem);
    }
  return OK;
}
#endif

/************************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint16_t setbits;
  uint32_t actual;

  /* Limit to max possible (if STM32_SPI_CLK_MAX is defined in board.h) */

  if (frequency > STM32_SPI_CLK_MAX)
    {
      frequency = STM32_SPI_CLK_MAX;
    }

  /* Has the frequency changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (frequency != priv->frequency)
    {
#endif
      /* Choices are limited by PCLK frequency with a set of divisors */

      if (frequency >= priv->spiclock >> 1)
        {
          /* More than fPCLK/2.  This is as fast as we can go */

          setbits = SPI_CR1_FPCLCKd2; /* 000: fPCLK/2 */
          actual = priv->spiclock >> 1;
        }
      else if (frequency >= priv->spiclock >> 2)
        {
          /* Between fPCLCK/2 and fPCLCK/4, pick the slower */

          setbits = SPI_CR1_FPCLCKd4; /* 001: fPCLK/4 */
          actual = priv->spiclock >> 2;
       }
      else if (frequency >= priv->spiclock >> 3)
        {
          /* Between fPCLCK/4 and fPCLCK/8, pick the slower */

          setbits = SPI_CR1_FPCLCKd8; /* 010: fPCLK/8 */
          actual = priv->spiclock >> 3;
        }
      else if (frequency >= priv->spiclock >> 4)
        {
          /* Between fPCLCK/8 and fPCLCK/16, pick the slower */

          setbits = SPI_CR1_FPCLCKd16; /* 011: fPCLK/16 */
          actual = priv->spiclock >> 4;
        }
      else if (frequency >= priv->spiclock >> 5)
        {
          /* Between fPCLCK/16 and fPCLCK/32, pick the slower */

          setbits = SPI_CR1_FPCLCKd32; /* 100: fPCLK/32 */
          actual = priv->spiclock >> 5;
        }
      else if (frequency >= priv->spiclock >> 6)
        {
          /* Between fPCLCK/32 and fPCLCK/64, pick the slower */

          setbits = SPI_CR1_FPCLCKd64; /*  101: fPCLK/64 */
          actual = priv->spiclock >> 6;
        }
      else if (frequency >= priv->spiclock >> 7)
        {
          /* Between fPCLCK/64 and fPCLCK/128, pick the slower */

          setbits = SPI_CR1_FPCLCKd128; /* 110: fPCLK/128 */
          actual = priv->spiclock >> 7;
        }
      else
        {
          /* Less than fPCLK/128.  This is as slow as we can go */

          setbits = SPI_CR1_FPCLCKd256; /* 111: fPCLK/256 */
          actual = priv->spiclock >> 8;
        }

      spi_modifycr1(priv, 0, SPI_CR1_SPE);
      spi_modifycr1(priv, setbits, SPI_CR1_BR_MASK);
      spi_modifycr1(priv, SPI_CR1_SPE, 0);

      /* Save the frequency selection so that subsequent reconfigurations will be
       * faster.
       */

      spivdbg("Frequency %d->%d\n", frequency, actual);

#ifndef CONFIG_SPI_OWNBUS
      priv->frequency = frequency;
      priv->actual    = actual;
    }
  return priv->actual;
#else
  return actual;
#endif
}

/************************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;

  spivdbg("mode=%d\n", mode);

  /* Has the mode changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (mode != priv->mode)
    {
#endif
      /* Yes... Set CR1 appropriately */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          setbits = 0;
          clrbits = SPI_CR1_CPOL|SPI_CR1_CPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          setbits = SPI_CR1_CPHA;
          clrbits = SPI_CR1_CPOL;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          setbits = SPI_CR1_CPOL;
          clrbits = SPI_CR1_CPHA;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          setbits = SPI_CR1_CPOL|SPI_CR1_CPHA;
          clrbits = 0;
          break;

        default:
          return;
        }

        spi_modifycr1(priv, 0, SPI_CR1_SPE);
        spi_modifycr1(priv, setbits, clrbits);
        spi_modifycr1(priv, SPI_CR1_SPE, 0);

        /* Save the mode so that subsequent re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
        priv->mode = mode;
    }
#endif
}

/************************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;

  spivdbg("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (nbits != priv->nbits)
    {
#endif
      /* Yes... Set CR1 appropriately */

      switch (nbits)
        {
        case -8:
          setbits = SPI_CR1_LSBFIRST;
#if defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)
          clrbits = SPI_CR1_CRCL;
#else
          clrbits = SPI_CR1_DFF;
#endif
          break;

        case 8:
          setbits = 0;
#if defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)
          clrbits = SPI_CR1_CRCL|SPI_CR1_LSBFIRST;
#else
          clrbits = SPI_CR1_DFF|SPI_CR1_LSBFIRST;
#endif
          break;

        case -16:
#if defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)
          setbits = SPI_CR1_CRCL|SPI_CR1_LSBFIRST;
#else
          setbits = SPI_CR1_DFF|SPI_CR1_LSBFIRST;
#endif
          clrbits = 0;
          break;

        case 16:
#if defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)
          setbits = SPI_CR1_CRCL;
#else
          setbits = SPI_CR1_DFF;
#endif
          clrbits = SPI_CR1_LSBFIRST;
          break;

        default:
          return;
        }

      spi_modifycr1(priv, 0, SPI_CR1_SPE);
      spi_modifycr1(priv, setbits, clrbits);
      spi_modifycr1(priv, SPI_CR1_SPE, 0);

      /* Save the selection so the subsequence re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->nbits = nbits;
    }
#endif
}

/************************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ************************************************************************************/

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint32_t regval;
  uint16_t ret;

  DEBUGASSERT(priv && priv->spibase);

  spi_writeword(priv, wd);
  ret = spi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error flags) */

  regval = spi_getreg(priv, STM32_SPI_SR_OFFSET);

  spivdbg("Sent: %04x Return: %04x Status: %02x\n", wd, ret, regval);
  UNUSED(regval);

  return ret;
}

/************************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#if !defined(CONFIG_STM32_SPI_DMA) || defined(CONFIG_STM32_DMACAPABLE)
#if !defined(CONFIG_STM32_SPI_DMA)
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
#endif
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
    {
      /* 16-bit mode */

      const uint16_t *src  = (const uint16_t*)txbuffer;;
            uint16_t *dest = (uint16_t*)rxbuffer;
            uint16_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
          {
              word = 0xffff;
          }

          /* Exchange one word */

          word = spi_send(dev, word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src  = (const uint8_t*)txbuffer;;
            uint8_t *dest = (uint8_t*)rxbuffer;
            uint8_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
          {
              word = 0xff;
          }

          /* Exchange one word */

          word = (uint8_t)spi_send(dev, (uint16_t)word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
}
#endif /* !CONFIG_STM32_SPI_DMA || CONFIG_STM32_DMACAPABLE */

/*************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI_DMA
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;

#ifdef CONFIG_STM32_DMACAPABLE
  if ((txbuffer && !stm32_dmacapable((uint32_t)txbuffer, nwords, priv->txccr)) ||
      (rxbuffer && !stm32_dmacapable((uint32_t)rxbuffer, nwords, priv->rxccr)))
    {
      /* Unsupported memory region, fall back to non-DMA method. */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
    }
  else
#endif
    {
      static uint16_t rxdummy = 0xffff;
      static const uint16_t txdummy = 0xffff;

      spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);
      DEBUGASSERT(priv && priv->spibase);

      /* Cancel previous setup */

      stm32_dmastop(priv->txdma);
      stm32_dmastop(priv->rxdma);

#ifdef CONFIG_SPI_SLAVE
      if (priv->mode_type == SPI_MODE_TYPE_SLAVE)
        {
          uint32_t status = spi_getreg(priv, STM32_SPI_SR_OFFSET);
          if ((status & SPI_SR_RXNE) || !(status & SPI_SR_TXE))
            {
              spidbg("Invalid status: 0x%08x\n", status);
              spi_rxtxdmastop_slave(dev);
            }
        }
#endif

#ifdef CONFIG_SPI_CRC16
      if (priv->crc16_en)
        {
          spi_modifycr1(priv, 0, SPI_CR1_CRCEN);

          /* Setup DMAs */

          /* RXCRCR and TXCRCR registers are cleared by setting CRCEN bit */
          spi_modifycr1(priv, SPI_CR1_CRCEN, 0);

          /* The counter of the reception DMA channel is set to the number of
           * data frames to receive including the CRC, DMA_RX = Numb_of_data + 2
           */
          spi_dmatxsetup(priv, txbuffer, &txdummy, nwords - 2);
        }
      else
#endif
        {
          spi_dmatxsetup(priv, txbuffer, &txdummy, nwords);
        }

      spi_dmarxsetup(priv, rxbuffer, &rxdummy, nwords);

      /* Start the DMAs */

      spi_dmarxstart(priv);
      spi_dmatxstart(priv);

      if (priv->mode_type == SPI_MODE_TYPE_MASTER)
        {
          /* Then wait for each to complete */

          spi_dmarxwait(priv);
          spi_dmatxwait(priv);
        }
      else
        {
          spi_modifycr1(priv, 0, SPI_CR1_SSM);
        }
    }
}
#endif /* CONFIG_STM32_SPI_DMA */

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords)
{
  spivdbg("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/************************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer, size_t nwords)
{
  spivdbg("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

#ifdef CONFIG_SPI_SLAVE
/************************************************************************************
 * Name: spi_portreset
 *
 * Description:
 *   Resets the specified SPI port to its initial state.
 *
 * Input Parameter:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_portreset(FAR struct stm32_spidev_s *priv)
{
  uint16_t regval;

  switch (priv->spibase)
    {
#ifdef CONFIG_STM32_SPI1
      case STM32_SPI1_BASE:
        regval  = getreg16(STM32_RCC_APB2RSTR);
        regval |= RCC_APB2RSTR_SPI1RST;
        putreg16(regval, STM32_RCC_APB2RSTR);

        regval &= ~(RCC_APB2RSTR_SPI1RST);
        putreg16(regval, STM32_RCC_APB2RSTR);
        break;
#endif

#ifdef CONFIG_STM32_SPI2
      case STM32_SPI2_BASE:
        regval = getreg16(STM32_RCC_APB1RSTR1);
        regval |= RCC_APB1RSTR1_SPI2RST;
        putreg16(regval, STM32_RCC_APB1RSTR1);

        regval &= ~(RCC_APB1RSTR1_SPI2RST);
        putreg16(regval, STM32_RCC_APB1RSTR1);
        break;
#endif

#ifdef CONFIG_STM32_SPI3
      case STM32_SPI3_BASE:
        regval = getreg16(STM32_RCC_APB1RSTR1);
        regval |= RCC_APB1RSTR1_SPI3RST;
        putreg16(regval, STM32_RCC_APB1RSTR1);

        regval &= ~(RCC_APB1RSTR1_SPI3RST);
        putreg16(regval, STM32_RCC_APB1RSTR1);
        break;
#endif

      default:
        break;
    }
}

/************************************************************************************
 * Name: stm32_spi_ss_isr
 *
 * Description:
 *   SPI SS interrupt service routine
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   nss_gpio - Pin for NSS
 *
 ************************************************************************************/

static int stm32_spi_ss_isr(struct stm32_spidev_s *priv, int nss_gpio)
{
  int nss = stm32_gpioread(nss_gpio);

  /* Check for start of transfer */
  if (!nss && !(spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SSM))
    {
      priv->xfering = true;
      CALL_CB_IF_SET(priv, txn_start);
    }

  /* Check for end of frame before entire buffer is filled */
  else if (priv->xfering)
    {
      spi_rxtxdmastop_slave((FAR struct spi_dev_s *)priv);

      if (priv->err_cnt++ < ERR_CNT_ZZZ)
        {
          spidbg("Early EOF%s\n", priv->err_cnt >= ERR_CNT_ZZZ ? " (zzz)" : "");
        }
      CALL_CB_IF_SET(priv, txn_err);
    }

  return OK;
}

#ifdef CONFIG_STM32_SPI1
/************************************************************************************
 * Name: stm32_spi1_ss_isr
 *
 * Description:
 *   SPI1 SS interrupt service routine
 *
 ************************************************************************************/

static int stm32_spi1_ss_isr(int irq, void *context)
{
  return stm32_spi_ss_isr(&g_spi1dev, GPIO_SPI1_NSS);
}
#endif

#ifdef CONFIG_STM32_SPI2
/************************************************************************************
 * Name: stm32_spi2_ss_isr
 *
 * Description:
 *   SPI2 SS interrupt service routine
 *
 ************************************************************************************/

static int stm32_spi2_ss_isr(int irq, void *context)
{
  return stm32_spi_ss_isr(&g_spi2dev, GPIO_SPI2_NSS);
}
#endif

#ifdef CONFIG_STM32_SPI3
/************************************************************************************
 * Name: stm32_spi3_ss_isr
 *
 * Description:
 *   SPI3 SS interrupt service routine
 *
 ************************************************************************************/

static int stm32_spi3_ss_isr(int irq, void *context)
{
  return stm32_spi_ss_isr(&g_spi3dev, GPIO_SPI3_NSS);
}
#endif

#ifdef CONFIG_STM32_SPI4
/************************************************************************************
 * Name: stm32_spi4_ss_isr
 *
 * Description:
 *   SPI4 SS interrupt service routine
 *
 ************************************************************************************/

static int stm32_spi4_ss_isr(int irq, void *context)
{
  return stm32_spi_ss_isr(&g_spi4dev, GPIO_SPI4_NSS);
}
#endif

#ifdef CONFIG_STM32_SPI5
/************************************************************************************
 * Name: stm32_spi5_ss_isr
 *
 * Description:
 *   SPI5 SS interrupt service routine
 *
 ************************************************************************************/

static int stm32_spi5_ss_isr(int irq, void *context)
{
  return stm32_spi_ss_isr(&g_spi5dev, GPIO_SPI5_NSS);
}
#endif

#ifdef CONFIG_STM32_SPI6
/************************************************************************************
 * Name: stm32_spi6_ss_isr
 *
 * Description:
 *   SPI6 SS interrupt service routine
 *
 ************************************************************************************/

static int stm32_spi6_ss_isr(int irq, void *context)
{
  return stm32_spi_ss_isr(&g_spi6dev, GPIO_SPI6_NSS);
}
#endif
#endif /* CONFIG_SPI_SLAVE */

/************************************************************************************
 * Name: spi_portinitialize
 *
 * Description:
 *   Initialize the selected SPI port in its default state (Master, 8-bit, mode 0, etc.)
 *
 * Input Parameter:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_portinitialize(FAR struct stm32_spidev_s *priv)
{
  uint16_t setbits;
  uint16_t clrbits;

  if (priv->mode_type == SPI_MODE_TYPE_SLAVE)
    {
      /* Configure CR1. Slave configuration:
       *   Mode 0:                        CPHA=0 and CPOL=0
       *   Master:                        MSTR=0
       *   8-bit:                         DFF=0
       *   8-bit:                         CRCL=0
       *   MSB tranmitted first:          LSBFIRST=0
       *   Replace NSS with SSI & SSI=1:  SSI=1 SSM=1 (prevents MODF error)
       *   Two lines full duplex:         BIDIMODE=0 BIDIOIE=(Don't care) and RXONLY=0
       */
#if defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)
      clrbits = SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_MSTR|SPI_CR1_BR_MASK|SPI_CR1_LSBFIRST|
                SPI_CR1_RXONLY|SPI_CR1_CRCL|SPI_CR1_BIDIOE|SPI_CR1_BIDIMODE;
#else
      clrbits = SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_MSTR|SPI_CR1_BR_MASK|SPI_CR1_LSBFIRST|
                SPI_CR1_RXONLY|SPI_CR1_DFF|SPI_CR1_BIDIOE|SPI_CR1_BIDIMODE;
#endif
      setbits = SPI_CR1_SSI|SPI_CR1_SSM;
    }
  else
    {
      /* Configure CR1. Default configuration:
       *   Mode 0:                        CPHA=0 and CPOL=0
       *   Master:                        MSTR=1
       *   8-bit:                         DFF=0
       *   8-bit:                         CRCL=0
       *   MSB tranmitted first:          LSBFIRST=0
       *   Replace NSS with SSI & SSI=1:  SSI=1 SSM=1 (prevents MODF error)
       *   Two lines full duplex:         BIDIMODE=0 BIDIOIE=(Don't care) and RXONLY=0
       */
#if defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)
      clrbits = SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_BR_MASK|SPI_CR1_LSBFIRST|
                SPI_CR1_RXONLY|SPI_CR1_CRCL|SPI_CR1_BIDIOE|SPI_CR1_BIDIMODE;
#else
      clrbits = SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_BR_MASK|SPI_CR1_LSBFIRST|
                SPI_CR1_RXONLY|SPI_CR1_DFF|SPI_CR1_BIDIOE|SPI_CR1_BIDIMODE;
#endif
      setbits = SPI_CR1_MSTR|SPI_CR1_SSI|SPI_CR1_SSM;
    }
  spi_modifycr1(priv, setbits, clrbits);

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;
#endif

  /* Select a default frequency of approx. 400KHz */

  if (priv->mode_type == SPI_MODE_TYPE_MASTER)
    {
      spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);
    }

#if defined(CONFIG_SPI_CRC16) && \
    (defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3))
  if (priv->crc16_en)
    {
      /* CRC setup */

      spi_modifycr1(priv, SPI_CR1_CRCL, 0);
    }
#endif

  /* CRCPOLY configuration */

  spi_putreg(priv, STM32_SPI_CRCPR_OFFSET, 0x8005);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

#ifndef CONFIG_SPI_OWNBUS
  sem_init(&priv->exclsem, 0, 1);
#endif

  /* Initialize the SPI semaphores that is used to wait for DMA completion */

#ifdef CONFIG_STM32_SPI_DMA
  sem_init(&priv->rxsem, 0, 0);
  sem_init(&priv->txsem, 0, 0);

  /* Get DMA channels.  NOTE: stm32_dmachannel() will always assign the DMA channel.
   * if the channel is not available, then stm32_dmachannel() will block and wait
   * until the channel becomes available.  WARNING: If you have another device sharing
   * a DMA channel with SPI and the code never releases that channel, then the call
   * to stm32_dmachannel()  will hang forever in this function!  Don't let your
   * design do that!
   */

  if (!priv->rxdma)
      priv->rxdma = stm32_dmachannel(priv->rxch);
  if (!priv->txdma)
      priv->txdma = stm32_dmachannel(priv->txch);

  DEBUGASSERT(priv->rxdma && priv->txdma);

  spi_putreg(priv, STM32_SPI_CR2_OFFSET, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
#endif

#ifdef SPI_CR2_FRXTH
  spi_modifycr2(priv, SPI_CR2_FRXTH, 0);
#endif

  /* Enable spi */

  spi_modifycr1(priv, SPI_CR1_SPE, 0);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  FAR struct stm32_spidev_s *priv = NULL;

  irqstate_t flags = irqsave();

#ifdef CONFIG_STM32_SPI1
  if (port == 1)
    {
      /* Select SPI1 */

      priv = &g_spi1dev;

      /* Only configure if the port is not already configured */

      if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI1_SCK);
          stm32_configgpio(GPIO_SPI1_MISO);
          stm32_configgpio(GPIO_SPI1_MOSI);

#ifdef CONFIG_SPI_SLAVE
          if (priv->mode_type == SPI_MODE_TYPE_SLAVE)
            stm32_gpiosetevent(GPIO_SPI1_NSS, true, true, true, stm32_spi1_ss_isr);
#endif
          /* Set up default configuration: Master, 8-bit, etc. */

          spi_portinitialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI2
  if (port == 2)
    {
      /* Select SPI2 */

      priv = &g_spi2dev;

      /* Only configure if the port is not already configured */

      if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI2_SCK);
          stm32_configgpio(GPIO_SPI2_MISO);
          stm32_configgpio(GPIO_SPI2_MOSI);

#ifdef CONFIG_SPI_SLAVE
          if (priv->mode_type == SPI_MODE_TYPE_SLAVE)
            stm32_gpiosetevent(GPIO_SPI2_NSS, true, true, true, stm32_spi2_ss_isr);
#endif
          /* Set up default configuration: Master, 8-bit, etc. */

          spi_portinitialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI3
  if (port == 3)
    {
      /* Select SPI3 */

      priv = &g_spi3dev;

      /* Only configure if the port is not already configured */

      if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI3 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI3_SCK);
          stm32_configgpio(GPIO_SPI3_MISO);
          stm32_configgpio(GPIO_SPI3_MOSI);

#ifdef CONFIG_SPI_SLAVE
          if (priv->mode_type == SPI_MODE_TYPE_SLAVE)
            stm32_gpiosetevent(GPIO_SPI3_NSS, true, true, true, stm32_spi3_ss_isr);
#endif
          /* Set up default configuration: Master, 8-bit, etc. */

          spi_portinitialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI4
  if (port == 4)
    {
      /* Select SPI4 */

      priv = &g_spi4dev;

      /* Only configure if the port is not already configured */

      if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI4 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI4_SCK);
          stm32_configgpio(GPIO_SPI4_MISO);
          stm32_configgpio(GPIO_SPI4_MOSI);

#ifdef CONFIG_SPI_SLAVE
          if (priv->mode_type == SPI_MODE_TYPE_SLAVE)
            stm32_gpiosetevent(GPIO_SPI4_NSS, true, true, true, stm32_spi4_ss_isr);
#endif
          /* Set up default configuration: Master, 8-bit, etc. */

          spi_portinitialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI5
  if (port == 5)
    {
      /* Select SPI5 */

      priv = &g_spi5dev;

      /* Only configure if the port is not already configured */

      if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI5 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI5_SCK);
          stm32_configgpio(GPIO_SPI5_MISO);
          stm32_configgpio(GPIO_SPI5_MOSI);

#ifdef CONFIG_SPI_SLAVE
          if (priv->mode_type == SPI_MODE_TYPE_SLAVE)
            stm32_gpiosetevent(GPIO_SPI5_NSS, true, true, true, stm32_spi5_ss_isr);
#endif
          /* Set up default configuration: Master, 8-bit, etc. */

          spi_portinitialize(priv);
        }
    }
  else
#endif
#ifdef CONFIG_STM32_SPI6
  if (port == 6)
    {
      /* Select SPI6 */

      priv = &g_spi6dev;

      /* Only configure if the port is not already configured */

      if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI6 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI6_SCK);
          stm32_configgpio(GPIO_SPI6_MISO);
          stm32_configgpio(GPIO_SPI6_MOSI);

#ifdef CONFIG_SPI_SLAVE
          if (priv->mode_type == SPI_MODE_TYPE_SLAVE)
            stm32_gpiosetevent(GPIO_SPI6_NSS, true, true, true, stm32_spi6_ss_isr);
#endif
          /* Set up default configuration: Master, 8-bit, etc. */

          spi_portinitialize(priv);
        }
    }
  else
#endif
    {
      spidbg("ERROR: Unsupported SPI port: %d\n", port);
      return NULL;
    }

  irqrestore(flags);
  return (FAR struct spi_dev_s *)priv;
}

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 || CONFIG_STM32_SPI3 || CONFIG_STM32_SPI4 || CONFIG_STM32_SPI5 || CONFIG_STM32_SPI6 */
