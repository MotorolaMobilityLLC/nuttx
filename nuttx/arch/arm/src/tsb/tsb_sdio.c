/**
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
#include <debug.h>

#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_sdio.h>
#include <nuttx/device_sdio_board.h>

#include <nuttx/gpio.h>
#include "up_arch.h"
#include "tsb_scm.h"

/* SDIO flags */
#define SDIO_FLAG_OPEN  BIT(0)
#define SDIO_FLAG_WRITE BIT(1)
#define SDIO_FLAG_READ  BIT(2)

/* System controller registers. The base is SYSCTL_BASE (0x40000000) */
#define UHSSD_DLLCTRL 0x628
#define UHSSD_IO3CTRL 0x64C
#define UHSSD_IO4CTRL 0x650
#define UHSSD_IO5CTRL 0x654
#define UHSSD_IO6CTRL 0x658
#define UHSSD_IO7CTRL 0x65C

/* UHSSD_DLLCTRL bit field details */
#define DLL_ENABLE BIT(0)

/* UHSSD_IO3CTRL bit field details */
#define SDCMD_PUENABLE BIT(0)

/* UHSSD_IO4CTRL bit field details */
#define SDDATA3_PUENABLE BIT(0)

/* UHSSD_IO5CTRL bit field details */
#define SDDATA2_PUENABLE BIT(0)

/* UHSSD_IO6CTRL bit field details */
#define SDDATA1_PUENABLE BIT(0)

/* UHSSD_IO7CTRL bit field details */
#define SDDATA0_PUENABLE BIT(0)

/* SD card/SDIO registers. The base is 0x40019000 */
#define SDMASYSADDR                  0x00
#define BLK_SIZE_COUNT               0x04
#define ARGUMENT1                    0x08
#define CMD_TRANSFERMODE             0x0C
#define RESPONSEREG0                 0x10
#define RESPONSEREG1                 0x14
#define RESPONSEREG2                 0x18
#define RESPONSEREG3                 0x1C
#define DATAPORTREG                  0x20
#define PRESENTSTATE                 0x24
#define HOST_PWR_BLKGAP_WAKEUP_CNTRL 0x28
#define CLOCK_SWRST_TIMEOUT_CONTROL  0x2C
#define INT_ERR_STATUS               0x30
#define INT_ERR_STATUS_EN            0x34
#define INT_ERR_SIGNAL_EN            0x38
#define AUTOCMD12ERST_HOST_CTRL2     0x3C
#define CAPABILITIES_LOW             0x40
#define CAPABILITIES_HI              0x44
#define MAXCURCAPAB                  0x48
#define FRCAUTOCMD_ERR_ERRINT_ST     0x50
#define ADMAERSTATUS                 0x54
#define ADMASYSADDR32                0x58
#define ADMASYSADDR64                0x5C
#define PRESETVAL_INIT_DFLT          0x60
#define PRESETVAL_HS_SDR12           0x64
#define PRESETVAL_SDR25_SDR50        0x68
#define PRESETVAL_SDR104_DDR50       0x6C
#define BOOTTIMCTRL                  0x70
#define SLOTINTSTATUS_HSTCTRLVERSION 0xFC

/* CMD_TRANSFERMODE bit field details */
#define BLOCK_COUNT_ENABLE        BIT(1)
#define TRANSFER_DIRECTION_SELECT BIT(4)
#define MULTI_SINGLE_BLOCK_SELECT BIT(5)

/* CMD_TRANSFERMODE definition */
#define COMMAND_CRC_CHECK_EN_DEF   0x08
#define COMMAND_INDEX_CHECK_EN_DEF 0x10
#define DATA_PRESENT_DEF           0x20
#define COMMAND_TYPE_DEF           0xC0
#define SDIO_MAKE_CMD(c, f)        (((c & 0xFF) << 8) | (f & 0xFF))

/* CMD_TRANSFERMODE RESPONSE_TYPE definition */
#define NO_RESPONSE             0
#define RESPONSE_LENGTH_136     1
#define RESPONSE_LENGTH_48      2
#define RESPONSE_LENGTH_BUSY_48 3

/* PRESENTSTATE bit shift definition */
#define COMMAND_INHIBIT_CMD   BIT(0)
#define COMMAND_INHIBIT_DAT   BIT(1)
#define DAT_LINE_ACTIVE       BIT(2)
#define WRITE_TRANSFER_ACTIVE BIT(8)
#define READ_TRANSFER_ACTIVE  BIT(9)
#define BUFFER_WRITE_ENABLE   BIT(10)
#define BUFFER_READ_ENABLE    BIT(11)

/* HOST_PWR_BLKGAP_WAKEUP_CNTRL bit field details */
#define DATA_TRASFER_WIDTH BIT(1)
#define SD_BUS_POWER       BIT(8)

/* HOST_PWR_BLKGAP_WAKEUP_CNTRL bit mask definition */
#define SD_BUS_VOLTAGE_SELECT_MASK 0x00000E00

/* HOST_PWR_BLKGAP_WAKEUP_CNTRL bit shift definition */
#define SD_BUS_VOLTAGE_SELECT_SHIFT 9

/* HOST_PWR_BLKGAP_WAKEUP_CNTRL SD_BUS_VOLTAGE_SELECT definition */
#define SUPPORT_1_8V 0x5
#define SUPPORT_3_0V 0x6
#define SUPPORT_3_3V 0x7

/* CLOCK_SWRST_TIMEOUT_CONTROL bit field details */
#define INTERNAL_CLOCK_EN     BIT(0)
#define INTERNAL_CLOCK_STABLE BIT(1)
#define SD_CLOCK_ENABLE       BIT(2)
#define SW_RESET_CMD_LINE     BIT(25)
#define SW_RESET_DAT_LINE     BIT(26)

/* CLOCK_SWRST_TIMEOUT_CONTROL bit mask definition */
#define SDCLK_FREQ_SELECT_MASK 0x0000FF00

/* CLOCK_SWRST_TIMEOUT_CONTROL bit shift definition */
#define SDCLK_FREQ_SELECT_SHIFT 8

/* INT_ERR_STATUS bit field details */
#define COMMAND_COMPLETE         BIT(0)
#define TRANSFER_COMPLETE        BIT(1)
#define BLOCKGAP_EVENT           BIT(2)
#define DMA_INTERRUPT            BIT(3)
#define BUFFER_WRITE_RDY         BIT(4)
#define BUFFER_READ_RDY          BIT(5)
#define CARD_INSERTION           BIT(6)
#define CARD_REMOVAL             BIT(7)
#define CARD_INTERRUPT           BIT(8)
#define INT_A                    BIT(9)
#define INT_B                    BIT(10)
#define INT_C                    BIT(11)
#define RETUNING_EVENT           BIT(12)
#define BOOT_ACK_RCV             BIT(13)
#define BOOT_TERMINATE_INTERRUPT BIT(14)
#define ERROR_INTERRUPT          BIT(15)
#define CMD_TIMEOUT_ERROR        BIT(16)
#define CMD_CRC_ERROR            BIT(17)
#define CMD_END_BIT_ERROR        BIT(18)
#define CMD_INDEX_ERROR          BIT(19)
#define DATA_TIMEOUT_ERROR       BIT(20)
#define DATA_CRC_ERROR           BIT(21)
#define DATA_END_BIT_ERROR       BIT(22)
#define CURRENT_LIMIT_ERROR      BIT(23)
#define AUTO_CMD_ERROR           BIT(24)
#define ADMA_ERROR               BIT(25)
#define TARGET_RESPONSE_ERROR    BIT(28)

/* INT_ERR_STATUS_EN bit field details */
#define CMD_COMPLETE_STAT_EN       BIT(0)
#define TRANSFER_COMPLETE_STAT_EN  BIT(1)
#define BUFFER_WRITE_READY_STAT_EN BIT(4)
#define BUFFER_READ_READY_STAT_EN  BIT(5)
#define CARD_INTERRUPT_STAT_EN     BIT(8)
#define CMD_TIMEOUT_ERR_STAT_EN    BIT(16)
#define CMD_CRC_ERR_STAT_EN        BIT(17)
#define CMD_END_BIT_ERR_STAT_EN    BIT(18)
#define CMD_INDEX_ERR_STAT_EN      BIT(19)
#define DATA_TIMEOUT_ERR_STAT_EN   BIT(20)
#define DATA_CRC_ERR_STAT_EN       BIT(21)
#define DATA_END_BIT_ERR_STAT_EN   BIT(22)

/* INT_ERR_SIGNAL_EN bit field details */
#define CMD_COMPLETE_EN       BIT(0)
#define TRANSFER_COMPLETE_EN  BIT(1)
#define BUFFER_WRITE_READY_EN BIT(4)
#define BUFFER_READ_READY_EN  BIT(5)
#define CARD_INTERRUPT_EN     BIT(8)
#define CMD_TIMEOUT_ERR_EN    BIT(16)
#define CMD_CRC_ERR_EN        BIT(17)
#define CMD_END_BIT_ERR_EN    BIT(18)
#define CMD_INDEX_ERR_EN      BIT(19)
#define DATA_TIMEOUT_ERR_EN   BIT(20)
#define DATA_CRC_ERR_EN       BIT(21)
#define DATA_END_BIT_ERR_EN   BIT(22)

/* AUTOCMD12ERST_HOST_CTRL2 bit mask definition */
#define UHS_MODE_SEL_MASK 0x00070000

/* AUTOCMD12ERST_HOST_CTRL2 bit shift definition */
#define UHS_MODE_SEL_SHIFT 16

/* AUTOCMD12ERST_HOST_CTRL2 UHS_MODE_SEL definition */
#define SDR12  0x0
#define SDR25  0x1
#define SDR50  0x2
#define SDR104 0x3
#define DDR50  0x4

/* CAPABILITIES_LOW bit field details */
#define VOLTAGE_SUPPORT_3_3V BIT(24)
#define VOLTAGE_SUPPORT_3_0V BIT(25)
#define VOLTAGE_SUPPORT_1_8V BIT(26)

/* CAPABILITIES_LOW bit mask definition */
#define BASE_CLOCK_FREQ_FOR_SD_CLOCK_MASK 0x0000FF00

/* CAPABILITIES_LOW bit shift definition */
#define BASE_CLOCK_FREQ_FOR_SD_CLOCK_SHIFT 8

/* CAPABILITIES_HI bit field details */
#define DRIVER_TYPEA_SUPPORT BIT(4)
#define DRIVER_TYPEC_SUPPORT BIT(5)
#define DRIVER_TYPED_SUPPORT BIT(6)

/* Host controller SDIO get capabilities bit masks */
#define HC_SDIO_CAP_NONREMOVABLE   0x00000001 /* Device is unremovable from
                                               * the slot */
#define HC_SDIO_CAP_4_BIT_DATA     0x00000002 /* Host support 4 bit transfers */
#define HC_SDIO_CAP_8_BIT_DATA     0x00000004 /* Host support 8 bit transfers */
#define HC_SDIO_CAP_MMC_HS         0x00000008 /* CHost support mmc high-speed
                                               * timings */
#define HC_SDIO_CAP_SD_HS          0x00000010 /* Host support SD high-speed
                                               * timings */
#define HC_SDIO_CAP_ERASE          0x00000020 /* Host allow erase and trim
                                               * commands */
#define HC_SDIO_CAP_1_2V_DDR       0x00000040 /* Host support DDR mode
                                               * at 1.2V */
#define HC_SDIO_CAP_1_8V_DDR       0x00000080 /* Host support DDR mode
                                               * at 1.8V */
#define HC_SDIO_CAP_POWER_OFF_CARD 0x00000100 /* Host can power off card */
#define HC_SDIO_CAP_UHS_SDR12      0x00000200 /* Host support UHS SDR12 mode */
#define HC_SDIO_CAP_UHS_SDR25      0x00000400 /* Host support UHS SDR25 mode */
#define HC_SDIO_CAP_UHS_SDR50      0x00000800 /* Host support UHS SDR50 mode */
#define HC_SDIO_CAP_UHS_SDR104     0x00001000 /* Host support UHS SDR104 mode */
#define HC_SDIO_CAP_UHS_DDR50      0x00002000 /* Host support UHS DDR50 mode */
#define HC_SDIO_CAP_DRIVER_TYPE_A  0x00004000 /* Host support Driver Type A */
#define HC_SDIO_CAP_DRIVER_TYPE_C  0x00008000 /* Host support Driver Type C */
#define HC_SDIO_CAP_DRIVER_TYPE_D  0x00010000 /* Host support Driver Type D */
#define HC_SDIO_CAP_HS200_1_2V     0x00020000 /* Host support HS200 mode
                                               * at 1.2V */
#define HC_SDIO_CAP_HS200_1_8V     0x00040000 /* Host support HS200 mode
                                               * at 1.8V */
#define HC_SDIO_CAP_HS400_1_2V     0x00080000 /* Host support HS400 mode
                                               * at 1.2V */
#define HC_SDIO_CAP_HS400_1_8V     0x00100000 /* Host support HS400 mode
                                               * at 1.8V */

/* Host controller SDIO voltage range bit masks */
#define HC_SDIO_VDD_165_195 0x00000001
#define HC_SDIO_VDD_20_21   0x00000002
#define HC_SDIO_VDD_21_22   0x00000004
#define HC_SDIO_VDD_22_23   0x00000008
#define HC_SDIO_VDD_23_24   0x00000010
#define HC_SDIO_VDD_24_25   0x00000020
#define HC_SDIO_VDD_25_26   0x00000040
#define HC_SDIO_VDD_26_27   0x00000080
#define HC_SDIO_VDD_27_28   0x00000100
#define HC_SDIO_VDD_28_29   0x00000200
#define HC_SDIO_VDD_29_30   0x00000400
#define HC_SDIO_VDD_30_31   0x00000800
#define HC_SDIO_VDD_31_32   0x00001000
#define HC_SDIO_VDD_32_33   0x00002000
#define HC_SDIO_VDD_33_34   0x00004000
#define HC_SDIO_VDD_34_35   0x00008000
#define HC_SDIO_VDD_35_36   0x00010000

/* Host controller SDIO power mode definition */
#define HC_SDIO_POWER_OFF       0x00
#define HC_SDIO_POWER_UP        0x01
#define HC_SDIO_POWER_ON        0x02
#define HC_SDIO_POWER_UNDEFINED 0x03

/* Host controller SDIO bus width definition */
#define HC_SDIO_BUS_WIDTH_1 0x00
#define HC_SDIO_BUS_WIDTH_4 0x02
#define HC_SDIO_BUS_WIDTH_8 0x03

/* Host controller SDIO timing definition */
#define HC_SDIO_TIMING_LEGACY     0x00
#define HC_SDIO_TIMING_MMC_HS     0x01
#define HC_SDIO_TIMING_SD_HS      0x02
#define HC_SDIO_TIMING_UHS_SDR12  0x03
#define HC_SDIO_TIMING_UHS_SDR25  0x04
#define HC_SDIO_TIMING_UHS_SDR50  0x05
#define HC_SDIO_TIMING_UHS_SDR104 0x06
#define HC_SDIO_TIMING_UHS_DDR50  0x07
#define HC_SDIO_TIMING_MMC_DDR52  0x08
#define HC_SDIO_TIMING_MMC_HS200  0x09
#define HC_SDIO_TIMING_MMC_HS400  0x0A

/* Host controller SDIO driver type definition */
#define HC_SDIO_SET_DRIVER_TYPE_B 0x00
#define HC_SDIO_SET_DRIVER_TYPE_A 0x01
#define HC_SDIO_SET_DRIVER_TYPE_C 0x02
#define HC_SDIO_SET_DRIVER_TYPE_D 0x03

/* Host controller SDIO command flags definition */
#define HC_SDIO_RSP_NONE    0x00
#define HC_SDIO_RSP_PRESENT 0x01
#define HC_SDIO_RSP_136     0x02
#define HC_SDIO_RSP_CRC     0x04
#define HC_SDIO_RSP_BUSY    0x08
#define HC_SDIO_RSP_OPCODE  0x10

/* Host controller SDIO command type definition */
#define HC_SDIO_CMD_AC   0x00 /* Normal, Addressed Command */
#define HC_SDIO_CMD_ADTC 0x01 /* Suspend, Addressed Data Transfer Command */
#define HC_SDIO_CMD_BC   0x02 /* Resume, Broadcasted Command, no response */
#define HC_SDIO_CMD_BCR  0x03 /* Abort, Broadcasted Command with response */

/* Host controller SDIO response definition */
#define HC_SDIO_RSP_R1_R5_R6_R7 (HC_SDIO_RSP_PRESENT | HC_SDIO_RSP_CRC | \
                                 HC_SDIO_RSP_OPCODE)
#define HC_SDIO_RSP_R1B         (HC_SDIO_RSP_PRESENT | HC_SDIO_RSP_CRC | \
                                 HC_SDIO_RSP_OPCODE | HC_SDIO_RSP_BUSY)
#define HC_SDIO_RSP_R2          (HC_SDIO_RSP_PRESENT | HC_SDIO_RSP_CRC | \
                                 HC_SDIO_RSP_136)
#define HC_SDIO_RSP_R3_R4       (HC_SDIO_RSP_PRESENT)

/* Host controller commands */
/* Standard MMC commands (4.1)      type  argument                   response */
/* class 1 */
#define HC_MMC_SWITCH               6  /* ac   [31:0] See below      R1b      */
/*
 * HC_MMC_SWITCH argument format:
 * [31:26] Always 0
 * [25:24] Access Mode
 * [23:16] Location of target Byte in EXT_CSD
 * [15:08] Value Byte
 * [07:03] Always 0
 * [02:00] Command Set
 */
#define HC_MMC_STOP_TRANSMISSION    12 /* ac                         R1b      */
/* class 2 */
#define HC_MMC_SET_BLOCKLEN         16 /* ac   [31:0] block len      R1       */
#define HC_MMC_READ_SINGLE_BLOCK    17 /* adtc [31:0] data addr      R1       */
#define HC_MMC_READ_MULTIPLE_BLOCK  18 /* adtc [31:0] data addr      R1       */
/* class 4 */
#define HC_MMC_SET_BLOCK_COUNT      23 /* adtc [31:0] data addr      R1       */
#define HC_MMC_WRITE_BLOCK          24 /* adtc [31:0] data addr      R1       */
#define HC_MMC_WRITE_MULTIPLE_BLOCK 25 /* adtc                       R1       */
/* class 7 */
#define HC_MMC_LOCK_UNLOCK          42 /* adtc                       R1b      */
/* class 8 */
#define HC_MMC_APP_CMD              55 /* ac   [31:16] RCA           R1       */

/* SD commands                      type  argument                   response */
/* Application commands */
#define HC_SD_APP_SET_BUS_WIDTH     6  /* ac   [1:0] bus width       R1       */
#define HC_SD_APP_SD_STATUS         13 /* adtc                       R1       */
#define HC_SD_APP_SEND_SCR          51 /* adtc                       R1       */

/* Host controller SDIO transfer data flags */
#define HC_GB_SDIO_DATA_WRITE  0x01 /* Request to be written */
#define HC_GB_SDIO_DATA_READ   0x02 /* Response to be read */
#define HC_GB_SDIO_DATA_STREAM 0x04 /* Data will be transfer until a cancel
                                     * command is send */

/* Host controller SDIO event bit mask definition */
#define HC_SDIO_CARD_INSERTED 0x01
#define HC_SDIO_CARD_REMOVED  0x02
#define HC_SDIO_WP            0x04

/* Block count definition */
#define MAX_BLK_COUNT       65535
#define MAX_BLK_SIZE        2048
#define BLOCK_LENGTH        512
#define SCR_BLOCK_LENGTH    8
#define STATUS_BLOCK_LENGTH 64
#define SWITCH_BLOCK_LENGTH 64
#define SINGLE_BLOCK_COUNT  1

/* SDIO clock definition */
#define MIN_FREQ_SUPPORT 400000   /* 400KHz */
#define MAX_FREQ_SUPPORT 50000000 /* 50MHz */

/* MHZ definition */
#define MHZ_DEFINE 1000 * 1000 /* Used for transferring Hz to MHz */

/* Data shift definition */
#define BYTE_SHIFT 8

/* Command interval definition */
#define COMMAND_INTERVAL 1000 /* 1ms */

/* Register definition */
#define REGISTER_INTERVAL  1000 /* 1ms */
#define REGISTER_MAX_RETRY 10

/* R2 response bit mask definition */
#define R2_RSP_MASK 0xFF000000

/* Register offset definition */
#define REG_OFFSET 0x02

/* SDIO buffer structure */
struct sdio_buffer
{
  int16_t head; /* Index to the head [IN] index in the buffer */
  int16_t tail; /* Index to the tail [OUT] index in the buffer */
  uint8_t *buffer; /* Pointer to the allocated buffer memory */
};

/**
 * @brief SDIO device private information
 */
struct tsb_sdio_info {
    /** Device driver handler */
    struct device *dev;
    /** SDIO write buffer structure */
    struct sdio_buffer write_buf;
    /** SDIO read buffer structure */
    struct sdio_buffer read_buf;
    /** Host controller state */
    uint32_t flags;
    /** SDIO register base */
    uint32_t sdio_reg_base;
    /** Command response */
    uint32_t resp[4];
    /** Card detect pin number */
    uint32_t cd_pin_number;
    /** SDIO number of blocks of data to transfer */
    uint16_t blocks;
    /** SDIO size of the blocks of data to transfer */
    uint16_t blksz;
    /** Card event type */
    uint8_t card_event;
    /** Previous card event type */
    uint8_t pre_card_event;
    /** SDIO command flag */
    uint8_t cmd_flags;
    /** SDIO data flag */
    uint8_t data_flags;
    /** Command semaphore */
    sem_t cmd_sem;
    /** Write semaphore */
    sem_t write_sem;
    /** Read semaphore */
    sem_t read_sem;
    /** DMA handler */
    void *dma;
    /** SDIO IRQ number */
    int sdio_irq;
    /** SDIO interrupt error status for the return of command */
    int sdio_int_err_status;
    /** Error status for SD host controller */
    int err_status;
    /** Data thread exit flag */
    bool thread_abort;
    /** Application command (command 55) flag */
    bool app_cmd;
    /** Data command flag */
    bool data_cmd;
    /** Data timeout flag */
    bool data_timeout;
    /** Write data thread handle */
    pthread_t write_data_thread;
    /** Read data thread handle */
    pthread_t read_data_thread;
    /** Write complete callback function*/
    sdio_transfer_callback write_callback;
    /** Read complete callback function*/
    sdio_transfer_callback read_callback;
    /** Event callback function */
    sdio_event_callback callback;
};

static struct device *sdio_dev = NULL;
static struct device *sdio_board_dev = NULL;

/**
 * @brief Get SDIO register value.
 *
 * @param base The SDIO register base address.
 * @param offset The SDIO register offset address.
 * @return Register value.
 */
static uint32_t sdio_getreg(uint32_t base, uint32_t offset)
{
    return getreg32(base + offset);
}

/**
 * @brief Put value to SDIO register.
 *
 * @param base The SDIO register base address.
 * @param offset The SDIO register offset address.
 * @param value The value to put.
 * @return None.
 */
static void sdio_putreg(uint32_t base, uint32_t offset, uint32_t value)
{
    putreg32(value, base + offset);
}

/**
 * @brief Put 16-bits value to SDIO register.
 *
 * @param base The SDIO register base address.
 * @param offset The SDIO register offset address.
 * @param value The 16-bits value to put.
 * @return None.
 */
static void sdio_putreg16(uint32_t base, uint32_t offset, uint16_t value)
{
    putreg16(value, base + offset);
}

/**
 * @brief Set bit or bits in the SDIO register.
 *
 * @param base The SDIO register base address.
 * @param offset The SDIO register offset address.
 * @param bitmask The bit mask.
 * @return None.
 */
static void sdio_reg_bit_set(uint32_t reg, uint32_t offset, uint32_t bitmask)
{
    uint32_t regvalue = sdio_getreg(reg, offset);

    regvalue |= bitmask;
    sdio_putreg(reg, offset, regvalue);
}

/**
 * @brief Clean bit or bits in the SDIO register.
 *
 * @param base The SDIO register base address.
 * @param offset The SDIO register offset address.
 * @param bitmask The bit mask.
 * @return None.
 */
static void sdio_reg_bit_clr(uint32_t reg, uint32_t offset, uint32_t bitmask)
{
    uint32_t regvalue = sdio_getreg(reg, offset);

    regvalue &= ~bitmask;
    sdio_putreg(reg, offset, regvalue);
}

/**
 * @brief Set field value in the SDIO register.
 *
 * @param base The SDIO register base address.
 * @param offset The SDIO register offset address.
 * @param bitmask The bit mask.
 * @param val The value to set.
 * @return None.
 */
static void sdio_reg_field_set(uint32_t reg, uint32_t offset, uint32_t bitmask,
                               uint32_t val)
{
    uint32_t regvalue = sdio_getreg(reg, offset);

    regvalue &= ~bitmask;
    regvalue |= (val & bitmask);
    sdio_putreg(reg, offset, regvalue);
}

#ifdef DEBUG_ON
/**
 * @brief Dump SDIO registers.
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdio_dump_registers(struct tsb_sdio_info *info)
{
    lldbg("SDMASYSADDR:                  0x%08x\n",
          sdio_getreg(info->sdio_reg_base, SDMASYSADDR));
    lldbg("BLK_SIZE_COUNT:               0x%08x\n",
          sdio_getreg(info->sdio_reg_base, BLK_SIZE_COUNT));
    lldbg("ARGUMENT1:                    0x%08x\n",
          sdio_getreg(info->sdio_reg_base, ARGUMENT1));
    lldbg("CMD_TRANSFERMODE:             0x%08x\n",
          sdio_getreg(info->sdio_reg_base, CMD_TRANSFERMODE));
    lldbg("RESPONSEREG0:                 0x%08x\n",
          sdio_getreg(info->sdio_reg_base, RESPONSEREG0));
    lldbg("RESPONSEREG1:                 0x%08x\n",
          sdio_getreg(info->sdio_reg_base, RESPONSEREG1));
    lldbg("RESPONSEREG2:                 0x%08x\n",
          sdio_getreg(info->sdio_reg_base, RESPONSEREG2));
    lldbg("RESPONSEREG3:                 0x%08x\n",
          sdio_getreg(info->sdio_reg_base, RESPONSEREG3));
    lldbg("DATAPORTREG:                  0x%08x\n",
          sdio_getreg(info->sdio_reg_base, DATAPORTREG));
    lldbg("PRESENTSTATE:                 0x%08x\n",
          sdio_getreg(info->sdio_reg_base, PRESENTSTATE));
    lldbg("HOST_PWR_BLKGAP_WAKEUP_CNTRL: 0x%08x\n",
          sdio_getreg(info->sdio_reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL));
    lldbg("CLOCK_SWRST_TIMEOUT_CONTROL:  0x%08x\n",
          sdio_getreg(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL));
    lldbg("INT_ERR_STATUS:               0x%08x\n",
          sdio_getreg(info->sdio_reg_base, INT_ERR_STATUS));
    lldbg("INT_ERR_STATUS_EN:            0x%08x\n",
          sdio_getreg(info->sdio_reg_base, INT_ERR_STATUS_EN));
    lldbg("INT_ERR_SIGNAL_EN:            0x%08x\n",
          sdio_getreg(info->sdio_reg_base, INT_ERR_SIGNAL_EN));
    lldbg("AUTOCMD12ERST_HOST_CTRL2:     0x%08x\n",
          sdio_getreg(info->sdio_reg_base, AUTOCMD12ERST_HOST_CTRL2));
    lldbg("CAPABILITIES_LOW:             0x%08x\n",
          sdio_getreg(info->sdio_reg_base, CAPABILITIES_LOW));
    lldbg("CAPABILITIES_HI:              0x%08x\n",
          sdio_getreg(info->sdio_reg_base, CAPABILITIES_HI));
    lldbg("MAXCURCAPAB:                  0x%08x\n",
          sdio_getreg(info->sdio_reg_base, MAXCURCAPAB));
    lldbg("FRCAUTOCMD_ERR_ERRINT_ST:     0x%08x\n",
          sdio_getreg(info->sdio_reg_base, FRCAUTOCMD_ERR_ERRINT_ST));
    lldbg("ADMAERSTATUS:                 0x%08x\n",
          sdio_getreg(info->sdio_reg_base, ADMAERSTATUS));
    lldbg("ADMASYSADDR32:                0x%08x\n",
          sdio_getreg(info->sdio_reg_base, ADMASYSADDR32));
    lldbg("ADMASYSADDR64:                0x%08x\n",
          sdio_getreg(info->sdio_reg_base, ADMASYSADDR64));
    lldbg("PRESETVAL_INIT_DFLT:          0x%08x\n",
          sdio_getreg(info->sdio_reg_base, PRESETVAL_INIT_DFLT));
    lldbg("PRESETVAL_HS_SDR12:           0x%08x\n",
          sdio_getreg(info->sdio_reg_base, PRESETVAL_HS_SDR12));
    lldbg("PRESETVAL_SDR25_SDR50:        0x%08x\n",
          sdio_getreg(info->sdio_reg_base, PRESETVAL_SDR25_SDR50));
    lldbg("PRESETVAL_SDR104_DDR50:       0x%08x\n",
          sdio_getreg(info->sdio_reg_base, PRESETVAL_SDR104_DDR50));
    lldbg("BOOTTIMCTRL:                  0x%08x\n",
          sdio_getreg(info->sdio_reg_base, BOOTTIMCTRL));
    lldbg("SLOTINTSTATUS_HSTCTRLVERSION: 0x%08x\n",
          sdio_getreg(info->sdio_reg_base, SLOTINTSTATUS_HSTCTRLVERSION));
}

/**
 * @brief Dump SDIO interrupt status registers.
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdio_dump_int_status_registers(struct tsb_sdio_info *info)
{
    uint32_t int_status = sdio_getreg(info->sdio_reg_base, INT_ERR_STATUS);

    lldbg("COMMAND_COMPLETE:         %s\n",
          (int_status & COMMAND_COMPLETE) ? "YES" : "NO");
    lldbg("TRANSFER_COMPLETE:        %s\n",
          (int_status & TRANSFER_COMPLETE) ? "YES" : "NO");
    lldbg("BLOCKGAP_EVENT:           %s\n",
          (int_status & BLOCKGAP_EVENT) ? "YES" : "NO");
    lldbg("DMA_INTERRUPT:            %s\n",
          (int_status & DMA_INTERRUPT) ? "YES" : "NO");
    lldbg("BUFFER_WRITE_RDY:         %s\n",
          (int_status & BUFFER_WRITE_RDY) ? "YES" : "NO");
    lldbg("BUFFER_READ_RDY:          %s\n",
          (int_status & BUFFER_READ_RDY) ? "YES" : "NO");
    lldbg("CARD_INSERTION:           %s\n",
          (int_status & CARD_INSERTION) ? "YES" : "NO");
    lldbg("CARD_REMOVAL:             %s\n",
          (int_status & CARD_REMOVAL) ? "YES" : "NO");
    lldbg("CARD_INTERRUPT:           %s\n",
          (int_status & CARD_INTERRUPT) ? "YES" : "NO");
    lldbg("INT_A:                    %s\n",
          (int_status & INT_A) ? "YES" : "NO");
    lldbg("INT_B:                    %s\n",
          (int_status & INT_B) ? "YES" : "NO");
    lldbg("INT_C:                    %s\n",
          (int_status & INT_C) ? "YES" : "NO");
    lldbg("RETUNING_EVENT:           %s\n",
          (int_status & RETUNING_EVENT) ? "YES" : "NO");
    lldbg("BOOT_ACK_RCV:             %s\n",
          (int_status & BOOT_ACK_RCV) ? "YES" : "NO");
    lldbg("BOOT_TERMINATE_INTERRUPT: %s\n",
          (int_status & BOOT_TERMINATE_INTERRUPT) ? "YES" : "NO");
    lldbg("ERROR_INTERRUPT:          %s\n",
          (int_status & ERROR_INTERRUPT) ? "YES" : "NO");
}

/**
 * @brief Dump SDIO interrupt error status registers.
 *
 * @param int_err_status The SDIO interrupt and error status.
 * @return None.
 */
static void sdio_dump_int_err_status_registers(uint32_t int_err_status)
{
    lldbg("CMD_TIMEOUT_ERROR:     %s\n",
          (int_err_status & CMD_TIMEOUT_ERROR) ? "YES" : "NO");
    lldbg("CMD_CRC_ERROR:         %s\n",
          (int_err_status & CMD_CRC_ERROR) ? "YES" : "NO");
    lldbg("CMD_END_BIT_ERROR:     %s\n",
          (int_err_status & CMD_END_BIT_ERROR) ? "YES" : "NO");
    lldbg("CMD_INDEX_ERROR:       %s\n",
          (int_err_status & CMD_INDEX_ERROR) ? "YES" : "NO");
    lldbg("DATA_TIMEOUT_ERROR:    %s\n",
          (int_err_status & DATA_TIMEOUT_ERROR) ? "YES" : "NO");
    lldbg("DATA_CRC_ERROR:        %s\n",
          (int_err_status & DATA_CRC_ERROR) ? "YES" : "NO");
    lldbg("DATA_END_BIT_ERROR:    %s\n",
          (int_err_status & DATA_END_BIT_ERROR) ? "YES" : "NO");
    lldbg("CURRENT_LIMIT_ERROR:   %s\n",
          (int_err_status & CURRENT_LIMIT_ERROR) ? "YES" : "NO");
    lldbg("AUTO_CMD_ERROR:        %s\n",
          (int_err_status & AUTO_CMD_ERROR) ? "YES" : "NO");
    lldbg("ADMA_ERROR:            %s\n",
          (int_err_status & ADMA_ERROR) ? "YES" : "NO");
    lldbg("TARGET_RESPONSE_ERROR: %s\n",
          (int_err_status & TARGET_RESPONSE_ERROR) ? "YES" : "NO");
}
#endif

/**
 * @brief Retrieve SDIO resource from driver core.
 *
 * This function gets the SDIO register base and irq number from driver core
 * infrastructure.
 *
 * @param dev The pointer to device structure.
 * @param info The SDIO driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdio_extract_resources(struct device *dev,
                                  struct tsb_sdio_info *info)
{
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS,
                                    "sdio_reg_base");
    if (!r) {
        return -EINVAL;
    }
    info->sdio_reg_base = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_IRQ, "irq_sdio");
    if (!r) {
        return -EINVAL;
    }
    info->sdio_irq = (int)r->start;

    return 0;
}

/**
 * @brief Calculate divisor for setting SD clock.
 *
 * The frequency of SDCLK is set by the following formula:
 * Clock Frequency = (Base Clock) / divisor
 * Thus, choose the smallest possible divisor which results in a clock frequency
 * that is less than or equal to the target frequency.
 *
 * @param ios Pointer to structure of ios.
 * @param base_clock Indicates the base (maximum) clock frequency for the SD
 *                   clock.
 * @return Value of SDIO clock divisor.
 */
static uint32_t sdio_calc_divisor(struct sdio_ios *ios, uint32_t base_clock)
{
    uint32_t divisor = 0;
    uint32_t sdio_base_clock = base_clock * MHZ_DEFINE; /* To MHz */

    /* Clock Frequency = (Base Clock) / divisor */
    divisor = sdio_base_clock / ios->clock;
    if ((sdio_base_clock / divisor) > ios->clock) {
        divisor++;
    }

    return divisor;
}

/**
 * @brief SD clock supply.
 *
 * This function is for supplying SD clock to a SD card. The clock shall be
 * supplied to the card before either of the following actions is taken.
 * 1. Issuing a SD command
 * 2. Detect an interrupt from a SD card in 4-bit mode.
 *
 * @param ios Pointer to structure of ios.
 * @param info The SDIO driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdio_clock_supply(struct sdio_ios *ios, struct tsb_sdio_info *info)
{
    uint32_t caps_low = 0, tsb_base_clock = 0, divisor = 0;
    uint8_t retry = 0;

    sdio_reg_bit_clr(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     SD_CLOCK_ENABLE);

    /*
     * Calculate a divisor for SD clock frequency. Base clock frequency of
     * Toshiba host controller is 192MHz (0xC0)
     */
    caps_low = sdio_getreg(info->sdio_reg_base, CAPABILITIES_LOW);
    tsb_base_clock = (caps_low & BASE_CLOCK_FREQ_FOR_SD_CLOCK_MASK) >>
                      BASE_CLOCK_FREQ_FOR_SD_CLOCK_SHIFT;
    divisor = sdio_calc_divisor(ios, tsb_base_clock);

    if (divisor == 0) {
        /* Use Toshiba 192MHz base clock */
        sdio_reg_bit_clr(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SDCLK_FREQ_SELECT_MASK);
    } else {
        /* SDCLK_FREQ_SELECT register is set by (divisor / 2) */
        sdio_reg_field_set(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                           SDCLK_FREQ_SELECT_MASK,
                           (divisor / 2) << SDCLK_FREQ_SELECT_SHIFT);
    }
    sdio_reg_bit_set(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     INTERNAL_CLOCK_EN);

    /* Waiting for INTERNAL_CLOCK_STABLE register stable */
    while (!(sdio_getreg(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL) &
             INTERNAL_CLOCK_STABLE) && (retry < REGISTER_MAX_RETRY)) {
        usleep(REGISTER_INTERVAL);
        retry++;
    }

    if (retry == REGISTER_MAX_RETRY) { /* Detect timeout */
        return -EINVAL;
    }

    sdio_reg_bit_set(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     SD_CLOCK_ENABLE);

    return 0;
}

/**
 * @brief Enable SD bus power.
 *
 * This function follows the sequence below to enable SD bus power.
 * 1. Switch the pin share mode and pin high for GPB2_SD_POWER_EN pin
 * 2. Pull up clk and SDIO data interface pins
 * 3. Set SD Bus Voltage Select register with supported maximum voltage
 * 4. Set SD Bus Power register
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdio_enable_bus_power(struct tsb_sdio_info *info)
{
    uint32_t caps_low = sdio_getreg(info->sdio_reg_base, CAPABILITIES_LOW);
    uint32_t voltage_support = 0;

    /* Pull up clk and SDIO data interface pins */
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO3CTRL, SDCMD_PUENABLE);
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO4CTRL, SDDATA3_PUENABLE);
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO5CTRL, SDDATA2_PUENABLE);
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO6CTRL, SDDATA1_PUENABLE);
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_IO7CTRL, SDDATA0_PUENABLE);

    /*
     * Get the support voltage of the Host Controller and set SD Bus Voltage
     * Select with supported maximum voltage
     */
    if (caps_low & VOLTAGE_SUPPORT_3_3V) {
        voltage_support = SUPPORT_3_3V;
    } else if (caps_low & VOLTAGE_SUPPORT_3_0V) {
        voltage_support = SUPPORT_3_0V;
    } else {
        voltage_support = SUPPORT_1_8V;
    }
    sdio_reg_field_set(info->sdio_reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                       SD_BUS_VOLTAGE_SELECT_MASK,
                       voltage_support << SD_BUS_VOLTAGE_SELECT_SHIFT);

    sdio_reg_bit_set(info->sdio_reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                     SD_BUS_POWER);
}

/**
 * @brief Disable SD bus power.
 *
 * This function follows the sequence below to enable SD bus power.
 * 1. Switch the pin share mode and pin low for GPB2_SD_POWER_EN pin
 * 2. Pull down clk and SDIO data interface pins
 * 3. Set SD Bus Voltage Select register with supported maximum voltage
 * 4. Clean SD Bus Power register
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdio_disable_bus_power(struct tsb_sdio_info *info)
{
    uint32_t caps_low = sdio_getreg(info->sdio_reg_base, CAPABILITIES_LOW);
    uint32_t voltage_support = 0;

    /* Pull down clk and SDIO data interface pins */
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO3CTRL, SDCMD_PUENABLE);
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO4CTRL, SDDATA3_PUENABLE);
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO5CTRL, SDDATA2_PUENABLE);
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO6CTRL, SDDATA1_PUENABLE);
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_IO7CTRL, SDDATA0_PUENABLE);

    /*
     * Get the support voltage of the Host Controller and set SD Bus Voltage
     * Select with supported maximum voltage
     */
    if (caps_low & VOLTAGE_SUPPORT_3_3V) {
        voltage_support = SUPPORT_3_3V;
    } else if (caps_low & VOLTAGE_SUPPORT_3_0V) {
        voltage_support = SUPPORT_3_0V;
    } else {
        voltage_support = SUPPORT_1_8V;
    }
    sdio_reg_field_set(info->sdio_reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                       SD_BUS_VOLTAGE_SELECT_MASK,
                       voltage_support << SD_BUS_VOLTAGE_SELECT_SHIFT);

    sdio_reg_bit_clr(info->sdio_reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                     SD_BUS_POWER);
}

/**
 * @brief Change SD bus width.
 *
 * This function follows the sequence below to enable SD bus power.
 * 1. Disable Card Interrupt in host
 * 2. Change bit mode for host
 * 3. Enable Card Interrupt in host
 *
 * @param ios Pointer to structure of ios.
 * @param info The SDIO driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdio_change_bus_width(struct sdio_ios *ios,
                                 struct tsb_sdio_info *info)
{
    /* Disable Card Interrupt in host */
    sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_STATUS_EN,
                     CARD_INTERRUPT_STAT_EN);
    sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                     CARD_INTERRUPT_EN);

    /* Change bit mode for host */
    switch (ios->bus_width) {
    case HC_SDIO_BUS_WIDTH_1:
        sdio_reg_bit_clr(info->sdio_reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                         DATA_TRASFER_WIDTH);
        break;
    case HC_SDIO_BUS_WIDTH_4:
    case HC_SDIO_BUS_WIDTH_8:
        sdio_reg_bit_set(info->sdio_reg_base, HOST_PWR_BLKGAP_WAKEUP_CNTRL,
                         DATA_TRASFER_WIDTH);
        break;
    default:
        return -EINVAL;
    }

    /* Enable Card Interrupt in host */
    sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS_EN,
                     CARD_INTERRUPT_STAT_EN);
    sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                     CARD_INTERRUPT_EN);

    return 0;
}

/**
 * @brief Prepare command.
 *
 * @param dev Pointer to structure of device.
 * @param cmd Pointer to structure of cmd.
 * @return None.
 */
static void sdio_prepare_command(struct device *dev, struct sdio_cmd *cmd)
{
    struct tsb_sdio_info *info = device_get_private(dev);

    if (cmd->data_blocks && cmd->data_blksz) {
        info->data_cmd = true;
        info->blksz = cmd->data_blksz;
        info->blocks = cmd->data_blocks;
        if (cmd->cmd == HC_MMC_SWITCH) { /* CMD6 */
            info->data_flags = HC_GB_SDIO_DATA_READ;
        } else if (cmd->cmd == 8) { /* CMD8 */
            info->data_flags = HC_GB_SDIO_DATA_READ;
        } else if (cmd->cmd == HC_MMC_READ_SINGLE_BLOCK) { /* CMD17 */
            info->data_flags = HC_GB_SDIO_DATA_READ;
        } else if (cmd->cmd == HC_MMC_READ_MULTIPLE_BLOCK) { /* CMD18 */
            info->data_flags = HC_GB_SDIO_DATA_READ;
        } else if (cmd->cmd == HC_MMC_WRITE_BLOCK) { /* CMD24 */
            info->data_flags = HC_GB_SDIO_DATA_WRITE;
        } else if (cmd->cmd == HC_MMC_WRITE_MULTIPLE_BLOCK) { /* CMD25 */
            info->data_flags = HC_GB_SDIO_DATA_WRITE;
        } else if (cmd->cmd == HC_SD_APP_SEND_SCR) { /* CMD51 */
            info->data_flags = HC_GB_SDIO_DATA_READ;
        } else if (cmd->cmd == HC_SD_APP_SD_STATUS) { /* ACMD13 */
            info->data_flags = HC_GB_SDIO_DATA_READ;
        }
    } else {
        info->data_cmd = false;
        info->blksz = 0;
        info->blocks = 0;
        info->data_flags = 0;
    }
}

/**
 * @brief Set transfer mode.
 *
 * @param dev Pointer to structure of device.
 * @param cmd Pointer to structure of cmd.
 * @return None.
 */
static void sdio_set_transfer_mode(struct device *dev, struct sdio_cmd *cmd)
{
    struct tsb_sdio_info *info = device_get_private(dev);
    uint16_t mode = 0;

    if (cmd->cmd == HC_MMC_READ_MULTIPLE_BLOCK ||
        cmd->cmd == HC_MMC_WRITE_MULTIPLE_BLOCK || info->blocks > 1) {
        mode = MULTI_SINGLE_BLOCK_SELECT | BLOCK_COUNT_ENABLE;
    }

    if (info->data_flags & HC_GB_SDIO_DATA_READ) {
        mode |= TRANSFER_DIRECTION_SELECT;
    }

    sdio_putreg16(info->sdio_reg_base, CMD_TRANSFERMODE, mode);
}

/**
 * @brief Recover error interrupt.
 *
 * This function follows the sequence below to recover error interrupt.
 * 1. Disable Error Interrupt Signal
 * 2. Check Error Interrupt Status for command
 * 3. Set Software Reset For CMD line (CR) if CMD Line Error occurs
 * 4. Check Error Interrupt Status for data
 * 5. Set Software Reset For DAT line (DR) if DAT Line Error occurs
 * 6. Clean previous error status
 *
 * @param info The SDIO driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdio_error_interrupt_recovery(struct tsb_sdio_info *info)
{
    uint32_t cmd_line_err = CMD_INDEX_ERROR | CMD_END_BIT_ERROR |
                            CMD_CRC_ERROR | CMD_TIMEOUT_ERROR;
    uint32_t dat_line_err = DATA_END_BIT_ERROR | DATA_CRC_ERROR |
                            DATA_TIMEOUT_ERROR;
    uint8_t retry = 0;

    /* Disable Error Interrupt Signal */
    sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_STATUS_EN,
                     DATA_END_BIT_ERR_STAT_EN | DATA_CRC_ERR_STAT_EN |
                     DATA_TIMEOUT_ERR_STAT_EN | CMD_INDEX_ERR_STAT_EN |
                     CMD_END_BIT_ERR_STAT_EN | CMD_CRC_ERR_STAT_EN |
                     CMD_TIMEOUT_ERR_STAT_EN);
    sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                     DATA_END_BIT_ERR_EN | DATA_CRC_ERR_EN |
                     DATA_TIMEOUT_ERR_EN | CMD_INDEX_ERR_EN |
                     CMD_END_BIT_ERR_EN | CMD_CRC_ERR_EN |
                     CMD_TIMEOUT_ERR_EN);

    /* Check Error Interrupt Status for command */
    if (info->err_status & cmd_line_err) { /* CMD Line Error occurs */
        /* Set Software Reset For CMD line (CR) */
        sdio_reg_bit_set(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SW_RESET_CMD_LINE);

        /* Check CR */
        while ((sdio_getreg(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL) &
                 SW_RESET_CMD_LINE) && (retry < REGISTER_MAX_RETRY)) {
            retry++;
        }

        if (retry == REGISTER_MAX_RETRY) { /* Detect timeout */
            return -EINVAL;
        }
    }

    /* Check Error Interrupt Status for data */
    if (info->err_status & dat_line_err) { /* DAT Line Error occurs */
        /* Set Software Reset For DAT line (DR) */
        sdio_reg_bit_set(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SW_RESET_DAT_LINE);

        /* Check DR */
        while ((sdio_getreg(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL) &
                 SW_RESET_DAT_LINE) && (retry < REGISTER_MAX_RETRY)) {
            retry++;
        }

        if (retry == REGISTER_MAX_RETRY) { /* Detect timeout */
            return -EINVAL;
        }
    }

    /* Clean previous error status */
    sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_STATUS,
                     DATA_END_BIT_ERROR | DATA_CRC_ERROR |
                     DATA_TIMEOUT_ERROR | CMD_INDEX_ERROR |
                     CMD_END_BIT_ERROR | CMD_CRC_ERROR |
                     CMD_TIMEOUT_ERROR);

    return 0;
}

/**
 * @brief Software reset.
 *
 * This function sets software reset register for SDIO CMD line and DAT line.
 *
 * @param info The SDIO driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdio_software_reset(struct tsb_sdio_info *info)
{
    uint8_t retry = 0;

    /* Set Software Reset For CMD line (CR) */
    sdio_reg_bit_set(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     SW_RESET_CMD_LINE);

    /* Check CR */
    while ((sdio_getreg(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL) &
             SW_RESET_CMD_LINE) && (retry < REGISTER_MAX_RETRY)) {
        usleep(REGISTER_INTERVAL);
        retry++;
    }

    if (retry == REGISTER_MAX_RETRY) { /* Detect timeout */
        return -ETIMEDOUT;
    }

    /* Set Software Reset For DAT line (DR) */
    sdio_reg_bit_set(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     SW_RESET_DAT_LINE);

    /* Check DR */
    while ((sdio_getreg(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL) &
             SW_RESET_DAT_LINE) && (retry < REGISTER_MAX_RETRY)) {
        usleep(REGISTER_INTERVAL);
        retry++;
    }

    if (retry == REGISTER_MAX_RETRY) { /* Detect timeout */
        return -ETIMEDOUT;
    }

    return 0;
}

/**
 * @brief SDIO device interrupt routing
 *
 * Set SD host controller configuration for card insert and remove. The
 * configuration includes command interrupts, SD bus power and SD clock. It also
 * inform SDIO protocol driver the card event status.
 *
 * @param irq Number of irq.
 * @param context Pointer to structure of device data.
 * @return 0 on success, negative errno on error.
 */
int sdio_irq_event(int irq, void *context)
{
    struct tsb_sdio_info *info = device_get_private(sdio_dev);
    uint8_t value = 0;

    gpio_mask_irq(irq);
    value = gpio_get_value(info->cd_pin_number);

    if (value) { /* Card remove */
        /* Disable command interrupts */
        sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_STATUS_EN,
                         CMD_TIMEOUT_ERR_STAT_EN | CMD_COMPLETE_STAT_EN);
        sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                         CMD_TIMEOUT_ERR_EN | CMD_COMPLETE_EN);
        /* Disable data error interrupt */
        sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_STATUS_EN,
                         DATA_TIMEOUT_ERR_STAT_EN);
        sdio_disable_bus_power(info);
        sdio_reg_bit_clr(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SD_CLOCK_ENABLE);
        info->card_event = HC_SDIO_CARD_REMOVED;
        /* Recover error interrupt if have */
        sdio_error_interrupt_recovery(info);
    } else { /* Card insert */
        /* Recover error interrupt if have */
        sdio_error_interrupt_recovery(info);
        /* Enable command interrupts */
        sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS_EN,
                         CMD_TIMEOUT_ERR_STAT_EN | CMD_COMPLETE_STAT_EN);
        sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                         CMD_TIMEOUT_ERR_EN | CMD_COMPLETE_EN);
        /* Enable data error interrupt */
        sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS_EN,
                         DATA_TIMEOUT_ERR_STAT_EN);
        sdio_enable_bus_power(info);
        sdio_reg_bit_set(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SD_CLOCK_ENABLE);
        info->card_event = HC_SDIO_CARD_INSERTED;
    }

    if (info->pre_card_event != info->card_event) {
        if (info->callback) {
            info->callback(info->card_event);
        }
        info->pre_card_event = info->card_event;
    }

    gpio_unmask_irq(irq);

    return 0;
}

/**
 * @brief SDIO interrupt handler.
 *
 * This function is attached as interrupt service when driver init.
 *
 * @param irq The IRQ number from OS.
 * @param context The context for this interrupt.
 * @return None.
 */
static int sdio_irq_handler(int irq, void *context)
{
    struct tsb_sdio_info *info = device_get_private(sdio_dev);
    uint32_t timeout_error = DATA_TIMEOUT_ERROR | CMD_TIMEOUT_ERROR;
    uint32_t int_err_status = 0;
    uint32_t resp_temp[4] = {0};

    int_err_status = sdio_getreg(info->sdio_reg_base, INT_ERR_STATUS);
    info->err_status = int_err_status;
    info->sdio_int_err_status = 0;

    if (int_err_status & ERROR_INTERRUPT) {
        /* Check error type for the return of command */
        if (int_err_status & timeout_error) {
            info->sdio_int_err_status = -ETIMEDOUT;
        } else {
            info->sdio_int_err_status = -EINVAL;
        }

        /* Clean the response of command if command timeout error happened */
        if (int_err_status & CMD_TIMEOUT_ERROR) {
            sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS,
                             COMMAND_COMPLETE);
            sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS,
                             ERROR_INTERRUPT);
            info->resp[0] = 0;
            info->resp[1] = 0;
            info->resp[2] = 0;
            info->resp[3] = 0;
            sem_post(&info->cmd_sem);
            /* Recover error interrupt */
            sdio_error_interrupt_recovery(info);
        }

        if (int_err_status & DATA_TIMEOUT_ERROR) {
            info->data_timeout = true;
        }
    }

    if (int_err_status & CARD_INTERRUPT) {
        /* Disable Card Interrupt in Host */
        sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_STATUS_EN,
                         CARD_INTERRUPT_STAT_EN);
        sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                         CARD_INTERRUPT_EN);
    }

    if (int_err_status & COMMAND_COMPLETE) {
        sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS, COMMAND_COMPLETE);
        /* Get Response Data */
        resp_temp[0] = sdio_getreg(info->sdio_reg_base, RESPONSEREG0);
        resp_temp[1] = sdio_getreg(info->sdio_reg_base, RESPONSEREG1);
        resp_temp[2] = sdio_getreg(info->sdio_reg_base, RESPONSEREG2);
        resp_temp[3] = sdio_getreg(info->sdio_reg_base, RESPONSEREG3);
        /*
         * In the CMD line the Most Significant Bit (MSB) is transmitted first,
         * the Least Significant Bit (LSB) is the last.
         */
        if (info->cmd_flags == HC_SDIO_RSP_R2) {
            info->resp[0] = (resp_temp[3] << BYTE_SHIFT) |
                            ((resp_temp[2] & R2_RSP_MASK) >> (BYTE_SHIFT * 3));
            info->resp[1] = (resp_temp[2] << BYTE_SHIFT) |
                            ((resp_temp[1] & R2_RSP_MASK) >> (BYTE_SHIFT * 3));
            info->resp[2] = (resp_temp[1] << BYTE_SHIFT) |
                            ((resp_temp[0] & R2_RSP_MASK) >> (BYTE_SHIFT * 3));
            info->resp[3] = (resp_temp[0] << BYTE_SHIFT);
        } else {
            info->resp[0] = resp_temp[0];
            info->resp[1] = resp_temp[1];
            info->resp[2] = resp_temp[2];
            info->resp[3] = resp_temp[3];
        }
        sem_post(&info->cmd_sem);
    }

    return 0;
}

/**
 * @brief Write data from buffer to FIFO.
 *
 * This function put the data from buffer to FIFO until the buffer is
 * empty. If buffer is empty, it calls the up layer callback
 * function in case of non-blocking mode.
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdio_write_fifo_data(struct tsb_sdio_info *info)
{
    uint32_t presentstate = 0, buf_port = 0;
    uint32_t data_mask = BUFFER_READ_ENABLE | BUFFER_WRITE_ENABLE |
                         READ_TRANSFER_ACTIVE | WRITE_TRANSFER_ACTIVE |
                         DAT_LINE_ACTIVE | COMMAND_INHIBIT_DAT;
    int16_t remaining = 0, i = 0;
    uint8_t *wbuf = info->write_buf.buffer;

    presentstate = sdio_getreg(info->sdio_reg_base, PRESENTSTATE);
    while (presentstate & data_mask) {
        /* Check data timeout */
        if (info->data_timeout == true) {
            /* Recover error interrupt */
            sdio_error_interrupt_recovery(info);
            info->data_timeout = false;
            break;
        }

        /* Check data error */
        if (!(presentstate & BUFFER_WRITE_ENABLE) &&
            (presentstate & WRITE_TRANSFER_ACTIVE)) {
            break;
        }

        if (presentstate & BUFFER_WRITE_ENABLE) {
            sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS,
                             BUFFER_WRITE_RDY);

            /*
             * Is there at least a full uint32_t data remaining in the user
             * buffer?
             */
            remaining = info->write_buf.tail - info->write_buf.head;
            if (remaining >= sizeof(uint32_t)) {
                /* Yes. Write uint32_t data to the FIFO */
                buf_port = wbuf[info->write_buf.head] |
                           wbuf[info->write_buf.head + 1] << BYTE_SHIFT |
                           wbuf[info->write_buf.head + 2] << (BYTE_SHIFT * 2) |
                           wbuf[info->write_buf.head + 3] << (BYTE_SHIFT * 3);
                info->write_buf.head += sizeof(uint32_t);
            } else {
                /*
                 * No. Write the bytes remaining in the user buffer to the
                 * FIFO
                 */
                for (i = 0; i < remaining; i++) {
                    buf_port |= wbuf[info->write_buf.head + i] <<
                                (BYTE_SHIFT * i);
                }
                info->write_buf.head = info->write_buf.tail;
            }
            sdio_putreg(info->sdio_reg_base, DATAPORTREG, buf_port);

            /* More blocks? */
            if (info->write_buf.head == info->write_buf.tail) { /* No */
                if (info->write_callback) { /* Non-blocking */
                    info->flags &= ~SDIO_FLAG_WRITE;
                    info->write_callback(info->write_buf.head, info->blksz,
                                         info->write_buf.buffer, 0);
                }
            }
        }
        presentstate = sdio_getreg(info->sdio_reg_base, PRESENTSTATE);
    }
}

/**
 * @brief Read data from FIFO to buffer.
 *
 * This function put the data from buffer to FIFO until the buffer is
 * empty. If buffer is empty, it calls the up layer callback
 * function in case of non-blocking mode.
 *
 * @param info The SDIO driver information.
 * @return None.
 */
static void sdio_read_fifo_data(struct tsb_sdio_info *info)
{
    uint32_t presentstate = 0, buf_port = 0;
    uint32_t data_mask = BUFFER_READ_ENABLE | BUFFER_WRITE_ENABLE |
                         READ_TRANSFER_ACTIVE | WRITE_TRANSFER_ACTIVE |
                         DAT_LINE_ACTIVE | COMMAND_INHIBIT_DAT;
    int16_t remaining = 0, i = 0;
    uint8_t *rbuf = info->read_buf.buffer;

    presentstate = sdio_getreg(info->sdio_reg_base, PRESENTSTATE);
    while (presentstate & data_mask) {
        /* Check data timeout */
        if (info->data_timeout == true) {
            /* Recover error interrupt */
            sdio_error_interrupt_recovery(info);
            info->data_timeout = false;
            break;
        }

        /* Check data error */
        if (!(presentstate & BUFFER_READ_ENABLE) &&
            (presentstate & READ_TRANSFER_ACTIVE)) {
            break;
        }

        if (presentstate & BUFFER_READ_ENABLE) {
            sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS,
                             BUFFER_READ_RDY);

            /*
             * Is there at least a full uint32_t data remaining in the user
             * buffer?
             */
            remaining = info->read_buf.tail - info->read_buf.head;
            buf_port = sdio_getreg(info->sdio_reg_base, DATAPORTREG);
            if (remaining >= sizeof(uint32_t)) {
                /* Yes. Transfer uint32_t data in FIFO to the user buffer */
                rbuf[info->read_buf.head] = (uint8_t)buf_port;
                rbuf[info->read_buf.head + 1] =
                                        (uint8_t)(buf_port >> (BYTE_SHIFT * 1));
                rbuf[info->read_buf.head + 2] =
                                        (uint8_t)(buf_port >> (BYTE_SHIFT * 2));
                rbuf[info->read_buf.head + 3] =
                                        (uint8_t)(buf_port >> (BYTE_SHIFT * 3));
                info->read_buf.head += sizeof(uint32_t);
            } else {
                /*
                 * No. Transfer the bytes remaining in FIFO to the user
                 * buffer
                 */
                for (i = 0; i < remaining; i++) {
                    rbuf[info->read_buf.head + i] =
                                        (uint8_t)(buf_port >> (BYTE_SHIFT * i));
                }
                info->read_buf.head = info->read_buf.tail;
            }

            /* More blocks? */
            if (info->read_buf.head == info->read_buf.tail) { /* No */
                if (info->read_callback) { /* Non-blocking */
                    info->flags &= ~SDIO_FLAG_READ;
                    info->read_callback(info->read_buf.head, info->blksz,
                                        info->read_buf.buffer, 0);
                }
            }
        }
        presentstate = sdio_getreg(info->sdio_reg_base, PRESENTSTATE);
    }
}

/**
 * @brief SDIO write data thread
 *
 * This is the thread for non-blocking data write.
 *
 * @return None.
 */
static void *sdio_write_data_thread(void *data)
{
    struct tsb_sdio_info *info = data;

    while (1) {
        sem_wait(&info->write_sem);
        if (info->thread_abort) {
            /* Exit sdio_write_data_thread loop */
            break;
        }
        sdio_write_fifo_data(info);
    }

    return NULL;
}

/**
 * @brief SDIO read data thread
 *
 * This is the thread for non-blocking data read.
 *
 * @return None.
 */
static void *sdio_read_data_thread(void *data)
{
    struct tsb_sdio_info *info = data;

    while (1) {
        sem_wait(&info->read_sem);
        if (info->thread_abort) {
            /* Exit sdio_read_data_thread loop */
            break;
        }
        sdio_read_fifo_data(info);
    }

    return NULL;
}

/**
 * @brief Get capabilities of SD host controller
 *
 * This function is to get capabilities of SD host controller. The
 * capabilities include:
 * 1. Unremovable support
 * 2. 4 bit transfers support
 * 3. 8 bit transfers support
 * 4. MMC high-speed timings support
 * 5. SD high-speed timings support
 * 6. Erase and trim commands support
 * 7. DDR mode at 1.2V support
 * 8. DDR mode at 1.8V support
 * 9. Power off card support
 * 10. UHS SDR12 mode support
 * 11. UHS SDR25 mode support
 * 12. UHS SDR50 mode support
 * 13. UHS SDR104 mode support
 * 14. UHS DDR50 mode support
 * 15. Driver Type A support
 * 16. Driver Type C support
 * 17. Driver Type D support
 * 18. HS200 mode at 1.2V support
 * 19. HS200 mode at 1.8V support
 * 20. HS400 mode at 1.2V support
 * 21. HS400 mode at 1.8V support
 * 22. SD VDD voltage range
 * 23. Minimum frequency support
 * 24. Maximum frequency support
 * 25. Maximum number of blocks per data command transfer
 * 26. Maximum size of each block to transfer
 *
 * @param dev Pointer to structure of device.
 * @param cap Pointer to structure of capabilities.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_get_capability(struct device *dev, struct sdio_cap *cap)
{
    /*
     * Not support for HC_SDIO_CAP_NONREMOVABLE, HC_SDIO_CAP_8_BIT_DATA,
     * HC_SDIO_CAP_1_2V_DDR, HC_SDIO_CAP_1_8V_DDR, HC_SDIO_CAP_UHS_SDR50,
     * HC_SDIO_CAP_UHS_SDR104, HC_SDIO_CAP_UHS_DDR50, HC_SDIO_CAP_HS200_1_2V,
     * HC_SDIO_CAP_HS200_1_8V, HC_SDIO_CAP_HS400_1_2V, HC_SDIO_CAP_HS400_1_8V,
     * HC_SDIO_CAP_ERASE
     */
    cap->caps = HC_SDIO_CAP_4_BIT_DATA | HC_SDIO_CAP_MMC_HS |
                HC_SDIO_CAP_SD_HS | HC_SDIO_CAP_POWER_OFF_CARD |
                HC_SDIO_CAP_UHS_SDR12 | HC_SDIO_CAP_UHS_SDR25 |
                HC_SDIO_CAP_DRIVER_TYPE_A | HC_SDIO_CAP_DRIVER_TYPE_C |
                HC_SDIO_CAP_DRIVER_TYPE_D;
    cap->ocr = HC_SDIO_VDD_33_34;
    cap->f_min = MIN_FREQ_SUPPORT;
    cap->f_max = MAX_FREQ_SUPPORT;
    cap->max_blk_count = MAX_BLK_COUNT;
    cap->max_blk_size = MAX_BLK_SIZE;

    return 0;
}

/**
 * @brief Set ios of SD host controller
 *
 * This function is to set the parameters to configure the SD controller. The
 * parameters include:
 * 1. Clock rate in Hz
 * 2. Voltage range
 * 3. Bus mode
 * 4. Power_mode
 * 5. Bus width
 * 6. Timing
 * 7. Signal voltage
 * 8. Driver type
 *
 * @param dev Pointer to structure of device.
 * @param ios Pointer to structure of ios.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_set_ios(struct device *dev, struct sdio_ios *ios)
{
    struct tsb_sdio_info *info = device_get_private(dev);

    uint32_t uhs_mode_sel = 0;

    /* Set clock */
    if (sdio_clock_supply(ios, info)) {
        return -EINVAL;
    }

    /* Set vdd */
    /* Not support to change vdd since board uses jumper to change power */

    /* set bus mode */
    /* Not support */

    /* Set power mode */
    switch (ios->power_mode) {
    case HC_SDIO_POWER_OFF:
        sdio_disable_bus_power(info);
        break;
    case HC_SDIO_POWER_UP:
    case HC_SDIO_POWER_ON:
        sdio_enable_bus_power(info);
        break;
    case HC_SDIO_POWER_UNDEFINED:
    default:
        return -EINVAL;
    }

    /* Set bus width */
    if (sdio_change_bus_width(ios, info)) {
        return -EINVAL;
    }

    /* Set timing */
    switch (ios->timing) {
    case HC_SDIO_TIMING_LEGACY:
    case HC_SDIO_TIMING_MMC_HS:
    case HC_SDIO_TIMING_SD_HS:
    case HC_SDIO_TIMING_UHS_SDR12:
        uhs_mode_sel = SDR12;
        break;
    case HC_SDIO_TIMING_UHS_SDR25:
        uhs_mode_sel = SDR25;
        break;
    case HC_SDIO_TIMING_UHS_SDR50:
        uhs_mode_sel = SDR50;
        break;
    case HC_SDIO_TIMING_UHS_SDR104:
        uhs_mode_sel = SDR104;
        break;
    case HC_SDIO_TIMING_UHS_DDR50:
        uhs_mode_sel = DDR50;
        break;
    case HC_SDIO_TIMING_MMC_DDR52:
    case HC_SDIO_TIMING_MMC_HS200:
    case HC_SDIO_TIMING_MMC_HS400:
    default:
        return -EINVAL;
    }
    sdio_reg_field_set(info->sdio_reg_base, AUTOCMD12ERST_HOST_CTRL2,
                       UHS_MODE_SEL_MASK,
                       uhs_mode_sel << UHS_MODE_SEL_SHIFT);

    /* Set signal voltage */
    /*
     * Not support to change signal voltage since board uses jumper to change
     * signal_voltage
     */

    /* Set driver type */
    switch (ios->drv_type) {
    case HC_SDIO_SET_DRIVER_TYPE_A:
        sdio_reg_bit_set(info->sdio_reg_base, CAPABILITIES_HI,
                         DRIVER_TYPEA_SUPPORT);
        break;
    case HC_SDIO_SET_DRIVER_TYPE_C:
        sdio_reg_bit_set(info->sdio_reg_base, CAPABILITIES_HI,
                         DRIVER_TYPEC_SUPPORT);
        break;
    case HC_SDIO_SET_DRIVER_TYPE_D:
        sdio_reg_bit_set(info->sdio_reg_base, CAPABILITIES_HI,
                         DRIVER_TYPED_SUPPORT);
        break;
    case HC_SDIO_SET_DRIVER_TYPE_B:
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

/**
 * @brief Send SDIO command through SDIO host controller
 *
 * This function follows the sequence to send command to card through SD host
 * controller.
 * 1. Check Command Inhibit (CMD) register
 * 2. Check that does issue the command with the busy
 * 3. Check that does issue Abort Command
 * 4. Check Command Inhibit (DAT) register
 * 5. Set Argument 1 Reg register
 * 6. Set Command Reg register
 * 7. Wait for Command Complete Int register
 *
 * @param dev Pointer to structure of device.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_send_cmd(struct device *dev, struct sdio_cmd *cmd)
{
    struct tsb_sdio_info *info = device_get_private(dev);
    uint8_t cmd_flags = 0;
    uint8_t retry = 0;

    /*
     * It needs to disable transfer complete interrupt in host for masking
     * incorrect interrupts that may occur while changing the bus width.
     */
    if (cmd->cmd == HC_SD_APP_SET_BUS_WIDTH || cmd->cmd == HC_MMC_LOCK_UNLOCK) {
        sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_STATUS_EN,
                         TRANSFER_COMPLETE_STAT_EN);
        sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                         TRANSFER_COMPLETE_EN);
    }

    info->cmd_flags = cmd->cmd_flags;

    sdio_prepare_command(dev, cmd);

    /*
     * Before SD core layer issues cmd18 (HC_MMC_READ_MULTIPLE_BLOCK) and cmd25
     * (HC_MMC_WRITE_MULTIPLE_BLOCK) for multiple block read and write, it
     * issues cmd23 (HC_MMC_SET_BLOCK_COUNT) to set block count of SD host
     * controller. No need to change it in card.
     */
    if (cmd->cmd == HC_MMC_SET_BLOCK_COUNT) {
        return 0;
    }

    /* Enable command interrupts */
    sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS_EN,
                     CMD_TIMEOUT_ERR_STAT_EN | CMD_COMPLETE_STAT_EN);
    sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                     CMD_TIMEOUT_ERR_EN |CMD_COMPLETE_EN);

    /* Wait for CMD Line unused */
    while ((sdio_getreg(info->sdio_reg_base, PRESENTSTATE) &
           COMMAND_INHIBIT_CMD) && (retry < REGISTER_MAX_RETRY)) {
        usleep(REGISTER_INTERVAL);
        retry++;
    }

    if (retry == REGISTER_MAX_RETRY) { /* Detect timeout */
        return -EINVAL;
    }

    /* Prepare data information */
    if (info->data_cmd) {
        sdio_putreg16(info->sdio_reg_base, BLK_SIZE_COUNT, info->blksz);
        sdio_putreg16(info->sdio_reg_base, BLK_SIZE_COUNT + REG_OFFSET,
                      info->blocks);
    }

    /* Issue the command with the busy? */
    if (cmd->cmd_flags == HC_SDIO_RSP_R1B) {
        /* Issue Abort Command? */
        if (cmd->cmd_type != HC_SDIO_CMD_BCR) {
            /* Wait for DAT Line unused */
            while ((sdio_getreg(info->sdio_reg_base, PRESENTSTATE) &
                   COMMAND_INHIBIT_DAT) && (retry < REGISTER_MAX_RETRY)) {
                usleep(REGISTER_INTERVAL);
                retry++;
            }

            if (retry == REGISTER_MAX_RETRY) { /* Detect timeout */
                return -EINVAL;
            }
        }
    }

    /* Set Argument 1 Reg register */
    sdio_putreg(info->sdio_reg_base, ARGUMENT1, cmd->cmd_arg);

    /* Set Transfer Mode Reg register */
    sdio_set_transfer_mode(dev, cmd);

    /* Set Command Reg register */
    switch (cmd->cmd_flags) {
    case HC_SDIO_RSP_NONE:
        cmd_flags = NO_RESPONSE;
        break;
    case HC_SDIO_RSP_R1_R5_R6_R7:
        cmd_flags = RESPONSE_LENGTH_48;
        break;
    case HC_SDIO_RSP_R1B:
        cmd_flags = RESPONSE_LENGTH_BUSY_48;
        break;
    case HC_SDIO_RSP_R2:
        cmd_flags = RESPONSE_LENGTH_136;
        break;
    case HC_SDIO_RSP_R3_R4:
        cmd_flags = RESPONSE_LENGTH_48;
        break;
    default:
        return -EINVAL;
    }

    if (cmd->cmd_flags & HC_SDIO_RSP_CRC) {
        cmd_flags |= COMMAND_CRC_CHECK_EN_DEF;
    }

    if (cmd->cmd_flags & HC_SDIO_RSP_OPCODE) {
        cmd_flags |= COMMAND_INDEX_CHECK_EN_DEF;
    }

    if (info->data_cmd) {
        cmd_flags |= DATA_PRESENT_DEF;
    }

    if (cmd->cmd == HC_MMC_STOP_TRANSMISSION) {
        cmd_flags |= COMMAND_TYPE_DEF;
    }

    sdio_putreg16(info->sdio_reg_base, CMD_TRANSFERMODE + REG_OFFSET,
                  SDIO_MAKE_CMD(cmd->cmd, cmd_flags));

    /* Wait for Command Complete Int register */
    sem_wait(&info->cmd_sem);
    usleep(COMMAND_INTERVAL);
    cmd->resp[0] = info->resp[0];
    cmd->resp[1] = info->resp[1];
    cmd->resp[2] = info->resp[2];
    cmd->resp[3] = info->resp[3];

    usleep(COMMAND_INTERVAL);
    if (cmd->cmd == HC_MMC_STOP_TRANSMISSION) {
        info->sdio_int_err_status = sdio_software_reset(info);
    }

    return info->sdio_int_err_status;
}

/**
 * @brief Write data to SD card
 *
 * This function is to write data to SD card through the SD host controller.
 * It could be blocking or non-blocking and through DMA or PIO mode.
 *
 * @param dev Pointer to structure of device.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_write(struct device *dev, struct sdio_transfer *transfer)
{
    struct tsb_sdio_info *info = device_get_private(dev);

    if (info->flags & SDIO_FLAG_WRITE) {
        return -EBUSY;
    }
    info->flags |= SDIO_FLAG_WRITE;

    /* Enable data error interrupt */
    sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS_EN,
                     DATA_TIMEOUT_ERR_STAT_EN);

    info->write_buf.buffer = transfer->data;
    info->write_buf.head = 0;
    info->write_buf.tail = (uint16_t)(transfer->blocks * transfer->blksz);
    info->blocks = transfer->blocks;
    info->blksz = transfer->blksz;
    info->write_callback = transfer->callback;

    if (!info->write_callback) { /* Blocking */
        sdio_write_fifo_data(info);
        info->flags &= ~SDIO_FLAG_WRITE;
    } else { /* Non-blocking */
        sem_post(&info->write_sem);
    }

    return 0;
}

/**
 * @brief Read data from SD card
 *
 * This function is to read data from SD card through the SD host controller.
 * It could be blocking or non-blocking and through DMA or PIO mode.
 *
 * @param dev Pointer to structure of device.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_read(struct device *dev, struct sdio_transfer *transfer)
{
    struct tsb_sdio_info *info = device_get_private(dev);

    if (info->flags & SDIO_FLAG_READ) {
        return -EBUSY;
    }
    info->flags |= SDIO_FLAG_READ;

    /* Enable data error interrupt */
    sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS_EN,
                     DATA_TIMEOUT_ERR_STAT_EN);

    info->read_buf.buffer = transfer->data;
    info->read_buf.head = 0;
    info->read_buf.tail = (uint16_t)(transfer->blocks * transfer->blksz);
    info->blocks = transfer->blocks;
    info->blksz = transfer->blksz;
    info->read_callback = transfer->callback;

    if (!info->read_callback) { /* Blocking */
        sdio_read_fifo_data(info);
        info->flags &= ~SDIO_FLAG_READ;
    } else { /* Non-blocking */
        sem_post(&info->read_sem);
    }

    return 0;
}

/**
 * @brief Attach callback function to SD host controller driver
 *
 * @param dev Pointer to structure of device.
 * @param callback Pointer to event callback function.
 * @return 0 on success, negative errno on error.
 */
static int tsb_sdio_attach_callback(struct device *dev,
                                    sdio_event_callback callback)
{
    struct tsb_sdio_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    info->callback = callback;
    return 0;
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
* @param dev Pointer to the SDIO device structure.
* @return 0 for success, negative errno on error
*/
static int tsb_sdio_dev_open(struct device *dev)
{
    struct tsb_sdio_info *info = NULL;
    irqstate_t flags;
    int ret = 0;
    uint32_t pin_number = 0;
    uint8_t value = 0;
    bool power = false;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    flags = irqsave();

    if (info->flags & SDIO_FLAG_OPEN) {
        ret = -EBUSY;
        goto err_irqrestore;
    }

    info->flags |= SDIO_FLAG_OPEN;
    info->card_event = HC_SDIO_CARD_REMOVED;
    info->pre_card_event = HC_SDIO_CARD_REMOVED;
    info->data_timeout = false;

    sdio_board_dev = device_open(DEVICE_TYPE_SDIO_BOARD_HW, 0);
    if (!sdio_board_dev) {
        ret = -ENODEV;
        goto err_irqrestore;
    }

    /* Switch the pin share mode and pin high for GPB2_SD_POWER_EN pin */
    power = true;
    ret = device_sdio_config_power_enable(sdio_board_dev, power);
    if (ret) {
        goto err_close_device;
    }

    /* SD CD pin and interrupt handler initialization */
    ret = device_sdio_config_card_detect(sdio_board_dev);
    if (ret) {
        goto err_close_device;
    }

    ret = device_sdio_board_get_cd_pin_number(sdio_board_dev, &pin_number);
    if (ret) {
        goto err_close_device;
    }
    info->cd_pin_number = pin_number;
    gpio_irqattach(info->cd_pin_number, sdio_irq_event);
    gpio_unmask_irq(info->cd_pin_number);

    info->thread_abort = false;
    ret = pthread_create(&info->write_data_thread, NULL, sdio_write_data_thread,
                         info);
    if (ret) {
        goto err_gpio_deactivate;
    }

    ret = pthread_create(&info->read_data_thread, NULL, sdio_read_data_thread,
                         info);
    if (ret) {
        goto err_write_pthread_join;
    }

    usleep(REGISTER_INTERVAL);
    value = gpio_get_value(info->cd_pin_number);

    /* Card is inserted before SDIO device open. */
    if (!value) {
        /* Enable command interrupts */
        sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS_EN,
                         CMD_TIMEOUT_ERR_STAT_EN | CMD_COMPLETE_STAT_EN);
        sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                         CMD_TIMEOUT_ERR_EN | CMD_COMPLETE_EN);
        /* Enable data error interrupt */
        sdio_reg_bit_set(info->sdio_reg_base, INT_ERR_STATUS_EN,
                         DATA_TIMEOUT_ERR_STAT_EN);
        sdio_enable_bus_power(info);
        sdio_reg_bit_set(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                         SD_CLOCK_ENABLE);
        info->card_event = HC_SDIO_CARD_INSERTED;
        if (info->pre_card_event != info->card_event) {
            if (info->callback) {
                info->callback(info->card_event);
            }
            info->pre_card_event = info->card_event;
        }
    }

    irqrestore(flags);

    return 0;

err_write_pthread_join:
    info->thread_abort = true;
    sem_post(&info->write_sem);
    /* Wait for write_data_thread completed */
    pthread_join(info->write_data_thread, NULL);
err_gpio_deactivate:
    gpio_unmask_irq(info->cd_pin_number);
    gpio_deactivate(info->cd_pin_number);
err_close_device:
    device_close(sdio_board_dev);
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
* @param dev Pointer to the SDIO device structure.
* @return None.
*/
static void tsb_sdio_dev_close(struct device *dev)
{
    struct tsb_sdio_info *info = NULL;
    irqstate_t flags;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    flags = irqsave();

    if (!(info->flags & SDIO_FLAG_OPEN)) {
        goto err_irqrestore;
    }

    info->flags &= ~SDIO_FLAG_OPEN;
    info->card_event = HC_SDIO_CARD_REMOVED;
    info->pre_card_event = HC_SDIO_CARD_REMOVED;

    /* Disable command interrupts */
    sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_STATUS_EN,
                     CMD_TIMEOUT_ERR_STAT_EN | CMD_COMPLETE_STAT_EN);
    sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_SIGNAL_EN,
                     CMD_TIMEOUT_ERR_EN | CMD_COMPLETE_EN);
    /* Disable data error interrupt */
    sdio_reg_bit_clr(info->sdio_reg_base, INT_ERR_STATUS_EN,
                     DATA_TIMEOUT_ERR_STAT_EN);
    sdio_disable_bus_power(info);
    sdio_reg_bit_clr(info->sdio_reg_base, CLOCK_SWRST_TIMEOUT_CONTROL,
                     SD_CLOCK_ENABLE);

    info->thread_abort = true;
    sem_post(&info->read_sem);
    sem_post(&info->write_sem);
    /* Wait for read_data_thread completed */
    pthread_join(info->read_data_thread, NULL);
    /* Wait for write_data_thread completed */
    pthread_join(info->write_data_thread, NULL);

    gpio_unmask_irq(info->cd_pin_number);
    gpio_deactivate(info->cd_pin_number);

    device_close(sdio_board_dev);

err_irqrestore:
    irqrestore(flags);
}

/**
* @brief The device probe function.
*
* This function is called by the system to probe the SDIO hardware and do SDIO
* early initialization when the system boots up. This function allocates memory
* for saving driver internal information data, attaches the interrupt handlers
* to IRQs of the controller and configures the pin settings of the SDIO
* controller.
*
* @param dev Pointer to the SDIO device structure.
* @return 0 for success, negative errno on error.
*/
static int tsb_sdio_dev_probe(struct device *dev)
{
    struct tsb_sdio_info *info = NULL;
    irqstate_t flags;
    int ret = 0;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    /* Enable the 2 clock gating (sdioSysClk/sdioSdClk) */
    tsb_clk_enable(TSB_CLK_SDIOSYS);
    tsb_clk_enable(TSB_CLK_SDIOSD);

    /* Reset the host controller */
    tsb_reset(TSB_RST_SDIOSYS);
    tsb_reset(TSB_RST_SDIOSD);

    /* Assert the DLL enable */
    sdio_reg_bit_set(SYSCTL_BASE, UHSSD_DLLCTRL, DLL_ENABLE);

    ret = tsb_request_pinshare(TSB_PIN_SDIO);
    if (ret) {
        lowsyslog("SDIO: cannot get ownership of SDIO pin\n");
        goto err_req_pinshare;
    }

    /* Switch the pin share mode for SD Interfaces */
    tsb_set_pinshare(TSB_PIN_SDIO);

    ret = sdio_extract_resources(dev, info);
    if (ret) {
        goto err_free_info;
    }

    ret = sem_init(&info->cmd_sem, 0, 0);
    if (ret) {
        goto err_free_info;
    }

    ret = sem_init(&info->write_sem, 0, 0);
    if (ret) {
        goto err_destroy_cmd_sem;
    }

    ret = sem_init(&info->read_sem, 0, 0);
    if (ret) {
        goto err_destroy_write_sem;
    }

    flags = irqsave();

    ret = irq_attach(info->sdio_irq, sdio_irq_handler);
    if (ret) {
        goto err_destroy_read_sem;
    }

    up_enable_irq(info->sdio_irq);

    sdio_dev = dev;
    info->dev = dev;
    device_set_private(dev, info);

    irqrestore(flags);

    return 0;

err_destroy_read_sem:
    irqrestore(flags);
    sem_destroy(&info->read_sem);
err_destroy_write_sem:
    sem_destroy(&info->write_sem);
err_destroy_cmd_sem:
    sem_destroy(&info->cmd_sem);
err_free_info:
err_req_pinshare:
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_DLLCTRL, DLL_ENABLE);
    tsb_clk_disable(TSB_CLK_SDIOSYS);
    tsb_clk_disable(TSB_CLK_SDIOSD);
    free(info);

    return ret;
}

/**
* @brief The device remove function.
*
* This function is called by the system to unregister this driver. It
* must be called after probe() and open(). It detaches IRQ handlers and frees
* the internal information memory space.
*
* @param dev Pointer to the SDIO device structure.
* @return None.
*/
static void tsb_sdio_dev_remove(struct device *dev)
{
    struct tsb_sdio_info *info = NULL;
    irqstate_t flags;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->flags & SDIO_FLAG_OPEN) {
        tsb_sdio_dev_close(dev);
    }
    info->flags = 0;

    flags = irqsave();
    up_disable_irq(info->sdio_irq);
    irq_detach(info->sdio_irq);
    sem_destroy(&info->read_sem);
    sem_destroy(&info->write_sem);
    sem_destroy(&info->cmd_sem);
    irqrestore(flags);
    sdio_reg_bit_clr(SYSCTL_BASE, UHSSD_DLLCTRL, DLL_ENABLE);
    tsb_clk_disable(TSB_CLK_SDIOSYS);
    tsb_clk_disable(TSB_CLK_SDIOSD);
    free(info);
    sdio_dev = NULL;
    device_set_private(dev, NULL);
    tsb_release_pinshare(TSB_PIN_SDIO);
}

static struct device_sdio_type_ops tsb_sdio_type_ops = {
    .get_capabilities = tsb_sdio_get_capability,
    .set_ios          = tsb_sdio_set_ios,
    .send_cmd         = tsb_sdio_send_cmd,
    .write            = tsb_sdio_write,
    .read             = tsb_sdio_read,
    .attach_callback  = tsb_sdio_attach_callback,
};

static struct device_driver_ops tsb_sdio_driver_ops = {
    .probe    = tsb_sdio_dev_probe,
    .remove   = tsb_sdio_dev_remove,
    .open     = tsb_sdio_dev_open,
    .close    = tsb_sdio_dev_close,
    .type_ops = &tsb_sdio_type_ops,
};

struct device_driver tsb_sdio_driver = {
    .type = DEVICE_TYPE_SDIO_HW,
    .name = "tsb_sdio",
    .desc = "TSB SDIO Driver",
    .ops  = &tsb_sdio_driver_ops,
};
