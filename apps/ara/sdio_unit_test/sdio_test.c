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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/device.h>
#include <nuttx/device_sdio.h>
#include <arch/tsb/chip.h>

/* SDIO operation definition */
#define SDIO_GET_CAPABILITY         0x01
#define SDIO_MMC_POWER_UP           0x02
#define SDIO_SET_CARD_TO_IDLE_STATE 0x03
#define SDIO_WRITE                  0x04
#define SDIO_READ                   0x05
#define SDIO_CARD_IDENTIFY          0x06

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

/* SDIO clock definition */
#define SDIO_INIT_SPEED_MODE   400000   /* 400KHz */
#define SDIO_NORMAL_SPEED_MODE 25000000 /* 25MHz */
#define SDIO_HIGH_SPEED_MODE   50000000 /* 50MHz */

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

/* Host controller SDIO response definition */
#define HC_SDIO_RSP_R1_R5_R6_R7 (HC_SDIO_RSP_PRESENT | HC_SDIO_RSP_CRC | \
                                 HC_SDIO_RSP_OPCODE)
#define HC_SDIO_RSP_R1B         (HC_SDIO_RSP_PRESENT | HC_SDIO_RSP_CRC | \
                                 HC_SDIO_RSP_OPCODE | HC_SDIO_RSP_BUSY)
#define HC_SDIO_RSP_R2          (HC_SDIO_RSP_PRESENT | HC_SDIO_RSP_CRC | \
                                 HC_SDIO_RSP_136)
#define HC_SDIO_RSP_R3_R4       (HC_SDIO_RSP_PRESENT)

/* Host controller SDIO command type definition */
#define HC_SDIO_CMD_AC   0x00 /* Normal, Addressed Command */
#define HC_SDIO_CMD_ADTC 0x01 /* Suspend, Addressed Data Transfer Command */
#define HC_SDIO_CMD_BC   0x02 /* Resume, Broadcasted Command, no response */
#define HC_SDIO_CMD_BCR  0x03 /* Abort, Broadcasted Command with response */

/* Host controller commands */
/* Standard MMC commands (4.1)      type  argument                   response */
/* class 1 */
#define HC_MMC_GO_IDLE_STATE        0  /* bc                                  */
#define HC_MMC_ALL_SEND_CID         2  /* bcr                        R2       */
#define HC_MMC_SELECT_CARD          7  /* ac   [31:16] RCA           R1       */
#define HC_MMC_SEND_CSD             9  /* ac   [31:16] RCA           R2       */
#define HC_MMC_SEND_CID             10 /* ac   [31:16] RCA           R2       */
#define HC_MMC_SEND_STATUS          13 /* ac   [31:16] RCA           R1       */
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
/* class 0 */
/* This is basically the same command as for MMC with some quirks. */
#define HC_SD_SEND_RELATIVE_ADDR    3  /* bcr                        R6       */
#define HC_SD_SEND_IF_COND          8  /* bcr  [11:0] See below      R7       */
/*
 * HC_SD_SEND_IF_COND argument format:
 * [31:12] Reserved (0)
 * [11:8] Host Voltage Supply Flags
 * [7:0] Check Pattern (0xAA)
 */
/* Application commands */
#define HC_SD_APP_SET_BUS_WIDTH     6  /* ac   [1:0] bus width       R1       */
#define HC_SD_APP_OP_COND           41 /* bcr  [31:0] OCR            R3       */
#define HC_SD_APP_SEND_SCR          51 /* adtc                       R1       */

/* Argument of ACMD41 bit field details */
#define OCR_VDD_33_34         BIT(21)
#define SDXC_POWER_CONTROL    BIT(28)
#define HOST_CAPACITY_SUPPORT BIT(30)

/* Response of ACMD41 (R3) bit field details */
#define CARD_CAPACITY_STATUS BIT(30)
#define BUSY_STATUS          BIT(31) /* Busy Status: 0b: On Initialization, 1b:
                                      * Initialization Complete */

/* SDIO card status bit field details */
#define SDIO_AKE_SEQ_ERROR      BIT(3)
#define SDIO_APP_CMD            BIT(5)
#define SDIO_READY_FOR_DATA     BIT(8)
#define SDIO_ERASE_RESET        BIT(13)
#define SDIO_CARD_ECC_DISABLED  BIT(14)
#define SDIO_WP_ERASE_SKIP      BIT(15)
#define SDIO_CSD_OVERWRITE      BIT(16)
#define SDIO_ERROR              BIT(19)
#define SDIO_CC_ERROR           BIT(20)
#define SDIO_CARD_ECC_FAILED    BIT(21)
#define SDIO_ILLEGAL_COMMAND    BIT(22)
#define SDIO_COM_CRC_ERROR      BIT(23)
#define SDIO_LOCK_UNLOCK_FAILED BIT(24)
#define SDIO_CARD_IS_LOCKED     BIT(25)
#define SDIO_WP_VIOLATION       BIT(26)
#define SDIO_ERASE_PARAM        BIT(27)
#define SDIO_ERASE_SEQ_ERROR    BIT(28)
#define SDIO_BLOCK_LEN_ERROR    BIT(29)
#define SDIO_ADDRESS_ERROR      BIT(30)
#define SDIO_OUT_OF_RANGE       BIT(31)

/* SDIO card status bit mask definition */
#define SDIO_CURRENT_STATE_MASK 0x00001E00

/* SDIO card status bit shift definition */
#define SDIO_CURRENT_STATE_SHIFT 9

/* New published RCA bit shift definition */
#define NEW_PUBLISHED_RCA_SHIFT 16

/* Voltage check definition */
#define CHECK_PATTERN          0xAA
#define CHECK_PATTERN_MASK     0xFF
#define VOLTAGE_SUPPLIED_MASK  0x1FF00
#define VOLTAGE_SUPPLIED_SHIFT 8

/* ACMD41 definition */
#define ACMD41_INTERVAL  50000 /* 50ms */
#define ACMD41_MAX_RETRY 20

/* SCR register bit field details */
#define SD_BUS_WIDTHS_4BIT BIT(18)

/* Data block definition */
#define BLOCK_LENGTH       512
#define BLOCK_COUNT        2
#define SCR_BLOCK_LENGTH   8
#define SCR_BLOCK_COUNT    1
#define SINGLE_BLOCK_COUNT 1

/* Data shift definition */
#define BYTE_SHIFT 8

/* Data block mode */
#define SINGLE_MODE   0x00
#define MULTIPLE_MODE 0x01

/* Start addrerss of data read and write */
#define DATA_START_ADDRESS 0

/* Data dump definition */
#define DATA_DUMP_SIZE 8

/**
 * @brief SDIO application private information
 */
struct sdio_app_info {
    /** SDIO capabilities */
    struct sdio_cap cap;
    /** Command response */
    uint32_t resp[4];
    /** RCA register */
    uint32_t rca;
    /** SCR register */
    uint32_t scr[2];
    /** CSD register */
    uint32_t csd[4];
    /** CID register */
    uint32_t cid[4];
    /** Addressed CID register */
    uint32_t addressed_cid[4];
    /** Card status */
    uint32_t card_status;
    /** Data block length */
    uint32_t blocklen;
    /** Data block count */
    uint32_t blockcnt;
    /** Card CID */
    uint16_t new_published_rca;
    /** SDIO operation */
    uint8_t operation;
    /** Data bus width */
    uint8_t bus_width;
    /** Data block mode */
    uint8_t data_mode;
    /** CMD8 valid flag */
    bool F8;
};

/**
 * @brief Print usage of this SDIO application.
 *
 * @return None.
 */
static void sdio_print_usage(void) {
    printf("Usage: sdio_test [-operation]\n");
    printf("    -g: sdio get capability operation\n");
    printf("    -p: sdio mmc power up\n");
    printf("    -I: set card to idle state\n");
    printf("    -w: sdio write operation\n");
    printf("        0: single block\n");
    printf("        1: multiple blocks\n");
    printf("    -r: sdio read operation\n");
    printf("        0: single block\n");
    printf("        1: multiple blocks\n");
    printf("    -i: card identify\n");
}

/**
 * @brief Dump capabilities of SD host controller.
 *
 * @param cap Pointer to structure of capabilities.
 * @return None.
 */
static void sdio_dump_capabilities(struct sdio_cap *cap)
{
    printf("sdio_test: capabilities of the SD host controller:\n");
    printf("sdio_test: HC_SDIO_CAP_NONREMOVABLE:   %s\n",
           (cap->caps & HC_SDIO_CAP_NONREMOVABLE) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_4_BIT_DATA:     %s\n",
           (cap->caps & HC_SDIO_CAP_4_BIT_DATA) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_8_BIT_DATA:     %s\n",
           (cap->caps & HC_SDIO_CAP_8_BIT_DATA) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_MMC_HS:         %s\n",
           (cap->caps & HC_SDIO_CAP_MMC_HS) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_SD_HS:          %s\n",
           (cap->caps & HC_SDIO_CAP_SD_HS) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_ERASE:          %s\n",
           (cap->caps & HC_SDIO_CAP_ERASE) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_1_2V_DDR:       %s\n",
           (cap->caps & HC_SDIO_CAP_1_2V_DDR) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_1_8V_DDR:       %s\n",
           (cap->caps & HC_SDIO_CAP_1_8V_DDR) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_POWER_OFF_CARD: %s\n",
           (cap->caps & HC_SDIO_CAP_POWER_OFF_CARD) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_UHS_SDR12:      %s\n",
           (cap->caps & HC_SDIO_CAP_UHS_SDR12) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_UHS_SDR25:      %s\n",
           (cap->caps & HC_SDIO_CAP_UHS_SDR25) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_UHS_SDR50:      %s\n",
           (cap->caps & HC_SDIO_CAP_UHS_SDR50) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_UHS_SDR104:     %s\n",
           (cap->caps & HC_SDIO_CAP_UHS_SDR104) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_UHS_DDR50:      %s\n",
           (cap->caps & HC_SDIO_CAP_UHS_DDR50) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_DRIVER_TYPE_A:  %s\n",
           (cap->caps & HC_SDIO_CAP_DRIVER_TYPE_A) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_DRIVER_TYPE_C:  %s\n",
           (cap->caps & HC_SDIO_CAP_DRIVER_TYPE_C) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_DRIVER_TYPE_D:  %s\n",
           (cap->caps & HC_SDIO_CAP_DRIVER_TYPE_D) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_HS200_1_2V:     %s\n",
           (cap->caps & HC_SDIO_CAP_HS200_1_2V) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_HS200_1_8V:     %s\n",
           (cap->caps & HC_SDIO_CAP_HS200_1_8V) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_HS400_1_2V:     %s\n",
           (cap->caps & HC_SDIO_CAP_HS400_1_2V) ? "YES" : "NO");
    printf("sdio_test: HC_SDIO_CAP_HS400_1_8V:     %s\n",
           (cap->caps & HC_SDIO_CAP_HS400_1_8V) ? "YES" : "NO");
    printf("sdio_test: ocr:                        0x%x\n", cap->ocr);
    printf("sdio_test: minimum frequency:          %d\n", cap->f_min);
    printf("sdio_test: maximum frequency:          %d\n", cap->f_max);
    printf("sdio_test: maximum block count:        %d\n", cap->max_blk_count);
    printf("sdio_test: maximum block size:         %d\n", cap->max_blk_size);
}

/**
 * @brief Dump card status.
 *
 * @param resp SDIO command response.
 * @return None.
 */
static void sdio_dump_card_status(uint32_t resp)
{
    printf("sdio_test: SDIO_AKE_SEQ_ERROR:      %s\n",
           (resp & SDIO_AKE_SEQ_ERROR) ? "YES" : "NO");
    printf("sdio_test: SDIO_APP_CMD:            %s\n",
           (resp & SDIO_APP_CMD) ? "YES" : "NO");
    printf("sdio_test: SDIO_READY_FOR_DATA:     %s\n",
           (resp & SDIO_READY_FOR_DATA) ? "YES" : "NO");
    printf("sdio_test: SDIO_CURRENT_STATE:      %x\n",
           (resp & SDIO_CURRENT_STATE_MASK) >> SDIO_CURRENT_STATE_SHIFT);
    printf("sdio_test: SDIO_ERASE_RESET:        %s\n",
           (resp & SDIO_ERASE_RESET) ? "YES" : "NO");
    printf("sdio_test: SDIO_CARD_ECC_DISABLED:  %s\n",
           (resp & SDIO_CARD_ECC_DISABLED) ? "YES" : "NO");
    printf("sdio_test: SDIO_WP_ERASE_SKIP:      %s\n",
           (resp & SDIO_WP_ERASE_SKIP) ? "YES" : "NO");
    printf("sdio_test: SDIO_CSD_OVERWRITE:      %s\n",
           (resp & SDIO_CSD_OVERWRITE) ? "YES" : "NO");
    printf("sdio_test: SDIO_ERROR:              %s\n",
           (resp & SDIO_ERROR) ? "YES" : "NO");
    printf("sdio_test: SDIO_CC_ERROR:           %s\n",
           (resp & SDIO_CC_ERROR) ? "YES" : "NO");
    printf("sdio_test: SDIO_CARD_ECC_FAILED:    %s\n",
           (resp & SDIO_CARD_ECC_FAILED) ? "YES" : "NO");
    printf("sdio_test: SDIO_ILLEGAL_COMMAND:    %s\n",
           (resp & SDIO_ILLEGAL_COMMAND) ? "YES" : "NO");
    printf("sdio_test: SDIO_COM_CRC_ERROR:      %s\n",
           (resp & SDIO_COM_CRC_ERROR) ? "YES" : "NO");
    printf("sdio_test: SDIO_LOCK_UNLOCK_FAILED: %s\n",
           (resp & SDIO_LOCK_UNLOCK_FAILED) ? "YES" : "NO");
    printf("sdio_test: SDIO_CARD_IS_LOCKED:     %s\n",
           (resp & SDIO_CARD_IS_LOCKED) ? "YES" : "NO");
    printf("sdio_test: SDIO_WP_VIOLATION:       %s\n",
           (resp & SDIO_WP_VIOLATION) ? "YES" : "NO");
    printf("sdio_test: SDIO_ERASE_PARAM:        %s\n",
           (resp & SDIO_ERASE_PARAM) ? "YES" : "NO");
    printf("sdio_test: SDIO_ERASE_SEQ_ERROR:    %s\n",
           (resp & SDIO_ERASE_SEQ_ERROR) ? "YES" : "NO");
    printf("sdio_test: SDIO_BLOCK_LEN_ERROR:    %s\n",
           (resp & SDIO_BLOCK_LEN_ERROR) ? "YES" : "NO");
    printf("sdio_test: SDIO_ADDRESS_ERROR:      %s\n",
           (resp & SDIO_ADDRESS_ERROR) ? "YES" : "NO");
    printf("sdio_test: SDIO_OUT_OF_RANGE:       %s\n",
           (resp & SDIO_OUT_OF_RANGE) ? "YES" : "NO");
}

/**
 * @brief Command parser.
 *
 * Parse the command input from console.
 *
 * @param info Pointer to structure of info.
 * @param argc Indicate how many input parameters.
 * @param argv Commands input from console.
 * @return 0 on success, negative errno on error.
 */
static int sdio_cmd_parser(struct sdio_app_info *info, int argc, char **argv)
{
    int option;
    int ret = -EINVAL;

    optind = -1; /* Force NuttX's getopt() to reinitialize. */
    while ((option = getopt(argc, argv, "gpIw:r:i")) != ERROR) {
        switch(option) {
            case 'g':
                info->operation = SDIO_GET_CAPABILITY;
                ret = 0;
                break;
            case 'p':
                info->operation = SDIO_MMC_POWER_UP;
                ret = 0;
                break;
            case 'I':
                info->operation = SDIO_SET_CARD_TO_IDLE_STATE;
                ret = 0;
                break;
            case 'w':
                info->operation = SDIO_WRITE;
                info->data_mode = (uint8_t)atoi(optarg);
                if (info->data_mode < SINGLE_MODE ||
                    info->data_mode > MULTIPLE_MODE) {
                    return -EINVAL;
                }
                ret = 0;
                break;
            case 'r':
                info->operation = SDIO_READ;
                info->data_mode = (uint8_t)atoi(optarg);
                if (info->data_mode < SINGLE_MODE ||
                    info->data_mode > MULTIPLE_MODE) {
                    return -EINVAL;
                }
                ret = 0;
                break;
            case 'i':
                info->operation = SDIO_CARD_IDENTIFY;
                ret = 0;
                break;
            default:
                return -EINVAL;
        }
    }

    return ret;
}

/**
 * @brief Set SDIO default parameters.
 *
 * This is function to set SDIO parameters using in this application to default
 * value.
 *
 * @param info Pointer to structure of info.
 * @return None.
 */
static void sdio_default_params(struct sdio_app_info *info)
{
    info->blocklen = BLOCK_LENGTH;
    info->blockcnt = BLOCK_COUNT;
    info->operation = SDIO_GET_CAPABILITY;
    info->data_mode = SINGLE_MODE;
    info->F8 = false;
}

/**
 * @brief Check card status.
 *
 * @param resp SDIO command response.
 * @return 0 on success, negative errno on error.
 */
static int sdio_check_card_status(uint32_t resp)
{
    int ret = 0;
    uint32_t error_mask = SDIO_OUT_OF_RANGE | SDIO_ADDRESS_ERROR |
                          SDIO_BLOCK_LEN_ERROR | SDIO_ERASE_SEQ_ERROR |
                          SDIO_ERASE_PARAM | SDIO_WP_VIOLATION |
                          SDIO_LOCK_UNLOCK_FAILED | SDIO_COM_CRC_ERROR |
                          SDIO_ILLEGAL_COMMAND | SDIO_CARD_ECC_FAILED |
                          SDIO_CC_ERROR | SDIO_ERROR | SDIO_CSD_OVERWRITE |
                          SDIO_WP_ERASE_SKIP | SDIO_AKE_SEQ_ERROR;

    if (resp & error_mask) {
        ret = -EIO;
    }

    return ret;
}

/**
 * @brief SDIO mmc power up.
 *
 * Set configurations to SD host controller for card identify phase.
 *
 * @param dev Pointer to structure of device.
 * @param ios Pointer to structure of ios.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_power_up(struct device *dev, struct sdio_ios *ios)
{
    ios->clock = SDIO_INIT_SPEED_MODE;
    ios->power_mode = HC_SDIO_POWER_UP;
    ios->bus_width = HC_SDIO_BUS_WIDTH_1;
    ios->timing = HC_SDIO_TIMING_SD_HS;
    ios->drv_type = HC_SDIO_SET_DRIVER_TYPE_A;
    return device_sdio_set_ios(dev, ios);
}

/**
 * @brief SDIO mmc power on.
 *
 * Set configurations to SD host controller for data transfer phase.
 *
 * @param dev Pointer to structure of device.
 * @param ios Pointer to structure of ios.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_power_on(struct device *dev, struct sdio_ios *ios)
{
    ios->clock = SDIO_NORMAL_SPEED_MODE;
    ios->power_mode = HC_SDIO_POWER_ON;
    ios->bus_width = HC_SDIO_BUS_WIDTH_4;
    ios->timing = HC_SDIO_TIMING_SD_HS;
    ios->drv_type = HC_SDIO_SET_DRIVER_TYPE_A;
    return device_sdio_set_ios(dev, ios);
}

/**
 * @brief Go idle state.
 *
 * Resets all cards to idle state.
 *
 * @param dev Pointer to structure of device.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_go_idle_state_cmd0(struct device *dev, struct sdio_cmd *cmd)
{
    cmd->cmd = HC_MMC_GO_IDLE_STATE;
    cmd->cmd_flags = HC_SDIO_RSP_NONE;
    cmd->cmd_type = HC_SDIO_CMD_BC;
    cmd->cmd_arg = 0;
    return device_sdio_send_cmd(dev, cmd);
}

/**
 * @brief Get Card IDentification (CID).
 *
 * Asks any card to send the CID numbers on the CMD line. (any card that is
 * connected to the host will respond)
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_all_send_cid_cmd2(struct device *dev,
                                      struct sdio_app_info *info,
                                      struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_ALL_SEND_CID;
    cmd->cmd_flags = HC_SDIO_RSP_R2;
    cmd->cmd_type = HC_SDIO_CMD_BCR;
    cmd->cmd_arg = 0;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support get CID (CMD2)\n");
        }
    } else {
        info->cid[0] = cmd->resp[0];
        info->cid[1] = cmd->resp[1];
        info->cid[2] = cmd->resp[2];
        info->cid[3] = cmd->resp[3];
    }

    return ret;
}

/**
 * @brief Select card.
 *
 * Command toggles a card between the stand-by and transfer states or between
 * the programming and disconnect states. In both cases, the card is selected
 * by its own relative address and gets deselected by any other address;
 * address 0 deselects all. In the case that the RCA equals 0, then the host
 * may do one of the following:
 * 1. Use other RCA number to perform card de-selection.
 * 2. Re-send CMD3 to change its RCA number to other than 0 and then use
 * CMD7 with RCA=0 for card de-selection.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_select_card_cmd7(struct device *dev,
                                     struct sdio_app_info *info,
                                     struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_SELECT_CARD;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_AC;
    cmd->cmd_arg = info->new_published_rca << NEW_PUBLISHED_RCA_SHIFT;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support select card (CMD7)\n");
        }
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    return ret;
}

/**
 * @brief Get Card-Specific Data (CSD).
 *
 * Addressed card sends its CSD on the CMD line.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_send_csd_cmd9(struct device *dev,
                                  struct sdio_app_info *info,
                                  struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_SEND_CSD;
    cmd->cmd_flags = HC_SDIO_RSP_R2;
    cmd->cmd_type = HC_SDIO_CMD_AC;
    cmd->cmd_arg = info->new_published_rca << NEW_PUBLISHED_RCA_SHIFT;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support get card CSD (CMD9)\n");
        }
    } else {
        info->csd[0] = cmd->resp[0];
        info->csd[1] = cmd->resp[1];
        info->csd[2] = cmd->resp[2];
        info->csd[3] = cmd->resp[3];
    }

    return ret;
}

/**
 * @brief Get addressed Card IDentification (CID).
 *
 * Addressed card sends its CID on CMD the line.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_send_cid_cmd10(struct device *dev,
                                   struct sdio_app_info *info,
                                   struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_SEND_CID;
    cmd->cmd_flags = HC_SDIO_RSP_R2;
    cmd->cmd_type = HC_SDIO_CMD_AC;
    cmd->cmd_arg = info->new_published_rca << NEW_PUBLISHED_RCA_SHIFT;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support get addressed CID (CMD10)\n");
        }
    } else {
        info->addressed_cid[0] = cmd->resp[0];
        info->addressed_cid[1] = cmd->resp[1];
        info->addressed_cid[2] = cmd->resp[2];
        info->addressed_cid[3] = cmd->resp[3];
    }

    return ret;
}

/**
 * @brief Get card status.
 *
 * Addressed card sends its status register.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_send_status_cmd13(struct device *dev,
                                      struct sdio_app_info *info,
                                      struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_SEND_STATUS;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_AC;
    cmd->cmd_arg = info->new_published_rca << NEW_PUBLISHED_RCA_SHIFT;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support get card status (CMD13)\n");
        }
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        info->card_status = cmd->resp[0];
        sdio_dump_card_status(cmd->resp[0]);
    }

    return ret;
}

/**
 * @brief Set block length.
 *
 * In the case of a Standard Capacity SD Memory Card, this command sets the
 * block length (in bytes) for all following block commands (read, write, lock).
 * Default block length is fixed to 512 Bytes. Set length is valid for memory
 * access commands only if partial block read operation are allowed in CSD.
 * In the case of SDHC and SDXC Cards, block length set by CMD16 command does
 * not affect memory read and write commands. Always 512 Bytes fixed block
 * length is used. This command is effective for LOCK_UNLOCK command.
 * In both cases, if block length is set larger than 512Bytes, the card sets
 * the BLOCK_LEN_ERROR bit.
 * In DDR50 mode, data is sampled on both edges of the clock. Therefore, block
 * length shall always be even.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_set_blocklen_cmd16(struct device *dev,
                                       struct sdio_app_info *info,
                                       struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_SET_BLOCKLEN;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_AC;
    cmd->cmd_arg = info->blocklen;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support set block length (CMD16)\n");
        }
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    return ret;
}

/**
 * @brief Read single block.
 *
 * In the case of a Standard Capacity SD Memory Card, this command, this command
 * reads a block of the size selected by the SET_BLOCKLEN command.
 * In case of SDHC and SDXC Cards, block length is fixed 512 Bytes regardless of
 * the SET_BLOCKLEN command.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_read_single_block_cmd17(struct device *dev,
                                            struct sdio_app_info *info,
                                            struct sdio_cmd *cmd,
                                            struct sdio_transfer *transfer)
{
    int ret = 0;
    uint8_t data[BLOCK_LENGTH] = {0};
    uint32_t i = 0;

    cmd->cmd = HC_MMC_READ_SINGLE_BLOCK;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_ADTC;
    cmd->cmd_arg = DATA_START_ADDRESS; /* start addr */
    cmd->data_blocks = SINGLE_BLOCK_COUNT;
    cmd->data_blksz = BLOCK_LENGTH;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support read single block (CMD17)\n");
        }
        return ret;
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    /* Clean data_blocks and data_blksz of cmd */
    cmd->data_blocks = 0;
    cmd->data_blksz = 0;

    transfer->blocks = SINGLE_BLOCK_COUNT;
    transfer->blksz = BLOCK_LENGTH;
    transfer->data = data;
    transfer->dma = NULL;
    transfer->callback = NULL; /* Blocking mode */
    ret = device_sdio_read(dev, transfer);

    if (ret) {
        printf("sdio_test: read single block fail\n");
    } else {
        printf("sdio_test: read single block success\n");
        printf("Data read:\n");
        for (i = 0; i < BLOCK_LENGTH; i++) {
            if ((i % DATA_DUMP_SIZE)) {
                printf("0x%02x ", data[i]);
            } else {
                printf("\n0x%02x ", data[i]);
            }
        }
        printf("\n");
    }

    return ret;
}

/**
 * @brief Read multiple blocks.
 *
 * Continuously transfers data blocks from card to host until interrupted by a
 * STOP_TRANSMISSION command. Block length is specified the same as
 * READ_SINGLE_BLOCK command.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_read_multiple_blocks_cmd18(struct device *dev,
                                               struct sdio_app_info *info,
                                               struct sdio_cmd *cmd,
                                               struct sdio_transfer *transfer)
{
    int ret = 0;
    uint8_t data[BLOCK_LENGTH * BLOCK_COUNT] = {0};
    uint32_t i = 0;

    cmd->cmd = HC_MMC_READ_MULTIPLE_BLOCK;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_ADTC;
    cmd->cmd_arg = DATA_START_ADDRESS; /* start addr */
    cmd->data_blocks = BLOCK_COUNT;
    cmd->data_blksz = BLOCK_LENGTH;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support read multiple blocks (CMD18)\n");
        }
        return ret;
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    /* Clean data_blocks and data_blksz of cmd */
    cmd->data_blocks = 0;
    cmd->data_blksz = 0;

    transfer->blocks = BLOCK_COUNT;
    transfer->blksz = BLOCK_LENGTH;
    transfer->data = data;
    transfer->dma = NULL;
    transfer->callback = NULL; /* Blocking mode */
    ret = device_sdio_read(dev, transfer);

    if (ret) {
        printf("sdio_test: read multiple blocks fail\n");
    } else {
        printf("sdio_test: read multiple blocks success\n");
        printf("Data read:\n");
        for (i = 0; i < (BLOCK_LENGTH * BLOCK_COUNT); i++) {
            if ((i % DATA_DUMP_SIZE)) {
                printf("0x%02x ", data[i]);
            } else {
                printf("\n0x%02x ", data[i]);
            }
        }
        printf("\n");
    }

    return ret;
}

/**
 * @brief Set block count.
 *
 * Specify block count for CMD18 and CMD25.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_set_block_count_cmd23(struct device *dev,
                                          struct sdio_app_info *info,
                                          struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_SET_BLOCK_COUNT;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_ADTC;
    cmd->cmd_arg = info->blockcnt;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support set block count (CMD23)\n");
        }
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    return ret;
}

/**
 * @brief Write single block.
 *
 * In case of SDSC Card, block length is set by the SET_BLOCKLEN command1.
 * In case of SDHC and SDXC Cards, block length is fixed 512 Bytes regardless
 * of the SET_BLOCKLEN command.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_write_block_cmd24(struct device *dev,
                                      struct sdio_app_info *info,
                                      struct sdio_cmd *cmd,
                                      struct sdio_transfer *transfer)
{
    int ret = 0;
    uint8_t data[BLOCK_LENGTH] = {0};
    uint32_t i = 0;

    cmd->cmd = HC_MMC_WRITE_BLOCK;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_ADTC;
    cmd->cmd_arg = DATA_START_ADDRESS; /* start addr */
    cmd->data_blocks = SINGLE_BLOCK_COUNT;
    cmd->data_blksz = BLOCK_LENGTH;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support write single block (CMD24)\n");
        }
        return ret;
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    /* Prepare data to write */
    for (i = 0; i < BLOCK_LENGTH; i++) {
        if (i < (BLOCK_LENGTH / 2)) {
            data[i] = i;
        } else {
            data[i] = i - (BLOCK_LENGTH / 2);
        }
    }

    /* Clean data_blocks and data_blksz of cmd */
    cmd->data_blocks = 0;
    cmd->data_blksz = 0;

    transfer->blocks = SINGLE_BLOCK_COUNT;
    transfer->blksz = BLOCK_LENGTH;
    transfer->data = data;
    transfer->dma = NULL;
    transfer->callback = NULL; /* Blocking mode */
    ret = device_sdio_write(dev, transfer);

    if (ret) {
        printf("sdio_test: write single block fail\n");
    } else {
        printf("sdio_test: write single block success\n");
    }

    return ret;
}

/**
 * @brief Write multiple blocks.
 *
 * Continuously writes blocks of data until a STOP_TRANSMISSION follows. Block
 * length is specified the same as WRITE_BLOCK command.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_write_multiple_blocks_cmd25(struct device *dev,
                                                struct sdio_app_info *info,
                                                struct sdio_cmd *cmd,
                                                struct sdio_transfer *transfer)
{
    int ret = 0;
    uint8_t data[BLOCK_LENGTH * BLOCK_COUNT] = {0};
    uint32_t i = 0, j = 0;

    cmd->cmd = HC_MMC_WRITE_MULTIPLE_BLOCK;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_ADTC;
    cmd->cmd_arg = DATA_START_ADDRESS; /* start addr */
    cmd->data_blocks = BLOCK_COUNT;
    cmd->data_blksz = BLOCK_LENGTH;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support write multiple blocks (CMD25)\n");
        }
        return ret;
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    /* Prepare data to write */
    for (i = 0; i < BLOCK_COUNT; i++) {
        for (j = 0; j < BLOCK_LENGTH; j++) {
            if (j < (BLOCK_LENGTH / 2)) {
                data[(i * BLOCK_LENGTH) + j] = j;
            } else {
                data[(i * BLOCK_LENGTH) + j] = j - (BLOCK_LENGTH / 2);
            }
        }
    }

    /* Clean data_blocks and data_blksz of cmd */
    cmd->data_blocks = 0;
    cmd->data_blksz = 0;

    transfer->blocks = BLOCK_COUNT;
    transfer->blksz = BLOCK_LENGTH;
    transfer->data = data;
    transfer->dma = NULL;
    transfer->callback = NULL; /* Blocking mode */
    ret = device_sdio_write(dev, transfer);

    if (ret) {
        printf("sdio_test: write multiple blocks fail\n");
    } else {
        printf("sdio_test: write multiple blocks success\n");
    }

    return ret;
}

/**
 * @brief Lock/unlock card.
 *
 * Used to set/reset the password or lock/unlock the card. The size of the data
 * block is set by the SET_BLOCK_LEN command. Reserved bits in the argument and
 * in Lock Card Data Structure shall be set to 0.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_lock_unlock_cmd42(struct device *dev,
                                      struct sdio_app_info *info,
                                      struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_LOCK_UNLOCK;
    cmd->cmd_flags = HC_SDIO_RSP_R1B;
    cmd->cmd_type = HC_SDIO_CMD_ADTC;
    cmd->cmd_arg = 0;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support lock/unlock card (CMD42)\n");
        }
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    return ret;
}

/**
 * @brief App cmd.
 *
 * Indicates to the card that the next command is an application specific
 * command rather than a standard command.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_app_cmd_cmd55(struct device *dev,
                                  struct sdio_app_info *info,
                                  struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_APP_CMD;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_AC;
    cmd->cmd_arg = 0;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support app command (CMD55)\n");
        }
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    return ret;
}

/**
 * @brief App command with new published RCA.
 *
 * Send command 55 argument as card's RCA.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_mmc_app_cmd_rca_cmd55(struct device *dev,
                                      struct sdio_app_info *info,
                                      struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_MMC_APP_CMD;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_AC;
    cmd->cmd_arg = info->new_published_rca << NEW_PUBLISHED_RCA_SHIFT;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support app command with rca (CMD55)\n");
        }
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    return ret;
}

/**
 * @brief Get card relative address (RCA).
 *
 * Ask the card to publish a new relative address.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_sd_send_relative_addr_cmd3(struct device *dev,
                                           struct sdio_app_info *info,
                                           struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_SD_SEND_RELATIVE_ADDR;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_BCR;
    cmd->cmd_arg = 0;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support get card relative address (CMD3)\n");
        }
    } else {
        info->rca = cmd->resp[0];
        info->new_published_rca = cmd->resp[0] >> NEW_PUBLISHED_RCA_SHIFT;
    }

    return ret;
}

/**
 * @brief Voltage check.
 *
 * Sends SD Memory Card interface condition, which includes host supply voltage
 * information and asks the card whether card supports voltage.
 * Reserved bits shall be set to '0'.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_sd_send_if_cond_cmd8(struct device *dev,
                                     struct sdio_app_info *info,
                                     struct sdio_cmd *cmd)
{
    int ret = 0;
    uint8_t check_pattern = CHECK_PATTERN, result_pattern = 0;

    cmd->cmd = HC_SD_SEND_IF_COND;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_BCR;
    /* Voltage Supplied:0001b, Value Definition:2.7-3.6V */
    cmd->cmd_arg = (((info->cap.ocr & VOLTAGE_SUPPLIED_MASK) != 0) <<
                    VOLTAGE_SUPPLIED_SHIFT) | check_pattern;
    ret = device_sdio_send_cmd(dev, cmd);

    /* Check Response */
    info->F8 = false;
    if (ret) { /* No Response */
        if (ret == -ETIMEDOUT) {
                printf("sdio_test: not support voltage check (CMD8)\n");
        }
    } else {
        result_pattern = cmd->resp[0] & CHECK_PATTERN_MASK;
        if (result_pattern != check_pattern) { /* Error Response */
            printf("sdio_test: pattern fail\n");
            ret = -EINVAL;
        } else { /* Good Response */
            info->F8 = true;
        }
    }

    return ret;
}

/**
 * @brief Set bus width.
 *
 * Defines the data bus width ('00'=1bit or '10'=4 bits bus) to be used for
 * data transfer. The allowed data bus widths are given in SCR register.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_sd_app_set_bus_width_acmd6(struct device *dev,
                                           struct sdio_app_info *info,
                                           struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_SD_APP_SET_BUS_WIDTH;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_AC;
    if (info->scr[1] & SD_BUS_WIDTHS_4BIT) {
        cmd->cmd_arg = HC_SDIO_BUS_WIDTH_4;
    } else {
        cmd->cmd_arg = HC_SDIO_BUS_WIDTH_1;
    }
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support set bus width (ACMD6)\n");
        }
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    return ret;
}

/**
 * @brief Get card Operation Conditions Register (OCR).
 *
 * Sends host capacity support information (HCS) and asks the accessed card to
 * send its operating condition register (OCR) content in the response on the
 * CMD line. HCS is effective when card receives SEND_IF_COND command.
 * Sends request to switch to 1.8V signaling (S18R). Reserved bit shall be set
 * to '0'. CCS bit is assigned to OCR[30].
 * XPC controls the maximum power in the default speed mode of SDXC card.
 * XPC=0 means 0.36W (100mA at 3.6V on VDD1) (max.) but speed class is not
 * supported. XPC=1 means 0.54W (150mA at 3.6V on VDD1) (max.) and speed class
 * is supported.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_sd_app_op_cond_acmd41(struct device *dev,
                                      struct sdio_app_info *info,
                                      struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_SD_APP_OP_COND;
    cmd->cmd_flags = HC_SDIO_RSP_R3_R4;
    cmd->cmd_type = HC_SDIO_CMD_BCR;
    cmd->cmd_arg = 0;
    ret = device_sdio_send_cmd(dev, cmd);

    /* Check OCR */
    if (ret) { /* OCR fails */
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support get card OCR (ACMD41)\n");
        }
    }

    return ret;
}

/**
 * @brief Get Operation Conditions Register (OCR) with initialization.
 *
 * Send command 41 with argument as initialization.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @return 0 on success, negative errno on error.
 */
static int sdio_sd_app_op_cond_init_acmd41(struct device *dev,
                                           struct sdio_app_info *info,
                                           struct sdio_cmd *cmd)
{
    int ret = 0;

    cmd->cmd = HC_SD_APP_OP_COND;
    cmd->cmd_flags = HC_SDIO_RSP_R3_R4;
    cmd->cmd_type = HC_SDIO_CMD_BCR;
    cmd->cmd_arg = HOST_CAPACITY_SUPPORT | SDXC_POWER_CONTROL | OCR_VDD_33_34;
    ret = device_sdio_send_cmd(dev, cmd);

    /* Check OCR */
    if (ret) { /* OCR fails */
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support get OCR with init (ACMD41)\n");
        }
    }

    return ret;
}

/**
 * @brief Get SD CARD Configuration Register (SCR).
 *
 * Reads the SCR.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @param cmd Pointer to structure of cmd.
 * @param transfer Pointer to structure of transfer.
 * @return 0 on success, negative errno on error.
 */
static int sdio_sd_app_send_scr_acmd51(struct device *dev,
                                       struct sdio_app_info *info,
                                       struct sdio_cmd *cmd,
                                       struct sdio_transfer *transfer)
{
    int ret = 0;
    uint8_t data[SCR_BLOCK_LENGTH] = {0};

    cmd->cmd = HC_SD_APP_SEND_SCR;
    cmd->cmd_flags = HC_SDIO_RSP_R1_R5_R6_R7;
    cmd->cmd_type = HC_SDIO_CMD_ADTC;
    cmd->cmd_arg = 0;
    cmd->data_blocks = SCR_BLOCK_COUNT;
    cmd->data_blksz = SCR_BLOCK_LENGTH;
    ret = device_sdio_send_cmd(dev, cmd);

    if (ret) {
        if (ret == -ETIMEDOUT) {
            printf("sdio_test: not support get card SCR (CMD51)\n");
        }
        return ret;
    } else {
        /* Check response */
        ret = sdio_check_card_status(cmd->resp[0]);
        if (ret) {
            sdio_dump_card_status(cmd->resp[0]);
        }
    }

    /* Clean data_blocks and data_blksz of cmd */
    cmd->data_blocks = 0;
    cmd->data_blksz = 0;

    transfer->blocks = SCR_BLOCK_COUNT;
    transfer->blksz = SCR_BLOCK_LENGTH;
    transfer->data = data;
    ret = device_sdio_read(dev, transfer);

    if (ret) {
        printf("sdio_test: get card SCR fail\n");
    } else {
        printf("sdio_test: get card SCR success\n");
        /*
         * In the CMD line the Most Significant Bit (MSB) is transmitted first,
         * the Least Significant Bit (LSB) is the last.
         */
        info->scr[1] = (data[0] << (BYTE_SHIFT * 3)) |
                       (data[1] << (BYTE_SHIFT * 2)) |
                       (data[2] << BYTE_SHIFT) |
                       data[3];
        info->scr[0] = (data[4] << (BYTE_SHIFT * 3)) |
                       (data[5] << (BYTE_SHIFT * 2)) |
                       (data[6] << BYTE_SHIFT) |
                       data[7];
        printf("sdio_test: info->scr[0] = 0x%08x\n", info->scr[0]);
        printf("sdio_test: info->scr[1] = 0x%08x\n", info->scr[1]);
    }

    return ret;
}

/**
 * @brief Initialize SD card.
 *
 * Initialize and configure SD card to data transfer state.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @return 0 on success, others on error.
 */
static int sdio_sd_initialize(struct device *dev, struct sdio_app_info *info)
{
    struct sdio_ios ios = {0};
    struct sdio_cmd cmd = {0};
    struct sdio_transfer transfer = {0};
    int ret = 0;

    /* Prepare response for command */
    cmd.resp = info->resp;

    /* Get CID (CMD2) */
    ret = sdio_mmc_all_send_cid_cmd2(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to get CID (CMD2)\n");
        return ret;
    }

    /* Get RCA (CMD3) */
    ret = sdio_sd_send_relative_addr_cmd3(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to get RCA (CMD3)\n");
        return ret;
    }

    /* Get card addressed CID */
    ret = sdio_mmc_send_cid_cmd10(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to get card addressed CID (CMD10)\n");
        return ret;
    }

    /* Get card CSD */
    ret = sdio_mmc_send_csd_cmd9(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to get card CSD (CMD9)\n");
        return ret;
    }

    /* Select card */
    ret = sdio_mmc_select_card_cmd7(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to select card (CMD7)\n");
        return ret;
    }

    /*
     * Indicates to the card that the next command is an application specific
     * command rather than a standard command
     */
    ret = sdio_mmc_app_cmd_rca_cmd55(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to indicate app specific command (CMD55)\n");
        return ret;
    }

    /* Get card SCR */
    ret = sdio_sd_app_send_scr_acmd51(dev, info, &cmd, &transfer);
    if (ret) {
        printf("sdio_test: fail to get card SCR (CMD51)\n");
        return ret;
    }

    /*
     * Indicates to the card that the next command is an application specific
     * command rather than a standard command
     */
    ret = sdio_mmc_app_cmd_rca_cmd55(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to indicate app specific command (CMD55)\n");
        return ret;
    }

    /* Unlock card */
    ret = sdio_mmc_lock_unlock_cmd42(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to unlock card (CMD42)\n");
        return ret;
    }

    /*
     * Indicates to the card that the next command is an application specific
     * command rather than a standard command
     */
    ret = sdio_mmc_app_cmd_rca_cmd55(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to indicate app specific command (CMD55)\n");
        return ret;
    }

    /* Set bus width of the SD card to 4 bit mode */
    ret = sdio_sd_app_set_bus_width_acmd6(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to set bus width to 4 bit mode (ACMD6)\n");
        return ret;
    }

    /* set clock to 25MHz clock frequency and bus width to 4 bit mode */
    ret = sdio_mmc_power_on(dev, &ios);
    if (ret) {
        printf("sdio_test: fail to set clock to 25MHz clock frequency\n");
        return ret;
    }

    /* Get card status */
    ret = sdio_mmc_send_status_cmd13(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to get card status (CMD13)\n");
        return ret;
    }

    return ret;
}

/**
 * @brief Card identify.
 *
 * Initialization and card identification sequence for the Standard Capacity SD
 * Memory Card (SDSC), the High Capacity SD Memory Card (SDHC) and the Extended
 * Capacity SD Memory Card (SDXC).
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @return 0 on success, others on error.
 */
static int sdio_card_identify(struct device *dev, struct sdio_app_info *info)
{
    struct sdio_cap cap = {0};
    struct sdio_ios ios = {0};
    struct sdio_cmd cmd = {0};
    int ret = 0;
    uint8_t retry = 0;

    /* Prepare response for command */
    cmd.resp = info->resp;

    /* Get capabilities of the SD host controller */
    ret = device_sdio_get_capabilities(dev, &cap);
    if (ret) {
        printf("sdio_test: fail to get host controller capabilities\n");
        return ret;
    }
    info->cap = cap;
    sdio_dump_capabilities(&cap);

    /*
     * Set clock to 400KHz clock frequency and bus width to 1 bit mode for card
     * identify phase
     */
    ret = sdio_mmc_power_up(dev, &ios);
    if (ret) {
        printf("sdio_test: fail to set clock to 400KHz clock frequency\n");
        return ret;
    }

    /* Reset Card (CMD0) */
    ret = sdio_mmc_go_idle_state_cmd0(dev, &cmd);
    if (ret) {
        printf("sdio_test: fail to reset card (CMD0)\n");
        return ret;
    }

    /* Voltage Check (CMD8) */
    ret = sdio_sd_send_if_cond_cmd8(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to voltage check (CMD8)\n");
        return ret;
    }

    /*
     * Indicates to the card that the next command is an application specific
     * command rather than a standard command
     */
    ret = sdio_mmc_app_cmd_cmd55(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to indicate app specific command (CMD55)\n");
        return ret;
    }

    /* Get OCR (ACMD41) Voltage Window = 0 */
    ret = sdio_sd_app_op_cond_acmd41(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to get OCR (ACMD41)\n");
        return ret;
    }

    /* Check F8 Flag */
    if (info->F8) {
        /*
         * Check busy status in the response of ACMD41. Repeat to issue ACMD41
         * while busy is indicated. The interval of ACMD41 shall be less than
         * 50ms. Detecting timeout of 1 second exits the loop.
         */
        while ((!(cmd.resp[0] & BUSY_STATUS)) && (retry < ACMD41_MAX_RETRY)) {
            usleep(ACMD41_INTERVAL);

            /*
             * Indicates to the card that the next command is an application
             * specific command rather than a standard command
             */
            ret = sdio_mmc_app_cmd_cmd55(dev, info, &cmd);
            if (ret) {
                printf("sdio_test: fail to issue app command (CMD55)\n");
                return ret;
            }

            /* Get OCR (ACMD41) Voltage with initialization */
            ret = sdio_sd_app_op_cond_init_acmd41(dev, info, &cmd);
            if (ret) {
                printf("sdio_test: fail to get OCR (ACMD41) with init\n");
                return ret;
            }
            retry++;
        }

        if (retry == ACMD41_MAX_RETRY) { /* Detect timeout of 1 second */
            printf("sdio_test: timeout to get OCR (ACMD41) with init\n");
            return ret;
        }

        /* Check CCS (Card Capacity Status) */
        if (cmd.resp[0] & CARD_CAPACITY_STATUS) { /* CCS=1 */
            printf("sdio_test: ## This is SDHC or SDXC card ##\n");
        } else { /* CCS=0 */
            printf("sdio_test: ## This is SDSC Ver.2.00 or Ver3.00 card ##\n");
        }

        ret = sdio_sd_initialize(dev, info);
        if (ret) {
            printf("sdio_test: fail to initialize SD card\n");
            return ret;
        }
    }

    return ret;
}

/**
 * @brief SDIO mmc test basic write.
 *
 * This function is to verify card identity, write single block data to SD card.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @return 0 on success, others on error.
 */
static int sdio_mmc_test_basic_write(struct device *dev,
                                     struct sdio_app_info *info)
{
    int ret = 0;
    struct sdio_cmd cmd = {0};
    struct sdio_transfer transfer = {0};

    /* Prepare response for command */
    cmd.resp = info->resp;

    ret = sdio_card_identify(dev, info);
    if (ret) {
        printf("sdio_test: fail to sdio_card_identify\n");
        return ret;
    }

    /* Set block size */
    info->blocklen = BLOCK_LENGTH;
    ret = sdio_mmc_set_blocklen_cmd16(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to set block size (CMD16)\n");
        return ret;
    }

    /* Write block */
    ret = sdio_mmc_write_block_cmd24(dev, info, &cmd, &transfer);
    if (ret) {
        printf("sdio_test: fail to write block (CMD24)\n");
        return ret;
    }

    return ret;
}

/**
 * @brief SDIO mmc test multiple write.
 *
 * This function is to verify card identity, write multiple blocks data to SD
 * card.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @return 0 on success, others on error.
 */
static int sdio_mmc_test_multi_write(struct device *dev,
                                     struct sdio_app_info *info)
{
    int ret = 0;
    struct sdio_cmd cmd = {0};
    struct sdio_transfer transfer = {0};

    /* Prepare response for command */
    cmd.resp = info->resp;

    ret = sdio_card_identify(dev, info);
    if (ret) {
        printf("sdio_test: fail to sdio_card_identify\n");
        return ret;
    }

    /* Set block size */
    info->blocklen = BLOCK_LENGTH;
    ret = sdio_mmc_set_blocklen_cmd16(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to set block size (CMD16)\n");
        return ret;
    }

    /* Set block count */
    info->blockcnt = BLOCK_COUNT;
    ret = sdio_mmc_set_block_count_cmd23(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to set block count (CMD23)\n");
        return ret;
    }

    /* Write blocks */
    ret = sdio_mmc_write_multiple_blocks_cmd25(dev, info, &cmd, &transfer);
    if (ret) {
        printf("sdio_test: fail to write blocks (CMD25)\n");
        return ret;
    }

    return ret;
}

/**
 * @brief SDIO mmc test basic read.
 *
 * This function is to verify card identity, read single block data from SD
 * card.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @return 0 on success, others on error.
 */
static int sdio_mmc_test_basic_read(struct device *dev,
                                    struct sdio_app_info *info)
{
    int ret = 0;
    struct sdio_cmd cmd = {0};
    struct sdio_transfer transfer = {0};

    /* Prepare response for command */
    cmd.resp = info->resp;

    ret = sdio_card_identify(dev, info);
    if (ret) {
        printf("sdio_test: fail to sdio_card_identify\n");
        return ret;
    }

    /* Set block size */
    info->blocklen = BLOCK_LENGTH;
    ret = sdio_mmc_set_blocklen_cmd16(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to set block size (CMD16)\n");
        return ret;
    }

    /* Read block */
    ret = sdio_mmc_read_single_block_cmd17(dev, info, &cmd, &transfer);
    if (ret) {
        printf("sdio_test: fail to read block (CMD17)\n");
        return ret;
    }

    return ret;
}

/**
 * @brief SDIO mmc test multiple read.
 *
 * This function is to verify card identity, read multiple blocks data from SD
 * card.
 *
 * @param dev Pointer to structure of device.
 * @param info Pointer to structure of info.
 * @return 0 on success, others on error.
 */
static int sdio_mmc_test_multi_read(struct device *dev,
                                    struct sdio_app_info *info)
{
    int ret = 0;
    struct sdio_cmd cmd = {0};
    struct sdio_transfer transfer = {0};

    /* Prepare response for command */
    cmd.resp = info->resp;

    ret = sdio_card_identify(dev, info);
    if (ret) {
        printf("sdio_test: fail to sdio_card_identify\n");
        return ret;
    }

    /* Set block size */
    info->blocklen = BLOCK_LENGTH;
    ret = sdio_mmc_set_blocklen_cmd16(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to set block size (CMD16)\n");
        return ret;
    }

    /* Set block count */
    info->blockcnt = BLOCK_COUNT;
    ret = sdio_mmc_set_block_count_cmd23(dev, info, &cmd);
    if (ret) {
        printf("sdio_test: fail to set block count (CMD23)\n");
        return ret;
    }

    /* Read blocks */
    ret = sdio_mmc_read_multiple_blocks_cmd18(dev, info, &cmd, &transfer);
    if (ret) {
        printf("sdio_test: fail to read blocks (CMD18)\n");
        return ret;
    }

    return ret;
}

/**
* @brief Entry of the SDIO test application.
*
* This is main function to verify those SDIO device driver APIs and SD card
* identify.
* 1. device_sdio_get_capabilities()
* 2. device_sdio_set_ios()
* 3. device_sdio_send_cmd()
* 4. device_sdio_write()
* 5. device_sdio_read()
*
* @param argc Indicate how many input parameters.
* @param argv Commands input from console.
* @return 0 on success, others on error.
*/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[]) {
#else
int sdio_test_main(int argc, char *argv[]) {
#endif
    struct sdio_app_info *info = NULL;
    struct sdio_cap cap = {0};
    struct sdio_ios ios = {0};
    struct sdio_cmd cmd = {0};
    struct device *dev;
    int ret = 0;

    info = zalloc(sizeof(struct sdio_app_info));
    if (!info) {
        printf("sdio_test: SDIO memory allocate fail\n");
        return EXIT_FAILURE;
    }

    sdio_default_params(info);

    /* Prepare response for command */
    cmd.resp = info->resp;

    ret = sdio_cmd_parser(info, argc, argv);
    if (ret) {
        ret = EXIT_FAILURE;
        sdio_print_usage();
        goto err_free_info;
    }

    dev = device_open(DEVICE_TYPE_SDIO_HW, 0);
    if (!dev) {
        printf("sdio_test: SDIO device open fail\n");
        ret = EXIT_FAILURE;
        goto err_free_info;
    }

    switch (info->operation) {
    case SDIO_GET_CAPABILITY:
        ret = device_sdio_get_capabilities(dev, &cap);
        if (!ret) {
            sdio_dump_capabilities(&cap);
        }
        break;
    case SDIO_MMC_POWER_UP:
        ret = sdio_mmc_power_up(dev, &ios);
        break;
    case SDIO_SET_CARD_TO_IDLE_STATE:
        ret = sdio_mmc_go_idle_state_cmd0(dev, &cmd);
        break;
    case SDIO_WRITE:
        if (info->data_mode == SINGLE_MODE) {
            ret = sdio_mmc_test_basic_write(dev, info);
        } else {
            ret = sdio_mmc_test_multi_write(dev, info);
        }
        break;
    case SDIO_READ:
        if (info->data_mode == SINGLE_MODE) {
            ret = sdio_mmc_test_basic_read(dev, info);
        } else {
            ret = sdio_mmc_test_multi_read(dev, info);
        }
        break;
    case SDIO_CARD_IDENTIFY:
        ret = sdio_card_identify(dev, info);
        break;
    default:
        ret = EXIT_FAILURE;
    }

    device_close(dev);

err_free_info:
    free(info);

    printf("sdio_test: test finish, result: 0x%x\n", ret);
    return ret;
}
