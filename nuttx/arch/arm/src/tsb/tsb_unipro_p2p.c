/**
 * Copyright (c) 2014-2015 Google Inc.
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
 * @author Perry Hung
 * @brief MIPI UniPro stack for APBridge ES1
 */
#include <string.h>
#include <nuttx/util.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>

#include <arch/tsb/unipro.h>
#include <arch/tsb/irq.h>
#include <errno.h>

#include "debug.h"
#include "up_arch.h"
#include "tsb_scm.h"
#include "tsb_unipro_es2.h"
#include "tsb_es2_mphy_fixups.h"

//#define UNIPRO_DEBUG_ATTRIBUTES
#ifdef UNIPRO_DEBUG_ATTRIBUTES
    #define DBG_ATTRIBUTE(attr, cportid, peer) do { \
        uint32_t val; \
        uint32_t rc; \
        unipro_attr_read(attr, &val, cportid, peer, &rc); \
        lldbg("%04x %s: 0x%x\n", attr, #attr, val); \
    } while (0);
#else
    #define DBG_ATTRIBUTE(attr, cportid, peer) do { \
    } while (0);
#endif

//#define UNIPRO_DEBUG_REGISTERS
#ifdef UNIPRO_DEBUG_REGISTERS
    #define DBG_REG(reg) do { \
        lldbg("%08x %s: 0x%x\n", reg, #reg, unipro_read(reg)); \
    } while (0)

    #define DBG_REG_IDX(reg, idx) do { \
        lldbg("%08x %s[%d]: 0x%x\n", (reg + i * 4), #reg, idx, unipro_read((reg + i * 4))); \
    } while (0)
#else
    #define DBG_REG(reg) do { \
    } while (0)

    #define DBG_REG_IDX(reg, idx) do { \
    } while (0)
#endif

#define ENABLE_UNIPRO_IRQ (0)

#if ENABLE_UNIPRO_IRQ
    static void unipro_irq_attach(void);
    static int unipro_irq(int irq, void *context);
#endif

uint32_t unipro_read(uint32_t offset) {
    return getreg32((volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}

void unipro_write(uint32_t offset, uint32_t v) {
    putreg32(v, (volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}

void unipro_powermode_change(uint8_t txgear, uint8_t rxgear, uint8_t pwrmode, uint8_t series, uint8_t termination) {
    uint32_t result_code;
    uint32_t attr_val;

    /* Table 5-26 UniPro Power Mode Change Part 1 */
    lldbg("5-26\n");
    unipro_write(UNIPRO_INT_EN, 0x1);

    /* Table 5-27 UniPro Power Mode Change Part 2 (See 5.7.7.2 UniPro Attribute Access */
    lldbg("5-27\n");
    unipro_attr_write(TSB_INTERRUPTENABLE, 0x20, 0 /* selector */, 0 /* peer */, &result_code);

    unipro_attr_write(PA_TXGEAR, txgear, 0 /* selector */, 0 /* peer */, &result_code);
    unipro_attr_write(PA_TXTERMINATION, termination, 0 /* selector */, 0 /* peer */, &result_code);
    unipro_attr_write(PA_HSSERIES, series, 0 /* selector */, 0 /* peer */, &result_code);
    unipro_attr_write(PA_ACTIVETXDATALANES, 1, 0 /* selector */, 0 /* peer */, &result_code);

    unipro_attr_write(PA_RXGEAR, rxgear, 0 /* selector */, 0 /* peer */, &result_code);
    unipro_attr_write(PA_RXTERMINATION, termination, 0 /* selector */, 0 /* peer */, &result_code);
    unipro_attr_write(PA_ACTIVERXDATALANES, 1, 0 /* selector */, 0 /* peer */, &result_code);


    unipro_attr_write(PA_PWRMODE, pwrmode, 0 /* selector */, 0 /* peer */, &result_code);

    /* Table 5-28 UniPro Power Mode Change Part 3 */
    lldbg("5-28: wait for powermode\n");
    while (!(unipro_read(UNIPRO_INT_BEF) & 0x1)) {
    }
    lldbg("powermode change done\n");

    /* Table 5-29 Check UniPro Power Mode Change (See 5.7.7.2 UniPro Attribute Access) */
    lldbg("5-29\n");
    unipro_attr_read(TSB_DME_POWERMODEIND, &attr_val, 0, 0, &result_code);
    lldbg("got=%d, expected=%d\n", attr_val, 2);
    unipro_attr_read(TSB_DME_POWERMODEIND, &attr_val, 0, 1, &result_code);
    lldbg("got=%d, expected=%d\n", attr_val, 4);
}

/**
 * @brief Stop the UniPro auto-link
 */
void unipro_stop(void) {
    unipro_write(LUP_LINKSUP_STOP, 0x1);
}

/**
 * @brief Restart the UniPro auto-link
 */
void unipro_restart(void) {
    unipro_write(LUP_LINKSUP_RESTART, 0x1);
}

void unipro_reset(void) {
    tsb_reset(TSB_RST_UNIPROSYS);
}

void unipro_p2p_setup(void) {
    uint32_t i;
    uint32_t result_code;
//    uint32_t attr_val;

    for (i=0; i < CPORT_MAX; i++) {

        lldbg("attributes for cport=%d\n", i);

        /* Disable CPORT before making changes */
        unipro_attr_write(T_CONNECTIONSTATE, 0, i /* selector */, 0 /* peer */, &result_code);

        unipro_attr_write(T_PEERDEVICEID, CONFIG_UNIPRO_REMOTE_DEVICEID, i /* selector */, 0 /* peer */, &result_code);
        unipro_attr_write(T_PEERCPORTID, i, i /* selector */, 0 /* peer */, &result_code);

        unipro_attr_write(T_TRAFFICCLASS, 0, i /* selector */, 0 /* peer */, &result_code);
        unipro_attr_write(T_TXTOKENVALUE, 0x20, i /* selector */, 0 /* peer */, &result_code);
        unipro_attr_write(T_RXTOKENVALUE, 0x20, i /* selector */, 0 /* peer */, &result_code);
        unipro_attr_write(T_CPORTFLAGS, 2 /*CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N*/, i /* selector */, 0 /* peer */, &result_code);

        unipro_attr_write(T_CPORTMODE, CPORT_MODE_APPLICATION, i /* selector */, 0 /* peer */, &result_code);
        unipro_attr_write(T_CREDITSTOSEND, 0, i /* selector */, 0 /* peer */, &result_code);

//        unipro_attr_read(T_LOCALBUFFERSPACE, &attr_val, i /* selector */, 1 /* peer */, &result_code);
//        unipro_attr_write(T_PEERBUFFERSPACE, attr_val, i /* selector */, 0 /* peer */, &result_code);

        unipro_attr_write(TSB_MAXSEGMENTCONFIG, 0x118, i /* selector */, 0 /* peer */, &result_code);

        /* Enable CPORT */
        unipro_attr_write(T_CONNECTIONSTATE, 1, i /* selector */, 0 /* peer */, &result_code);
    }

#ifdef CONFIG_ICE_CORE
//    unipro_powermode_change(1 /* txgear */, 1 /* rxgear */, 0x22 /* PWM */, 0 /*series */, 1 /* termination */);
    unipro_powermode_change(2 /* txgear */, 2 /* rxgear */, 0x11 /* HS */, 0 /*series */, 1 /* termination */);
#else
    unipro_attr_write(PA_TXTERMINATION, 1, 0 /* selector */, 0 /* peer */, &result_code);
    unipro_attr_write(PA_RXTERMINATION, 1, 0 /* selector */, 0 /* peer */, &result_code);
#endif
}

#if ENABLE_UNIPRO_IRQ
static void unipro_irq_attach(void) {
    irq_attach(TSB_IRQ_UNIPRO, unipro_irq);
    up_enable_irq(TSB_IRQ_UNIPRO);
}

static int unipro_irq(int irq, void *context) {
    uint32_t val;

    DBG_UNIPRO("%s\n", __func__);
    unipro_dump_ints();
    unipro_dump_status();

    val = unipro_read(UNIPRO_INT_AFT);
    if (val) {
        unipro_write(UNIPRO_INT_BEF, val);
    }

    val = unipro_read(LUP_INT_AFT);
    if (val) {
        unipro_write(LUP_INT_BEF, val);
    }

    val = unipro_read(A2D_ATTRACS_INT_AFT);
    if (val) {
        unipro_write(A2D_ATTRACS_INT_BEF, val );
    }

    return 0;
}
#endif

/*
 * debug interfaces
 */
void unipro_dump_attributes(int peer) {
    /*
     * L1 attributes
     */
    DBG_ATTRIBUTE(TX_HSMODE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_HSGEAR_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_PWMG0_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_PWMGEAR_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_AMPLITUDE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_EXTERNALSYNC_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_HS_UNTERMINATED_LINE_DRIVE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_LS_TERMINATED_LINE_DRIVE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_MIN_SLEEP_NOCONFIG_TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_MIN_STALL_NOCONFIG_TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_MIN_SAVE_CONFIG_TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_REF_CLOCK_SHARED_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_PHY_MAJORMINOR_RELEASE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_PHY_EDITORIAL_RELEASE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_HIBERN8TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_ADVANCED_GRANULARITY_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_ADVANCED_HIBERN8TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_HS_EQUALIZER_SETTING_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(TX_MODE, 0, peer);
    DBG_ATTRIBUTE(TX_HSRATE_SERIES, 0, peer);
    DBG_ATTRIBUTE(TX_HSGEAR, 0, peer);
    DBG_ATTRIBUTE(TX_PWMGEAR, 0, peer);
    DBG_ATTRIBUTE(TX_AMPLITUDE, 0, peer);
    DBG_ATTRIBUTE(TX_HS_SLEWRATE, 0, peer);
    DBG_ATTRIBUTE(TX_SYNC_SOURCE, 0, peer);
    DBG_ATTRIBUTE(TX_HS_SYNC_LENGTH, 0, peer);
    DBG_ATTRIBUTE(TX_HS_PREPARE_LENGTH, 0, peer);
    DBG_ATTRIBUTE(TX_LS_PREPARE_LENGTH, 0, peer);
    DBG_ATTRIBUTE(TX_HIBERN8_CONTROL, 0, peer);
    DBG_ATTRIBUTE(TX_LCC_ENABLE, 0, peer);
    DBG_ATTRIBUTE(TX_PWM_BURST_CLOSURE_EXTENSION, 0, peer);
    DBG_ATTRIBUTE(TX_BYPASS_8B10B_ENABLE, 0, peer);
    DBG_ATTRIBUTE(TX_DRIVER_POLARITY, 0, peer);
    DBG_ATTRIBUTE(TX_HS_UNTERMINATED_LINE_DRIVE_ENABLE, 0, peer);
    DBG_ATTRIBUTE(TX_LS_TERMINATED_LINE_DRIVE_ENABLE, 0, peer);
    DBG_ATTRIBUTE(TX_LCC_SEQUENCER, 0, peer);
    DBG_ATTRIBUTE(TX_MIN_ACTIVATETIME, 0, peer);
    DBG_ATTRIBUTE(TX_ADVANCED_GRANULARITY_SETTING, 0, peer);
    DBG_ATTRIBUTE(TX_ADVANCED_GRANULARITY, 0, peer);
    DBG_ATTRIBUTE(TX_HS_EQUALIZER_SETTING, 0, peer);
    DBG_ATTRIBUTE(TX_FSM_STATE, 0, peer);
    DBG_ATTRIBUTE(MC_OUTPUT_AMPLITUDE, 0, peer);
    DBG_ATTRIBUTE(MC_HS_UNTERMINATED_ENABLE, 0, peer);
    DBG_ATTRIBUTE(MC_LS_TERMINATED_ENABLE, 0, peer);
    DBG_ATTRIBUTE(MC_HS_UNTERMINATED_LINE_DRIVE_ENABLE, 0, peer);
    DBG_ATTRIBUTE(MC_LS_TERMINATED_LINE_DRIVE_ENABLE, 0, peer);
    DBG_ATTRIBUTE(RX_HSMODE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_HSGEAR_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_PWMG0_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_PWMGEAR_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_HS_UNTERMINATED_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_LS_TERMINATED_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_MIN_SLEEP_NOCONFIG_TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_MIN_STALL_NOCONFIG_TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_MIN_SAVE_CONFIG_TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_REF_CLOCK_SHARED_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_HS_G1_SYNC_LENGTH_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_HS_G1_PREPARE_LENGTH_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_LS_PREPARE_LENGTH_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_PWM_BURST_CLOSURE_LENGTH_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_MIN_ACTIVATETIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_PHY_MAJORMINOR_RELEASE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_PHY_EDITORIAL_RELEASE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_HIBERN8TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_HS_G2_SYNC_LENGTH_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_HS_G3_SYNC_LENGTH_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_HS_G2_PREPARE_LENGTH_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_HS_G3_PREPARE_LENGTH_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_ADVANCED_GRANULARITY_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_ADVANCED_HIBERN8TIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_ADVANCED_MIN_ACTIVATETIME_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(RX_MODE, 0, peer);
    DBG_ATTRIBUTE(RX_HSRATE_SERIES, 0, peer);
    DBG_ATTRIBUTE(RX_HSGEAR, 0, peer);
    DBG_ATTRIBUTE(RX_PWMGEAR, 0, peer);
    DBG_ATTRIBUTE(RX_LS_TERMINATED_ENABLE, 0, peer);
    DBG_ATTRIBUTE(RX_HS_UNTERMINATED_ENABLE, 0, peer);
    DBG_ATTRIBUTE(RX_ENTER_HIBERN8, 0, peer);
    DBG_ATTRIBUTE(RX_BYPASS_8B10B_ENABLE, 0, peer);
    DBG_ATTRIBUTE(RX_TERMINATION_FORCE_ENABLE, 0, peer);
    DBG_ATTRIBUTE(RX_FSM_STATE, 0, peer);
    DBG_ATTRIBUTE(OMC_TYPE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_HSMODE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_HSGEAR_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_HS_START_TIME_VAR_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_HS_START_TIME_RANGE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_RX_SA_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_RX_LA_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_LS_PREPARE_LENGTH, 0, peer);
    DBG_ATTRIBUTE(MC_PWMG0_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_PWMGEAR_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_LS_TERMINATED_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_HS_UNTERMINATED_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_LS_TERMINATED_LINE_DRIVE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_HS_UNTERMINATED_LINE_DRIVE_CAPABILIT, 0, peer);
    DBG_ATTRIBUTE(MC_MFG_ID_PART1, 0, peer);
    DBG_ATTRIBUTE(MC_MFG_ID_PART2, 0, peer);
    DBG_ATTRIBUTE(MC_PHY_MAJORMINOR_RELEASE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_PHY_EDITORIAL_RELEASE_CAPABILITY, 0, peer);
    DBG_ATTRIBUTE(MC_VENDOR_INFO_PART1, 0, peer);
    DBG_ATTRIBUTE(MC_VENDOR_INFO_PART2, 0, peer);
    DBG_ATTRIBUTE(MC_VENDOR_INFO_PART3, 0, peer);
    DBG_ATTRIBUTE(MC_VENDOR_INFO_PART4, 0, peer);

    /*
     * L1.5 attributes
     */
    DBG_ATTRIBUTE(PA_PHYTYPE, 0, peer);
    DBG_ATTRIBUTE(PA_AVAILTXDATALANES, 0, peer);
    DBG_ATTRIBUTE(PA_AVAILRXDATALANES, 0, peer);
    DBG_ATTRIBUTE(PA_MINRXTRAILINGCLOCKS, 0, peer);
    DBG_ATTRIBUTE(PA_TXHSG1SYNCLENGTH, 0, peer);
    DBG_ATTRIBUTE(PA_TXHSG1PREPARELENGTH, 0, peer);
    DBG_ATTRIBUTE(PA_TXHSG2SYNCLENGTH, 0, peer);
    DBG_ATTRIBUTE(PA_TXHSG2PREPARELENGTH, 0, peer);
    DBG_ATTRIBUTE(PA_TXHSG3SYNCLENGTH, 0, peer);
    DBG_ATTRIBUTE(PA_TXHSG3PREPARELENGTH, 0, peer);
    DBG_ATTRIBUTE(PA_TXMK2EXTENSION, 0, peer);
    DBG_ATTRIBUTE(PA_PEERSCRAMBLING, 0, peer);
    DBG_ATTRIBUTE(PA_TXSKIP, 0, peer);
    DBG_ATTRIBUTE(PA_SAVECONFIGEXTENSIONENABLE, 0, peer);
    DBG_ATTRIBUTE(PA_LOCALTXLCCENABLE, 0, peer);
    DBG_ATTRIBUTE(PA_PEERTXLCCENABLE, 0, peer);
    DBG_ATTRIBUTE(PA_ACTIVETXDATALANES, 0, peer);
    DBG_ATTRIBUTE(PA_CONNECTEDTXDATALANES, 0, peer);
    DBG_ATTRIBUTE(PA_TXTRAILINGCLOCKS, 0, peer);
    DBG_ATTRIBUTE(PA_TXPWRSTATUS, 0, peer);
    DBG_ATTRIBUTE(PA_TXGEAR, 0, peer);
    DBG_ATTRIBUTE(PA_TXTERMINATION, 0, peer);
    DBG_ATTRIBUTE(PA_HSSERIES, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODE, 0, peer);
    DBG_ATTRIBUTE(PA_ACTIVERXDATALANES, 0, peer);
    DBG_ATTRIBUTE(PA_CONNECTEDRXDATALANES, 0, peer);
    DBG_ATTRIBUTE(PA_RXPWRSTATUS, 0, peer);
    DBG_ATTRIBUTE(PA_RXGEAR, 0, peer);
    DBG_ATTRIBUTE(PA_RXTERMINATION, 0, peer);
    DBG_ATTRIBUTE(PA_SCRAMBLING, 0, peer);
    DBG_ATTRIBUTE(PA_MAXRXPWMGEAR, 0, peer);
    DBG_ATTRIBUTE(PA_MAXRXHSGEAR, 0, peer);
    DBG_ATTRIBUTE(PA_PACPREQTIMEOUT, 0, peer);
    DBG_ATTRIBUTE(PA_PACPREQEOBTIMEOUT, 0, peer);
    DBG_ATTRIBUTE(PA_REMOTEVERINFO, 0, peer);
    DBG_ATTRIBUTE(PA_LOGICALLANEMAP, 0, peer);
    DBG_ATTRIBUTE(PA_SLEEPNOCONFIGTIME, 0, peer);
    DBG_ATTRIBUTE(PA_STALLNOCONFIGTIME, 0, peer);
    DBG_ATTRIBUTE(PA_SAVECONFIGTIME, 0, peer);
    DBG_ATTRIBUTE(PA_RXHSUNTERMINATIONCAPABILITY, 0, peer);
    DBG_ATTRIBUTE(PA_RXLSTERMINATIONCAPABILITY, 0, peer);
    DBG_ATTRIBUTE(PA_HIBERN8TIME, 0, peer);
    DBG_ATTRIBUTE(PA_TACTIVATE, 0, peer);
    DBG_ATTRIBUTE(PA_LOCALVERINFO, 0, peer);
    DBG_ATTRIBUTE(PA_GRANULARITY, 0, peer);
    DBG_ATTRIBUTE(PA_MK2EXTENSIONGUARDBAND, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA0, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA1, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA2, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA3, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA4, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA5, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA6, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA7, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA8, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA9, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA10, 0, peer);
    DBG_ATTRIBUTE(PA_PWRMODEUSERDATA11, 0, peer);
    DBG_ATTRIBUTE(PA_PACPFRAMECOUNT, 0, peer);
    DBG_ATTRIBUTE(PA_PACPERRORCOUNT, 0, peer);
    DBG_ATTRIBUTE(PA_PHYTESTCONTROL, 0, peer);

    /*
     * L2 attributes
     */
    DBG_ATTRIBUTE(DL_TXPREEMPTIONCAP, 0, peer);
    DBG_ATTRIBUTE(DL_TC0TXMAXSDUSIZE, 0, peer);
    DBG_ATTRIBUTE(DL_TC0RXINITCREDITVAL, 0, peer);
    DBG_ATTRIBUTE(DL_TC1TXMAXSDUSIZE, 0, peer);
    DBG_ATTRIBUTE(DL_TC1RXINITCREDITVAL, 0, peer);
    DBG_ATTRIBUTE(DL_TC0TXBUFFERSIZE, 0, peer);
    DBG_ATTRIBUTE(DL_TC1TXBUFFERSIZE, 0, peer);
    DBG_ATTRIBUTE(DL_TC0TXFCTHRESHOLD, 0, peer);
    DBG_ATTRIBUTE(DL_FC0PROTECTIONTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DL_TC0REPLAYTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DL_AFC0REQTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DL_AFC0CREDITTHRESHOLD, 0, peer);
    DBG_ATTRIBUTE(DL_TC0OUTACKTHRESHOLD, 0, peer);
    DBG_ATTRIBUTE(DL_PEERTC0PRESENT, 0, peer);
    DBG_ATTRIBUTE(DL_PEERTC0RXINITCREDITVAL, 0, peer);
    DBG_ATTRIBUTE(DL_TC1TXFCTHRESHOLD, 0, peer);
    DBG_ATTRIBUTE(DL_FC1PROTECTIONTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DL_TC1REPLAYTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DL_AFC1REQTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DL_AFC1CREDITTHRESHOLD, 0, peer);
    DBG_ATTRIBUTE(DL_TC1OUTACKTHRESHOLD, 0, peer);
    DBG_ATTRIBUTE(DL_PEERTC1PRESENT, 0, peer);
    DBG_ATTRIBUTE(DL_PEERTC1RXINITCREDITVAL, 0, peer);

    /*
     * L3 attributes
     */
    DBG_ATTRIBUTE(N_DEVICEID, 0, peer);
    DBG_ATTRIBUTE(N_DEVICEID_VALID, 0, peer);
    DBG_ATTRIBUTE(N_TC0TXMAXSDUSIZE, 0, peer);
    DBG_ATTRIBUTE(N_TC1TXMAXSDUSIZE, 0, peer);

    /*
     * L4 attributes
     */
    DBG_ATTRIBUTE(T_NUMCPORTS, 0, peer);
    DBG_ATTRIBUTE(T_NUMTESTFEATURES, 0, peer);
    DBG_ATTRIBUTE(T_TC0TXMAXSDUSIZE, 0, peer);
    DBG_ATTRIBUTE(T_TC1TXMAXSDUSIZE, 0, peer);
    DBG_ATTRIBUTE(T_TSTCPORTID, 0, peer);
    DBG_ATTRIBUTE(T_TSTSRCON, 0, peer);
    DBG_ATTRIBUTE(T_TSTSRCPATTERN, 0, peer);
    DBG_ATTRIBUTE(T_TSTSRCINCREMENT, 0, peer);
    DBG_ATTRIBUTE(T_TSTSRCMESSAGESIZE, 0, peer);
    DBG_ATTRIBUTE(T_TSTSRCMESSAGECOUNT, 0, peer);
    DBG_ATTRIBUTE(T_TSTSRCINTERMESSAGEGAP, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTON, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTERRORDETECTIONENABLE, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTPATTERN, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTINCEMENT, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTMESSAGECOUNT, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTMESSAGEOFFSET, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTMESSAGESIZE, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTFCCREDITS, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTINTERFCTOKENGAP, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTINITIALFCCREDITS, 0, peer);
    DBG_ATTRIBUTE(T_TSTDSTERRORCODE, 0, peer);

    /*
     * DME attributes
     */
    DBG_ATTRIBUTE(DME_DDBL1_REVISION, 0, peer);
    DBG_ATTRIBUTE(DME_DDBL1_LEVEL, 0, peer);
    DBG_ATTRIBUTE(DME_DDBL1_DEVICECLASS, 0, peer);
    DBG_ATTRIBUTE(DME_DDBL1_MANUFACTURERID, 0, peer);
    DBG_ATTRIBUTE(DME_DDBL1_PRODUCTID, 0, peer);
    DBG_ATTRIBUTE(DME_DDBL1_LENGTH, 0, peer);
    DBG_ATTRIBUTE(DME_FC0PROTECTIONTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DME_TC0REPLAYTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DME_AFC0REQTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DME_FC1PROTECTIONTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DME_TC1REPLAYTIMEOUTVAL, 0, peer);
    DBG_ATTRIBUTE(DME_AFC1REQTIMEOUTVAL, 0, peer);

    /*
     * TSB attributes
     */

    DBG_ATTRIBUTE(TSB_T_REGACCCTRL_TESTONLY, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_DDBL2_A, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_DDBL2_B, 0, peer);
    DBG_ATTRIBUTE(TSB_MAILBOX, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_LAYERENABLEREQ, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_LAYERENABLECNF, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_RESETREQ, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_RESETCNF, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_ENDPOINTRESETREQ, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_ENDPOINTRESETCNF, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_ENDPOINTRESETIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_LINKSTARTUPREQ, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_LINKSTARTUPCNF, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_LINKSTARTUPIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_LINKLOSTIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_HIBERNATEENTERREQ, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_HIBERNATEENTERCNF, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_HIBERNATEENTERIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_HIBERNATEEXITREQ, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_HIBERNATEEXITCNF, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_HIBERNATEEXITIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_POWERMODEIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_TESTMODEREQ, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_TESTMODECNF, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_TESTMODEIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_ERRORPHYIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_ERRORPAIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_ERRORDIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_ERRORNIND, 0, peer);
    DBG_ATTRIBUTE(TSB_DME_ERRORTIND, 0, peer);
    DBG_ATTRIBUTE(TSB_INTERRUPTENABLE, 0, peer);
    DBG_ATTRIBUTE(TSB_INTERRUPTSTATUS, 0, peer);
    DBG_ATTRIBUTE(TSB_L2STATUS, 0, peer);
    DBG_ATTRIBUTE(TSB_POWERSTATE, 0, peer);
    DBG_ATTRIBUTE(TSB_TXBURSTCLOSUREDELAY, 0, peer);
    DBG_ATTRIBUTE(TSB_MPHYCFGUPDT, 0, peer);
    DBG_ATTRIBUTE(TSB_ADJUSTTRAILINGCLOCKS, 0, peer);
    DBG_ATTRIBUTE(TSB_SUPPRESSRREQ, 0, peer);
    DBG_ATTRIBUTE(TSB_L2TIMEOUT, 0, peer);
    DBG_ATTRIBUTE(TSB_MAXSEGMENTCONFIG, 0, peer);
    DBG_ATTRIBUTE(TSB_TBD, 0, peer);
    DBG_ATTRIBUTE(TSB_RBD, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGTXBYTECOUNT, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGRXBYTECOUNT, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGINVALIDBYTEENABLE, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGLINKSTARTUP, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGPWRCHANGE, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGSTATES, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGCOUNTER0, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGCOUNTER1, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGCOUNTER0MASK, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGCOUNTER1MASK, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGCOUNTERCONTROL, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGCOUNTEROVERFLOW, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGOMC, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGCOUNTERBMASK, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGSAVECONFIGTIME, 0, peer);
    DBG_ATTRIBUTE(TSB_DEBUGCLOCKENABLE, 0, peer);
    DBG_ATTRIBUTE(TSB_DEEPSTALLCFG, 0, peer);
    DBG_ATTRIBUTE(TSB_DEEPSTALLSTATUS, 0, peer);
}

void unipro_dump_cport_attributes(size_t start, size_t end, int peer) {
    size_t i;

    for (i = start; i <= end; i++) {
        lldbg("CP%d:\n", i);

        DBG_ATTRIBUTE(T_PEERDEVICEID, 0, peer);
        DBG_ATTRIBUTE(T_PEERCPORTID, 0, peer);
        DBG_ATTRIBUTE(T_CONNECTIONSTATE, 0, peer);
        DBG_ATTRIBUTE(T_TRAFFICCLASS, 0, peer);
        DBG_ATTRIBUTE(T_PROTOCOLID, 0, peer);
        DBG_ATTRIBUTE(T_CPORTFLAGS, 0, peer);
        DBG_ATTRIBUTE(T_TXTOKENVALUE, 0, peer);
        DBG_ATTRIBUTE(T_RXTOKENVALUE, 0, peer);
        DBG_ATTRIBUTE(T_LOCALBUFFERSPACE, 0, peer);
        DBG_ATTRIBUTE(T_PEERBUFFERSPACE, 0, peer);
        DBG_ATTRIBUTE(T_CREDITSTOSEND, 0, peer);
        DBG_ATTRIBUTE(T_CPORTMODE, 0, peer);
    }
}

/* Attributes as sources of interrupts from the Unipro ports */
static uint16_t unipro_irq_attr[] = {
    TSB_DME_ENDPOINTRESETIND,
    TSB_DME_LINKSTARTUPIND,
    TSB_DME_LINKLOSTIND,
    TSB_DME_HIBERNATEENTERIND,
    TSB_DME_HIBERNATEEXITIND,
    TSB_DME_POWERMODEIND,
    TSB_DME_TESTMODEIND,
    TSB_DME_ERRORPHYIND,
    TSB_DME_ERRORPAIND,
    TSB_DME_ERRORDIND,
    0,                          // Not recommended to read
    TSB_DME_ERRORTIND,
    TSB_DME_ERRORDIND,
    TSB_DEBUGCOUNTEROVERFLOW,
    TSB_DME_LINKSTARTUPCNF,
    TSB_MAILBOX
};

void unipro_dump_status(void) {
    while (1) {
        uint32_t attr_val;
        uint32_t rc;
        size_t j;

        unipro_attr_read(TSB_INTERRUPTSTATUS, &attr_val, 0, 0, &rc);
        if (!attr_val) {
            break;
        }

        lldbg("TSB_INTERRUPTSTATUS, val=0x%x, rc=%d\n", attr_val, rc);
        for (j = 0; j < sizeof(unipro_irq_attr)/sizeof(unipro_irq_attr[0]); j++) {
            if ((attr_val & (1 << j)) && unipro_irq_attr[j]) {
                unipro_attr_read(unipro_irq_attr[j], &attr_val, 0, 0, &rc);
                lldbg("j=%d, attr=0x%x, val=0x%x, rc=%d\n", j, unipro_irq_attr[j], attr_val, rc);
            }
        }
    }
}

/* Attributes as sources of interrupts from the Unipro ports */
void unipro_dump_ints(void) {
    DBG_REG(UNIPRO_INT_EN);
    DBG_REG(UNIPRO_INT_BEF);
    DBG_REG(UNIPRO_INT_AFT);

    DBG_REG(LUP_INT_EN);
    DBG_REG(LUP_INT_BEF);
    DBG_REG(LUP_INT_AFT);

    DBG_REG(A2D_ATTRACS_INT_EN);
    DBG_REG(A2D_ATTRACS_INT_BEF);
    DBG_REG(A2D_ATTRACS_INT_AFT);
}

void unipro_dump_cports(void) {
    DBG_REG(CPORT_STATUS_0);
    DBG_REG(CPORT_STATUS_1);
    DBG_REG(CPORT_STATUS_2);
    DBG_REG(CPORT_CREDIT_0);
    DBG_REG(CPORT_CREDIT_1);
}

void unipro_dump_rx(void) {
    size_t i;

    DBG_REG(AHM_RX_EOM_INT_EN_0);
    DBG_REG(AHM_RX_EOM_INT_BEF_0);
    DBG_REG(AHM_RX_EOM_INT_AFT_0);

    DBG_REG(AHM_RX_EOM_INT_EN_1);
    DBG_REG(AHM_RX_EOM_INT_BEF_1);
    DBG_REG(AHM_RX_EOM_INT_AFT_1);

    DBG_REG(AHM_RX_EOM_INT_EN_2);
    DBG_REG(AHM_RX_EOM_INT_BEF_2);
    DBG_REG(AHM_RX_EOM_INT_AFT_2);

    DBG_REG(AHM_RX_EOT_INT_EN_0);
    DBG_REG(AHM_RX_EOT_INT_BEF_0);
    DBG_REG(AHM_RX_EOT_INT_AFT_0);

    DBG_REG(AHM_RX_EOT_INT_EN_1);
    DBG_REG(AHM_RX_EOT_INT_BEF_1);
    DBG_REG(AHM_RX_EOT_INT_AFT_1);

    DBG_REG(AHM_HRESP_ERR_INT_EN_0);
    DBG_REG(AHM_HRESP_ERR_INT_BEF_0);
    DBG_REG(AHM_HRESP_ERR_INT_AFT_0);

    DBG_REG(AHM_HRESP_ERR_INT_EN_1);
    DBG_REG(AHM_HRESP_ERR_INT_BEF_1);
    DBG_REG(AHM_HRESP_ERR_INT_AFT_1);

    DBG_REG(CPB_RX_E2EFC_RSLT_ERR_INT_EN_0);
    DBG_REG(CPB_RX_E2EFC_RSLT_ERR_INT_BEF_0);
    DBG_REG(CPB_RX_E2EFC_RSLT_ERR_INT_AFT_0);

    DBG_REG(CPB_RX_E2EFC_RSLT_ERR_INT_EN_1);
    DBG_REG(CPB_RX_E2EFC_RSLT_ERR_INT_BEF_1);
    DBG_REG(CPB_RX_E2EFC_RSLT_ERR_INT_AFT_1);

    DBG_REG(CPB_RX_MSGST_ERR_INT_EN_0);
    DBG_REG(CPB_RX_MSGST_ERR_INT_BEF_0);
    DBG_REG(CPB_RX_MSGST_ERR_INT_AFT_0);

    DBG_REG(CPB_RX_MSGST_ERR_INT_EN_1);
    DBG_REG(CPB_RX_MSGST_ERR_INT_BEF_1);
    DBG_REG(CPB_RX_MSGST_ERR_INT_AFT_1);

    DBG_REG(CPB_RXDATAEMPTY_0);
    DBG_REG(CPB_RXDATAEMPTY_1);

    DBG_REG(CPB_RX_E2EFC_RSLTCODE_0);
    DBG_REG(CPB_RX_E2EFC_RSLTCODE_1);
    DBG_REG(CPB_RX_E2EFC_RSLTCODE_2);
    DBG_REG(CPB_RX_E2EFC_RSLTCODE_3);
    DBG_REG(CPB_RX_E2EFC_RSLTCODE_4);
    DBG_REG(CPB_RX_E2EFC_RSLTCODE_5);

    for (i = 0; i < CPORT_MAX; i++) {
        DBG_REG_IDX(AHM_ADDRESS_00, i);
        DBG_REG_IDX(CPB_RX_E2EFC_RETRY_TIMES_00, i);
        DBG_REG_IDX(REG_RX_PAUSE_SIZE_00, i);
        DBG_REG_IDX(CPB_RX_TRANSFERRED_DATA_SIZE_00, i);
        DBG_REG_IDX(RX_SW_RESET_00, i);
    }
}

void unipro_dump_tx(void) {
    size_t i;

    DBG_REG(AHS_TIMEOUT_INT_EN_0);
    DBG_REG(AHS_TIMEOUT_INT_BEF_0);
    DBG_REG(AHS_TIMEOUT_INT_AFT_0);

    DBG_REG(AHS_TIMEOUT_INT_EN_1);
    DBG_REG(AHS_TIMEOUT_INT_BEF_1);
    DBG_REG(AHS_TIMEOUT_INT_AFT_1);

    DBG_REG(AHS_HRESP_MODE_0);
    DBG_REG(AHS_HRESP_MODE_1);

    DBG_REG(CPB_TX_RSLTCODE_ERR_INT_EN_0);
    DBG_REG(CPB_TX_RSLTCODE_ERR_INT_BEF_0);
    DBG_REG(CPB_TX_RSLTCODE_ERR_INT_AFT_0);

    DBG_REG(CPB_TX_RSLTCODE_ERR_INT_EN_1);
    DBG_REG(CPB_TX_RSLTCODE_ERR_INT_BEF_1);
    DBG_REG(CPB_TX_RSLTCODE_ERR_INT_AFT_1);

    DBG_REG(CPB_TX_E2EFC_EN_0);
    DBG_REG(CPB_TX_E2EFC_EN_1);

    DBG_REG(TX_CLK_ENA_0);
    DBG_REG(TX_CLK_ENA_1);

    DBG_REG(CPB_TXQUEUEEMPTY_0);
    DBG_REG(CPB_TXQUEUEEMPTY_1);

    DBG_REG(CPB_TXDATAEMPTY_0);
    DBG_REG(CPB_TXDATAEMPTY_1);

    DBG_REG(CPB_TX_RESULTCODE_0);
    DBG_REG(CPB_TX_RESULTCODE_1);
    DBG_REG(CPB_TX_RESULTCODE_2);
    DBG_REG(CPB_TX_RESULTCODE_3);
    DBG_REG(CPB_TX_RESULTCODE_4);
    DBG_REG(CPB_TX_RESULTCODE_5);

    for (i = 0; i < CPORT_MAX; i++) {
        DBG_REG_IDX(AHS_TIMEOUT_00, i);
        DBG_REG_IDX(CPB_TX_BUFFER_SPACE_00, i);
        DBG_REG_IDX(REG_TX_BUFFER_SPACE_OFFSET_00, i);
        DBG_REG_IDX(TX_SW_RESET_00, i);
    }
}
