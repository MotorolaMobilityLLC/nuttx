/**
 * Copyright (c) 2014-2015 Google Inc.
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
 *
 * @author Perry Hung
 * @brief MIPI UniPro stack for APBridge ES1
 */

#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>
#include <nuttx/irq.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/util.h>

#include <arch/chip/irq.h>
#include <arch/chip/unipro_p2p.h>

#include <apps/ice/svc.h>

#include "debug.h"
#include "up_arch.h"
#include "tsb_scm.h"
#include "tsb_unipro_es2.h"
#include "tsb_es2_mphy_fixups.h"

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    #define TSB_READ_STATUS(boot_status) (boot_status << 24)
#elif defined(CONFIG_TSB_CHIP_REV_ES3)
    #define TSB_READ_STATUS(boot_status) (boot_status)
#endif

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    #define DME_DDBL2_INIT_STATUS   T_TSTSRCINCREMENT
#endif

//#define CONFIG_UNIPRO_P2P_DBG_VERBOSE
#if defined(CONFIG_UNIPRO_P2P_DBG_VERBOSE)
#define DBG_ENTRY(x) { x, #x }
#else
#define DBG_ENTRY(x) { x, 0 }
#endif

struct dbg_entry {
    uint32_t val;
    const char *const str;
};

uint32_t unipro_read(uint32_t offset) {
    return getreg32((volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}

void unipro_write(uint32_t offset, uint32_t v) {
    putreg32(v, (volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}

static void unipro_dump_attribute_array(const struct dbg_entry *attributes, int cportid, int peer) {
    while (attributes && attributes->val) {

        uint32_t val;
        unipro_attr_read(attributes->val, &val, cportid, peer);
        lldbg("%04x %s: %08x\n", attributes->val, (attributes->str ? attributes->str : ""), val);
        attributes++;
    }
}

static void unipro_dump_register_array(const struct dbg_entry *registers) {
    while (registers && registers->val) {
        lldbg("%08x %s: %08x\n", registers->val, (registers->str ? registers->str : ""), unipro_read(registers->val));
        registers++;
    }
}

static void unipro_dump_register_index_array(const struct dbg_entry *registers, int idx) {
    while (registers && registers->val) {
        lldbg("%08x [%d] %s: %08x\n", (registers->val + idx * 4), idx, (registers->str ? registers->str : ""), unipro_read((registers->val + idx * 4)));
        registers++;
    }
}

void unipro_powermode_change(uint8_t txgear, uint8_t rxgear, uint8_t pwrmode, uint8_t series, uint8_t termination) {
    unipro_attr_write(PA_TXGEAR, txgear, 0 /* selector */, 0 /* peer */);
    unipro_attr_write(PA_TXTERMINATION, termination, 0 /* selector */, 0 /* peer */);
    unipro_attr_write(PA_HSSERIES, series, 0 /* selector */, 0 /* peer */);
    unipro_attr_write(PA_ACTIVETXDATALANES, 1, 0 /* selector */, 0 /* peer */);

    unipro_attr_write(PA_RXGEAR, rxgear, 0 /* selector */, 0 /* peer */);
    unipro_attr_write(PA_RXTERMINATION, termination, 0 /* selector */, 0 /* peer */);
    unipro_attr_write(PA_ACTIVERXDATALANES, 1, 0 /* selector */, 0 /* peer */);

    unipro_attr_write(PA_PWRMODE, pwrmode, 0 /* selector */, 0 /* peer */);
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

/**
 * @brief Reset the UniPro hardware block
 */
void unipro_reset(void) {
    tsb_reset(TSB_RST_UNIPROSYS);
}

static int unipro_mbox_enable_cport(uint32_t cport) {
    int rc = unipro_attr_peer_write(MBOX_ACK_ATTR, TSB_MAIL_RESET, 0 /* selector */);
    if (rc) {
        lldbg("MBOX_ACK_ATTR write failed: %d\n", rc);
        return rc;
    }

    rc = unipro_attr_peer_write(TSB_MAILBOX, (cport + 1), 0 /* selector */);
    if (rc) {
        lldbg("TSB_MAILBOX write failed: %d\n", rc);
        return rc;
    }

    uint32_t retries = 2048;
    uint32_t val;
    do {
        rc = unipro_attr_peer_read(MBOX_ACK_ATTR, &val, 0 /* selector */);
        if (rc) {
            lldbg("%s(): MBOX_ACK_ATTR poll failed: %d\n", __func__, rc);
            return rc;
        }
    } while (val != (cport + 1) && --retries > 0);

    if (!retries) {
        return -ETIMEDOUT;
    }

    return rc;
}

void unipro_p2p_setup_connection(unsigned int cport) {
    /* Set-up the L4 attributes */
    uint32_t cport_flags = CPORT_FLAGS_CSV_N;
    const uint32_t traffic_class = CPORT_TC0;
    const uint32_t token_value = 0x20;
    const uint32_t max_segment = 0x118;
    const uint32_t default_buffer_space = 0x240;

    uint32_t boot_mode = 0;
    int rc = unipro_attr_peer_read(DME_DDBL2_INIT_STATUS, &boot_mode, 0 /* selector */);
    if (rc) {
        lldbg("Failed to read boot mode\n");
    } else {
        boot_mode = TSB_READ_STATUS(boot_mode);
        if (boot_mode == INIT_STATUS_UNIPRO_BOOT_STARTED ||
            boot_mode == INIT_STATUS_FALLLBACK_UNIPRO_BOOT_STARTED) {
            cport_flags |= CPORT_FLAGS_CSD_N;
        } else {
            cport_flags |= CPORT_FLAGS_E2EFC;
        }
    }

    /* Disable CPORTs before making changes. */
    unipro_attr_local_write(T_CONNECTIONSTATE, 0 /* disconnected */, cport);
    unipro_attr_peer_write(T_CONNECTIONSTATE, 0 /* disconnected */, cport);

    unipro_attr_local_write(T_PEERDEVICEID, CONFIG_UNIPRO_REMOTE_DEVICEID, cport);
    unipro_attr_peer_write(T_PEERDEVICEID, CONFIG_UNIPRO_LOCAL_DEVICEID, cport);

    unipro_attr_local_write(T_PEERCPORTID, cport, cport);
    unipro_attr_peer_write(T_PEERCPORTID, cport, cport);

    unipro_attr_local_write(T_TRAFFICCLASS, traffic_class, cport);
    unipro_attr_peer_write(T_TRAFFICCLASS, traffic_class, cport);

    unipro_attr_local_write(T_TXTOKENVALUE, token_value, cport);
    unipro_attr_peer_write(T_TXTOKENVALUE, token_value, cport);

    unipro_attr_local_write(T_RXTOKENVALUE, token_value, cport);
    unipro_attr_peer_write(T_RXTOKENVALUE, token_value, cport);

    unipro_attr_local_write(T_CPORTFLAGS, cport_flags, cport);
    unipro_attr_peer_write(T_CPORTFLAGS, cport_flags, cport);

    unipro_attr_local_write(T_CPORTMODE, CPORT_MODE_APPLICATION, cport);
    unipro_attr_peer_write(T_CPORTMODE, CPORT_MODE_APPLICATION, cport);

    unipro_attr_local_write(T_CREDITSTOSEND, 0, cport);
    unipro_attr_peer_write(T_CREDITSTOSEND, 0, cport);

    /* The local buffer space has to match peer buffer space if E2EFC or CSD is enabled. */
    uint32_t local = 0;
    unipro_attr_local_read(T_LOCALBUFFERSPACE, &local, cport);
    if (!local) {
        local = default_buffer_space;
    }

    uint32_t peer = 0;
    unipro_attr_peer_read(T_LOCALBUFFERSPACE, &peer, cport);
    if (!peer) {
        peer = default_buffer_space;
    }

    unipro_attr_local_write(T_PEERBUFFERSPACE, peer, cport);
    //unipro_attr_peer_write(T_PEERBUFFERSPACE, local, cport, NULL);

    unipro_attr_local_write(TSB_MAXSEGMENTCONFIG, max_segment, cport);
    unipro_attr_peer_write(TSB_MAXSEGMENTCONFIG, max_segment, cport);

    /* Enable CPORTs. */
    unipro_attr_local_write(T_CONNECTIONSTATE, 1 /* connected */, cport);
    unipro_attr_peer_write(T_CONNECTIONSTATE, 1 /* connected */, cport);

    /* Configure the local cport's registers. */
    unipro_enable_cport(cport);

    /* Notify the remote to configure it's cport registers. */
    unipro_mbox_enable_cport(cport);
}

#if defined(CONFIG_ICE_APBA)
void unipro_p2p_peer_detected(void) {
    /* Enable the control cport. */
    svc_send_event(SVC_EVENT_MOD_DETECTED,
        (void *)(unsigned int)0,
        (void *)(unsigned int)0,
        (void *)(unsigned int)0);
}

void unipro_p2p_peer_lost(struct p2p_link_err_reason *reason) {
    /* Enable the control cport. */
    svc_send_event(SVC_EVENT_UNIPRO_LINK_DOWN,
        reason,
        (void *)(unsigned int)0,
        (void *)(unsigned int)0);
}

bool unipro_p2p_is_link_up(void) {
    return unipro_read(LUP_INT_BEF) & 0x1;
}

#endif

void unipro_p2p_setup(void) {
    /* Layer 1.5 attributes */
    unipro_attr_local_write(PA_TXTERMINATION, 1, 0 /* selector */);
    unipro_attr_local_write(PA_RXTERMINATION, 1, 0 /* selector */);

    /* Layer 3 attributes */
    unipro_attr_local_write(N_DEVICEID, CONFIG_UNIPRO_LOCAL_DEVICEID, 0 /* selector */);
    unipro_attr_local_write(N_DEVICEID_VALID, 1, 0 /* selector */);

    unipro_attr_peer_write(N_DEVICEID, CONFIG_UNIPRO_REMOTE_DEVICEID, 0 /* selector */);
    unipro_attr_peer_write(N_DEVICEID_VALID, 1, 0 /* selector */);
}

void unipro_p2p_detect_linkloss(void) {
    int retval;

    retval = unipro_attr_local_write(TSB_INTERRUPTENABLE,
                                     TSB_INTERRUPTSTATUS_LINKLOSTIND |
                                     TSB_INTERRUPTSTATUS_PAINITERR |
                                     TSB_INTERRUPTSTATUS_MAILBOX, 0);
    if (retval) {
        lldbg("Failed to enable tsb interrupts\n");
    }
}

const static struct dbg_entry LAYER_ATTRIBUTES[] = {
    /*
     * L1 attributes
     */
    DBG_ENTRY(TX_HSMODE_CAPABILITY),
    DBG_ENTRY(TX_HSGEAR_CAPABILITY),
    DBG_ENTRY(TX_PWMG0_CAPABILITY),
    DBG_ENTRY(TX_PWMGEAR_CAPABILITY),
    DBG_ENTRY(TX_AMPLITUDE_CAPABILITY),
    DBG_ENTRY(TX_EXTERNALSYNC_CAPABILITY),
    DBG_ENTRY(TX_HS_UNTERMINATED_LINE_DRIVE_CAPABILITY),
    DBG_ENTRY(TX_LS_TERMINATED_LINE_DRIVE_CAPABILITY),
    DBG_ENTRY(TX_MIN_SLEEP_NOCONFIG_TIME_CAPABILITY),
    DBG_ENTRY(TX_MIN_STALL_NOCONFIG_TIME_CAPABILITY),
    DBG_ENTRY(TX_MIN_SAVE_CONFIG_TIME_CAPABILITY),
    DBG_ENTRY(TX_REF_CLOCK_SHARED_CAPABILITY),
    DBG_ENTRY(TX_PHY_MAJORMINOR_RELEASE_CAPABILITY),
    DBG_ENTRY(TX_PHY_EDITORIAL_RELEASE_CAPABILITY),
    DBG_ENTRY(TX_HIBERN8TIME_CAPABILITY),
    DBG_ENTRY(TX_ADVANCED_GRANULARITY_CAPABILITY),
    DBG_ENTRY(TX_ADVANCED_HIBERN8TIME_CAPABILITY),
    DBG_ENTRY(TX_HS_EQUALIZER_SETTING_CAPABILITY),
    DBG_ENTRY(TX_MODE),
    DBG_ENTRY(TX_HSRATE_SERIES),
    DBG_ENTRY(TX_HSGEAR),
    DBG_ENTRY(TX_PWMGEAR),
    DBG_ENTRY(TX_AMPLITUDE),
    DBG_ENTRY(TX_HS_SLEWRATE),
    DBG_ENTRY(TX_SYNC_SOURCE),
    DBG_ENTRY(TX_HS_SYNC_LENGTH),
    DBG_ENTRY(TX_HS_PREPARE_LENGTH),
    DBG_ENTRY(TX_LS_PREPARE_LENGTH),
    DBG_ENTRY(TX_HIBERN8_CONTROL),
    DBG_ENTRY(TX_LCC_ENABLE),
    DBG_ENTRY(TX_PWM_BURST_CLOSURE_EXTENSION),
    DBG_ENTRY(TX_BYPASS_8B10B_ENABLE),
    DBG_ENTRY(TX_DRIVER_POLARITY),
    DBG_ENTRY(TX_HS_UNTERMINATED_LINE_DRIVE_ENABLE),
    DBG_ENTRY(TX_LS_TERMINATED_LINE_DRIVE_ENABLE),
    DBG_ENTRY(TX_LCC_SEQUENCER),
    DBG_ENTRY(TX_MIN_ACTIVATETIME),
    DBG_ENTRY(TX_ADVANCED_GRANULARITY_SETTING),
    DBG_ENTRY(TX_ADVANCED_GRANULARITY),
    DBG_ENTRY(TX_HS_EQUALIZER_SETTING),
    DBG_ENTRY(TX_FSM_STATE),
    DBG_ENTRY(MC_OUTPUT_AMPLITUDE),
    DBG_ENTRY(MC_HS_UNTERMINATED_ENABLE),
    DBG_ENTRY(MC_LS_TERMINATED_ENABLE),
    DBG_ENTRY(MC_HS_UNTERMINATED_LINE_DRIVE_ENABLE),
    DBG_ENTRY(MC_LS_TERMINATED_LINE_DRIVE_ENABLE),
    DBG_ENTRY(RX_HSMODE_CAPABILITY),
    DBG_ENTRY(RX_HSGEAR_CAPABILITY),
    DBG_ENTRY(RX_PWMG0_CAPABILITY),
    DBG_ENTRY(RX_PWMGEAR_CAPABILITY),
    DBG_ENTRY(RX_HS_UNTERMINATED_CAPABILITY),
    DBG_ENTRY(RX_LS_TERMINATED_CAPABILITY),
    DBG_ENTRY(RX_MIN_SLEEP_NOCONFIG_TIME_CAPABILITY),
    DBG_ENTRY(RX_MIN_STALL_NOCONFIG_TIME_CAPABILITY),
    DBG_ENTRY(RX_MIN_SAVE_CONFIG_TIME_CAPABILITY),
    DBG_ENTRY(RX_REF_CLOCK_SHARED_CAPABILITY),
    DBG_ENTRY(RX_HS_G1_SYNC_LENGTH_CAPABILITY),
    DBG_ENTRY(RX_HS_G1_PREPARE_LENGTH_CAPABILITY),
    DBG_ENTRY(RX_LS_PREPARE_LENGTH_CAPABILITY),
    DBG_ENTRY(RX_PWM_BURST_CLOSURE_LENGTH_CAPABILITY),
    DBG_ENTRY(RX_MIN_ACTIVATETIME_CAPABILITY),
    DBG_ENTRY(RX_PHY_MAJORMINOR_RELEASE_CAPABILITY),
    DBG_ENTRY(RX_PHY_EDITORIAL_RELEASE_CAPABILITY),
    DBG_ENTRY(RX_HIBERN8TIME_CAPABILITY),
    DBG_ENTRY(RX_HS_G2_SYNC_LENGTH_CAPABILITY),
    DBG_ENTRY(RX_HS_G3_SYNC_LENGTH_CAPABILITY),
    DBG_ENTRY(RX_HS_G2_PREPARE_LENGTH_CAPABILITY),
    DBG_ENTRY(RX_HS_G3_PREPARE_LENGTH_CAPABILITY),
    DBG_ENTRY(RX_ADVANCED_GRANULARITY_CAPABILITY),
    DBG_ENTRY(RX_ADVANCED_HIBERN8TIME_CAPABILITY),
    DBG_ENTRY(RX_ADVANCED_MIN_ACTIVATETIME_CAPABILITY),
    DBG_ENTRY(RX_MODE),
    DBG_ENTRY(RX_HSRATE_SERIES),
    DBG_ENTRY(RX_HSGEAR),
    DBG_ENTRY(RX_PWMGEAR),
    DBG_ENTRY(RX_LS_TERMINATED_ENABLE),
    DBG_ENTRY(RX_HS_UNTERMINATED_ENABLE),
    DBG_ENTRY(RX_ENTER_HIBERN8),
    DBG_ENTRY(RX_BYPASS_8B10B_ENABLE),
    DBG_ENTRY(RX_TERMINATION_FORCE_ENABLE),
    DBG_ENTRY(RX_FSM_STATE),
    DBG_ENTRY(OMC_TYPE_CAPABILITY),
    DBG_ENTRY(MC_HSMODE_CAPABILITY),
    DBG_ENTRY(MC_HSGEAR_CAPABILITY),
    DBG_ENTRY(MC_HS_START_TIME_VAR_CAPABILITY),
    DBG_ENTRY(MC_HS_START_TIME_RANGE_CAPABILITY),
    DBG_ENTRY(MC_RX_SA_CAPABILITY),
    DBG_ENTRY(MC_RX_LA_CAPABILITY),
    DBG_ENTRY(MC_LS_PREPARE_LENGTH),
    DBG_ENTRY(MC_PWMG0_CAPABILITY),
    DBG_ENTRY(MC_PWMGEAR_CAPABILITY),
    DBG_ENTRY(MC_LS_TERMINATED_CAPABILITY),
    DBG_ENTRY(MC_HS_UNTERMINATED_CAPABILITY),
    DBG_ENTRY(MC_LS_TERMINATED_LINE_DRIVE_CAPABILITY),
    DBG_ENTRY(MC_HS_UNTERMINATED_LINE_DRIVE_CAPABILIT),
    DBG_ENTRY(MC_MFG_ID_PART1),
    DBG_ENTRY(MC_MFG_ID_PART2),
    DBG_ENTRY(MC_PHY_MAJORMINOR_RELEASE_CAPABILITY),
    DBG_ENTRY(MC_PHY_EDITORIAL_RELEASE_CAPABILITY),
    DBG_ENTRY(MC_VENDOR_INFO_PART1),
    DBG_ENTRY(MC_VENDOR_INFO_PART2),
    DBG_ENTRY(MC_VENDOR_INFO_PART3),
    DBG_ENTRY(MC_VENDOR_INFO_PART4),

    /*
     * L1.5 attributes
     */
    DBG_ENTRY(PA_PHYTYPE),
    DBG_ENTRY(PA_AVAILTXDATALANES),
    DBG_ENTRY(PA_AVAILRXDATALANES),
    DBG_ENTRY(PA_MINRXTRAILINGCLOCKS),
    DBG_ENTRY(PA_TXHSG1SYNCLENGTH),
    DBG_ENTRY(PA_TXHSG1PREPARELENGTH),
    DBG_ENTRY(PA_TXHSG2SYNCLENGTH),
    DBG_ENTRY(PA_TXHSG2PREPARELENGTH),
    DBG_ENTRY(PA_TXHSG3SYNCLENGTH),
    DBG_ENTRY(PA_TXHSG3PREPARELENGTH),
    DBG_ENTRY(PA_TXMK2EXTENSION),
    DBG_ENTRY(PA_PEERSCRAMBLING),
    DBG_ENTRY(PA_TXSKIP),
    DBG_ENTRY(PA_SAVECONFIGEXTENSIONENABLE),
    DBG_ENTRY(PA_LOCALTXLCCENABLE),
    DBG_ENTRY(PA_PEERTXLCCENABLE),
    DBG_ENTRY(PA_ACTIVETXDATALANES),
    DBG_ENTRY(PA_CONNECTEDTXDATALANES),
    DBG_ENTRY(PA_TXTRAILINGCLOCKS),
    DBG_ENTRY(PA_TXPWRSTATUS),
    DBG_ENTRY(PA_TXGEAR),
    DBG_ENTRY(PA_TXTERMINATION),
    DBG_ENTRY(PA_HSSERIES),
    DBG_ENTRY(PA_PWRMODE),
    DBG_ENTRY(PA_ACTIVERXDATALANES),
    DBG_ENTRY(PA_CONNECTEDRXDATALANES),
    DBG_ENTRY(PA_RXPWRSTATUS),
    DBG_ENTRY(PA_RXGEAR),
    DBG_ENTRY(PA_RXTERMINATION),
    DBG_ENTRY(PA_SCRAMBLING),
    DBG_ENTRY(PA_MAXRXPWMGEAR),
    DBG_ENTRY(PA_MAXRXHSGEAR),
    DBG_ENTRY(PA_PACPREQTIMEOUT),
    DBG_ENTRY(PA_PACPREQEOBTIMEOUT),
    DBG_ENTRY(PA_REMOTEVERINFO),
    DBG_ENTRY(PA_LOGICALLANEMAP),
    DBG_ENTRY(PA_SLEEPNOCONFIGTIME),
    DBG_ENTRY(PA_STALLNOCONFIGTIME),
    DBG_ENTRY(PA_SAVECONFIGTIME),
    DBG_ENTRY(PA_RXHSUNTERMINATIONCAPABILITY),
    DBG_ENTRY(PA_RXLSTERMINATIONCAPABILITY),
    DBG_ENTRY(PA_HIBERN8TIME),
    DBG_ENTRY(PA_TACTIVATE),
    DBG_ENTRY(PA_LOCALVERINFO),
    DBG_ENTRY(PA_GRANULARITY),
    DBG_ENTRY(PA_MK2EXTENSIONGUARDBAND),
    DBG_ENTRY(PA_PWRMODEUSERDATA0),
    DBG_ENTRY(PA_PWRMODEUSERDATA1),
    DBG_ENTRY(PA_PWRMODEUSERDATA2),
    DBG_ENTRY(PA_PWRMODEUSERDATA3),
    DBG_ENTRY(PA_PWRMODEUSERDATA4),
    DBG_ENTRY(PA_PWRMODEUSERDATA5),
    DBG_ENTRY(PA_PWRMODEUSERDATA6),
    DBG_ENTRY(PA_PWRMODEUSERDATA7),
    DBG_ENTRY(PA_PWRMODEUSERDATA8),
    DBG_ENTRY(PA_PWRMODEUSERDATA9),
    DBG_ENTRY(PA_PWRMODEUSERDATA10),
    DBG_ENTRY(PA_PWRMODEUSERDATA11),
    DBG_ENTRY(PA_PACPFRAMECOUNT),
    DBG_ENTRY(PA_PACPERRORCOUNT),
    DBG_ENTRY(PA_PHYTESTCONTROL),

    /*
     * L2 attributes
     */
    DBG_ENTRY(DL_TXPREEMPTIONCAP),
    DBG_ENTRY(DL_TC0TXMAXSDUSIZE),
    DBG_ENTRY(DL_TC0RXINITCREDITVAL),
    DBG_ENTRY(DL_TC1TXMAXSDUSIZE),
    DBG_ENTRY(DL_TC1RXINITCREDITVAL),
    DBG_ENTRY(DL_TC0TXBUFFERSIZE),
    DBG_ENTRY(DL_TC1TXBUFFERSIZE),
    DBG_ENTRY(DL_TC0TXFCTHRESHOLD),
    DBG_ENTRY(DL_FC0PROTECTIONTIMEOUTVAL),
    DBG_ENTRY(DL_TC0REPLAYTIMEOUTVAL),
    DBG_ENTRY(DL_AFC0REQTIMEOUTVAL),
    DBG_ENTRY(DL_AFC0CREDITTHRESHOLD),
    DBG_ENTRY(DL_TC0OUTACKTHRESHOLD),
    DBG_ENTRY(DL_PEERTC0PRESENT),
    DBG_ENTRY(DL_PEERTC0RXINITCREDITVAL),
    DBG_ENTRY(DL_TC1TXFCTHRESHOLD),
    DBG_ENTRY(DL_FC1PROTECTIONTIMEOUTVAL),
    DBG_ENTRY(DL_TC1REPLAYTIMEOUTVAL),
    DBG_ENTRY(DL_AFC1REQTIMEOUTVAL),
    DBG_ENTRY(DL_AFC1CREDITTHRESHOLD),
    DBG_ENTRY(DL_TC1OUTACKTHRESHOLD),
    DBG_ENTRY(DL_PEERTC1PRESENT),
    DBG_ENTRY(DL_PEERTC1RXINITCREDITVAL),

    /*
     * L3 attributes
     */
    DBG_ENTRY(N_DEVICEID),
    DBG_ENTRY(N_DEVICEID_VALID),
    DBG_ENTRY(N_TC0TXMAXSDUSIZE),
    DBG_ENTRY(N_TC1TXMAXSDUSIZE),

    /*
     * L4 attributes
     */
    DBG_ENTRY(T_NUMCPORTS),
    DBG_ENTRY(T_NUMTESTFEATURES),
    DBG_ENTRY(T_TC0TXMAXSDUSIZE),
    DBG_ENTRY(T_TC1TXMAXSDUSIZE),
    DBG_ENTRY(T_TSTCPORTID),
    DBG_ENTRY(T_TSTSRCON),
    DBG_ENTRY(T_TSTSRCPATTERN),
    DBG_ENTRY(T_TSTSRCINCREMENT),
    DBG_ENTRY(T_TSTSRCMESSAGESIZE),
    DBG_ENTRY(T_TSTSRCMESSAGECOUNT),
    DBG_ENTRY(T_TSTSRCINTERMESSAGEGAP),
    DBG_ENTRY(T_TSTDSTON),
    DBG_ENTRY(T_TSTDSTERRORDETECTIONENABLE),
    DBG_ENTRY(T_TSTDSTPATTERN),
    DBG_ENTRY(T_TSTDSTINCREMENT),
    DBG_ENTRY(T_TSTDSTMESSAGECOUNT),
    DBG_ENTRY(T_TSTDSTMESSAGEOFFSET),
    DBG_ENTRY(T_TSTDSTMESSAGESIZE),
    DBG_ENTRY(T_TSTDSTFCCREDITS),
    DBG_ENTRY(T_TSTDSTINTERFCTOKENGAP),
    DBG_ENTRY(T_TSTDSTINITIALFCCREDITS),
    DBG_ENTRY(T_TSTDSTERRORCODE),

    /*
     * DME attributes
     */
    DBG_ENTRY(DME_DDBL1_REVISION),
    DBG_ENTRY(DME_DDBL1_LEVEL),
    DBG_ENTRY(DME_DDBL1_DEVICECLASS),
    DBG_ENTRY(DME_DDBL1_MANUFACTURERID),
    DBG_ENTRY(DME_DDBL1_PRODUCTID),
    DBG_ENTRY(DME_DDBL1_LENGTH),
    DBG_ENTRY(DME_FC0PROTECTIONTIMEOUTVAL),
    DBG_ENTRY(DME_TC0REPLAYTIMEOUTVAL),
    DBG_ENTRY(DME_AFC0REQTIMEOUTVAL),
    DBG_ENTRY(DME_FC1PROTECTIONTIMEOUTVAL),
    DBG_ENTRY(DME_TC1REPLAYTIMEOUTVAL),
    DBG_ENTRY(DME_AFC1REQTIMEOUTVAL),

    /*
     * TSB attributes
     */
    DBG_ENTRY(TSB_T_REGACCCTRL_TESTONLY),
    DBG_ENTRY(TSB_DME_DDBL2_A),
    DBG_ENTRY(TSB_DME_DDBL2_B),
    DBG_ENTRY(TSB_MAILBOX),
    DBG_ENTRY(TSB_DME_LAYERENABLEREQ),
    DBG_ENTRY(TSB_DME_LAYERENABLECNF),
    DBG_ENTRY(TSB_DME_RESETREQ),
    DBG_ENTRY(TSB_DME_RESETCNF),
    DBG_ENTRY(TSB_DME_ENDPOINTRESETREQ),
    DBG_ENTRY(TSB_DME_ENDPOINTRESETCNF),
    DBG_ENTRY(TSB_DME_ENDPOINTRESETIND),
    DBG_ENTRY(TSB_DME_LINKSTARTUPREQ),
    DBG_ENTRY(TSB_DME_LINKSTARTUPCNF),
    DBG_ENTRY(TSB_DME_LINKSTARTUPIND),
    DBG_ENTRY(TSB_DME_LINKLOSTIND),
    DBG_ENTRY(TSB_DME_HIBERNATEENTERREQ),
    DBG_ENTRY(TSB_DME_HIBERNATEENTERCNF),
    DBG_ENTRY(TSB_DME_HIBERNATEENTERIND),
    DBG_ENTRY(TSB_DME_HIBERNATEEXITREQ),
    DBG_ENTRY(TSB_DME_HIBERNATEEXITCNF),
    DBG_ENTRY(TSB_DME_HIBERNATEEXITIND),
    DBG_ENTRY(TSB_DME_POWERMODEIND),
    DBG_ENTRY(TSB_DME_TESTMODEREQ),
    DBG_ENTRY(TSB_DME_TESTMODECNF),
    DBG_ENTRY(TSB_DME_TESTMODEIND),
    DBG_ENTRY(TSB_DME_ERRORPHYIND),
    DBG_ENTRY(TSB_DME_ERRORPAIND),
    DBG_ENTRY(TSB_DME_ERRORDIND),
    DBG_ENTRY(TSB_DME_ERRORNIND),
    DBG_ENTRY(TSB_DME_ERRORTIND),
    DBG_ENTRY(TSB_INTERRUPTENABLE),
    DBG_ENTRY(TSB_INTERRUPTSTATUS),
    DBG_ENTRY(TSB_L2STATUS),
    DBG_ENTRY(TSB_POWERSTATE),
    DBG_ENTRY(TSB_TXBURSTCLOSUREDELAY),
    DBG_ENTRY(TSB_MPHYCFGUPDT),
    DBG_ENTRY(TSB_ADJUSTTRAILINGCLOCKS),
    DBG_ENTRY(TSB_SUPPRESSRREQ),
    DBG_ENTRY(TSB_L2TIMEOUT),
    DBG_ENTRY(TSB_MAXSEGMENTCONFIG),
    DBG_ENTRY(TSB_TBD),
    DBG_ENTRY(TSB_RBD),
    DBG_ENTRY(TSB_DEBUGTXBYTECOUNT),
    DBG_ENTRY(TSB_DEBUGRXBYTECOUNT),
    DBG_ENTRY(TSB_DEBUGINVALIDBYTEENABLE),
    DBG_ENTRY(TSB_DEBUGLINKSTARTUP),
    DBG_ENTRY(TSB_DEBUGPWRCHANGE),
    DBG_ENTRY(TSB_DEBUGSTATES),
    DBG_ENTRY(TSB_DEBUGCOUNTER0),
    DBG_ENTRY(TSB_DEBUGCOUNTER1),
    DBG_ENTRY(TSB_DEBUGCOUNTER0MASK),
    DBG_ENTRY(TSB_DEBUGCOUNTER1MASK),
    DBG_ENTRY(TSB_DEBUGCOUNTERCONTROL),
    DBG_ENTRY(TSB_DEBUGCOUNTEROVERFLOW),
    DBG_ENTRY(TSB_DEBUGOMC),
    DBG_ENTRY(TSB_DEBUGCOUNTERBMASK),
    DBG_ENTRY(TSB_DEBUGSAVECONFIGTIME),
    DBG_ENTRY(TSB_DEBUGCLOCKENABLE),
    DBG_ENTRY(TSB_DEEPSTALLCFG),
    DBG_ENTRY(TSB_DEEPSTALLSTATUS),

    { 0, 0 }
};

const static struct dbg_entry CPORT_ATTRIBUTES[] = {
    DBG_ENTRY(T_CONNECTIONSTATE),
    DBG_ENTRY(T_PEERDEVICEID),
    DBG_ENTRY(T_PEERCPORTID),
    DBG_ENTRY(T_TRAFFICCLASS),
    DBG_ENTRY(T_PROTOCOLID),
    DBG_ENTRY(T_CPORTFLAGS),
    DBG_ENTRY(T_TXTOKENVALUE),
    DBG_ENTRY(T_RXTOKENVALUE),
    DBG_ENTRY(T_LOCALBUFFERSPACE),
    DBG_ENTRY(T_PEERBUFFERSPACE),
    DBG_ENTRY(T_CREDITSTOSEND),
    DBG_ENTRY(T_CPORTMODE),
    { 0, 0 }
};

const struct dbg_entry INT_REGISTERS[] = {
    DBG_ENTRY(UNIPRO_INT_EN),
    DBG_ENTRY(UNIPRO_INT_BEF),
    DBG_ENTRY(UNIPRO_INT_AFT),

    DBG_ENTRY(LUP_INT_EN),
    DBG_ENTRY(LUP_INT_BEF),
    DBG_ENTRY(LUP_INT_AFT),

    DBG_ENTRY(A2D_ATTRACS_INT_EN),
    DBG_ENTRY(A2D_ATTRACS_INT_BEF),
    DBG_ENTRY(A2D_ATTRACS_INT_AFT),

    { 0, 0 }
};

const struct dbg_entry CPORT_REGISTERS[] = {
    DBG_ENTRY(CPORT_STATUS_0),
    DBG_ENTRY(CPORT_STATUS_1),
    DBG_ENTRY(CPORT_STATUS_2),
    DBG_ENTRY(CPORT_CREDIT_0),
    DBG_ENTRY(CPORT_CREDIT_1),
    { 0, 0 }
};

const struct dbg_entry RX_REGISTERS[] = {
    DBG_ENTRY(AHM_RX_EOM_INT_EN_0),
    DBG_ENTRY(AHM_RX_EOM_INT_BEF_0),
    DBG_ENTRY(AHM_RX_EOM_INT_AFT_0),

    DBG_ENTRY(AHM_RX_EOM_INT_EN_1),
    DBG_ENTRY(AHM_RX_EOM_INT_BEF_1),
    DBG_ENTRY(AHM_RX_EOM_INT_AFT_1),

    DBG_ENTRY(AHM_RX_EOM_INT_EN_2),
    DBG_ENTRY(AHM_RX_EOM_INT_BEF_2),
    DBG_ENTRY(AHM_RX_EOM_INT_AFT_2),

    DBG_ENTRY(AHM_RX_EOT_INT_EN_0),
    DBG_ENTRY(AHM_RX_EOT_INT_BEF_0),
    DBG_ENTRY(AHM_RX_EOT_INT_AFT_0),

    DBG_ENTRY(AHM_RX_EOT_INT_EN_1),
    DBG_ENTRY(AHM_RX_EOT_INT_BEF_1),
    DBG_ENTRY(AHM_RX_EOT_INT_AFT_1),

    DBG_ENTRY(AHM_HRESP_ERR_INT_EN_0),
    DBG_ENTRY(AHM_HRESP_ERR_INT_BEF_0),
    DBG_ENTRY(AHM_HRESP_ERR_INT_AFT_0),

    DBG_ENTRY(AHM_HRESP_ERR_INT_EN_1),
    DBG_ENTRY(AHM_HRESP_ERR_INT_BEF_1),
    DBG_ENTRY(AHM_HRESP_ERR_INT_AFT_1),

    DBG_ENTRY(CPB_RX_E2EFC_RSLT_ERR_INT_EN_0),
    DBG_ENTRY(CPB_RX_E2EFC_RSLT_ERR_INT_BEF_0),
    DBG_ENTRY(CPB_RX_E2EFC_RSLT_ERR_INT_AFT_0),

    DBG_ENTRY(CPB_RX_E2EFC_RSLT_ERR_INT_EN_1),
    DBG_ENTRY(CPB_RX_E2EFC_RSLT_ERR_INT_BEF_1),
    DBG_ENTRY(CPB_RX_E2EFC_RSLT_ERR_INT_AFT_1),

    DBG_ENTRY(CPB_RX_MSGST_ERR_INT_EN_0),
    DBG_ENTRY(CPB_RX_MSGST_ERR_INT_BEF_0),
    DBG_ENTRY(CPB_RX_MSGST_ERR_INT_AFT_0),

    DBG_ENTRY(CPB_RX_MSGST_ERR_INT_EN_1),
    DBG_ENTRY(CPB_RX_MSGST_ERR_INT_BEF_1),
    DBG_ENTRY(CPB_RX_MSGST_ERR_INT_AFT_1),

    DBG_ENTRY(CPB_RXDATAEMPTY_0),
    DBG_ENTRY(CPB_RXDATAEMPTY_1),

    DBG_ENTRY(CPB_RX_E2EFC_RSLTCODE_0),
    DBG_ENTRY(CPB_RX_E2EFC_RSLTCODE_1),
    DBG_ENTRY(CPB_RX_E2EFC_RSLTCODE_2),
    DBG_ENTRY(CPB_RX_E2EFC_RSLTCODE_3),
    DBG_ENTRY(CPB_RX_E2EFC_RSLTCODE_4),
    DBG_ENTRY(CPB_RX_E2EFC_RSLTCODE_5),

    { 0, 0 }
};

const struct dbg_entry TX_REGISTERS[] = {

    DBG_ENTRY(AHS_TIMEOUT_INT_EN_0),
    DBG_ENTRY(AHS_TIMEOUT_INT_BEF_0),
    DBG_ENTRY(AHS_TIMEOUT_INT_AFT_0),

    DBG_ENTRY(AHS_TIMEOUT_INT_EN_1),
    DBG_ENTRY(AHS_TIMEOUT_INT_BEF_1),
    DBG_ENTRY(AHS_TIMEOUT_INT_AFT_1),

    DBG_ENTRY(AHS_HRESP_MODE_0),
    DBG_ENTRY(AHS_HRESP_MODE_1),

    DBG_ENTRY(CPB_TX_RSLTCODE_ERR_INT_EN_0),
    DBG_ENTRY(CPB_TX_RSLTCODE_ERR_INT_BEF_0),
    DBG_ENTRY(CPB_TX_RSLTCODE_ERR_INT_AFT_0),

    DBG_ENTRY(CPB_TX_RSLTCODE_ERR_INT_EN_1),
    DBG_ENTRY(CPB_TX_RSLTCODE_ERR_INT_BEF_1),
    DBG_ENTRY(CPB_TX_RSLTCODE_ERR_INT_AFT_1),

    DBG_ENTRY(CPB_TX_E2EFC_EN_0),
    DBG_ENTRY(CPB_TX_E2EFC_EN_1),

    DBG_ENTRY(TX_CLK_ENA_0),
    DBG_ENTRY(TX_CLK_ENA_1),

    DBG_ENTRY(CPB_TXQUEUEEMPTY_0),
    DBG_ENTRY(CPB_TXQUEUEEMPTY_1),

    DBG_ENTRY(CPB_TXDATAEMPTY_0),
    DBG_ENTRY(CPB_TXDATAEMPTY_1),

    DBG_ENTRY(CPB_TX_RESULTCODE_0),
    DBG_ENTRY(CPB_TX_RESULTCODE_1),
    DBG_ENTRY(CPB_TX_RESULTCODE_2),
    DBG_ENTRY(CPB_TX_RESULTCODE_3),
    DBG_ENTRY(CPB_TX_RESULTCODE_4),
    DBG_ENTRY(CPB_TX_RESULTCODE_5),

    { 0, 0 }
};

const struct dbg_entry CPORT_RX_REGISTERS[] = {
    DBG_ENTRY(AHM_ADDRESS_00),
    DBG_ENTRY(CPB_RX_E2EFC_RETRY_TIMES_00),
    DBG_ENTRY(REG_RX_PAUSE_SIZE_00),
    DBG_ENTRY(CPB_RX_TRANSFERRED_DATA_SIZE_00),
    DBG_ENTRY(RX_SW_RESET_00),
    { 0, 0 }
};

const struct dbg_entry CPORT_TX_REGISTERS[] = {
    DBG_ENTRY(AHS_TIMEOUT_00),
    DBG_ENTRY(CPB_TX_BUFFER_SPACE_00),
    DBG_ENTRY(REG_TX_BUFFER_SPACE_OFFSET_00),
    DBG_ENTRY(TX_SW_RESET_00),
    { 0, 0 }
};

/*
 * debug interfaces
 */
void unipro_dump_attributes(int peer) {
    unipro_dump_attribute_array(LAYER_ATTRIBUTES, 0, peer);
}

void unipro_dump_cport_attributes(size_t start, size_t end, int peer) {
    size_t i;

    for (i = start; i <= end; i++) {
        lldbg("CP%d:\n", i);
        unipro_dump_attribute_array(CPORT_ATTRIBUTES, i, peer);
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

        rc = unipro_attr_read(TSB_INTERRUPTSTATUS, &attr_val, 0, 0);
        if (!attr_val) {
            break;
        }

        lldbg("TSB_INTERRUPTSTATUS, val=0x%x, rc=%d\n", attr_val, rc);
        for (j = 0; j < sizeof(unipro_irq_attr)/sizeof(unipro_irq_attr[0]); j++) {
            if ((attr_val & (1 << j)) && unipro_irq_attr[j]) {
                rc = unipro_attr_read(unipro_irq_attr[j], &attr_val, 0, 0);
                lldbg("j=%d, attr=0x%x, val=0x%x, rc=%d\n", j, unipro_irq_attr[j], attr_val, rc);
            }
        }
    }
}

/* Attributes as sources of interrupts from the Unipro ports */
void unipro_dump_ints(void) {
    unipro_dump_register_array(INT_REGISTERS);
}

void unipro_dump_cports(void) {
    unipro_dump_register_array(CPORT_REGISTERS);
}

void unipro_dump_rx(void) {
    size_t i;

    unipro_dump_register_array(RX_REGISTERS);

    for (i = 0; i < unipro_cport_count(); i++) {
        unipro_dump_register_index_array(CPORT_RX_REGISTERS, i);
    }
}

void unipro_dump_tx(void) {
    size_t i;

    unipro_dump_register_array(TX_REGISTERS);

    for (i = 0; i < unipro_cport_count(); i++) {
        unipro_dump_register_index_array(CPORT_TX_REGISTERS, i);
    }
}