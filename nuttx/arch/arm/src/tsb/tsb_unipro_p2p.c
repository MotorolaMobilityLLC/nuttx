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

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>
#include <nuttx/irq.h>
#include <nuttx/list.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/util.h>

#include <arch/chip/irq.h>
#include <arch/chip/unipro_p2p.h>

#include "debug.h"
#include "up_arch.h"
#include "tsb_scm.h"
#include "tsb_unipro.h"
#include "tsb_unipro_es2.h"
#include "tsb_es2_mphy_fixups.h"

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    #define TSB_READ_STATUS(boot_status) (boot_status << 24)
#elif defined(CONFIG_TSB_CHIP_REV_ES3)
    #define TSB_READ_STATUS(boot_status) (boot_status)
#endif

#define DME_DDBL2_VID               0x6000
#define DME_DDBL2_PID               0x6001
#define DME_DDBL2_SERIALNO_L        0x6002
#define DME_DDBL2_SERIALNO_H        0x6003
#define DME_DDBL2_INIT_TYPE         0x6100
#define DME_DDBL2_ENDPOINTID_H      0x6102
#define DME_DDBL2_ENDPOINTID_L      0x6103
#define DME_DDBL2_SCR               0x6104

#if defined(CONFIG_TSB_CHIP_REV_ES2)
    #define DME_DDBL2_INIT_STATUS   T_TSTSRCINCREMENT
#elif defined(CONFIG_TSB_CHIP_REV_ES3)
    #define DME_DDBL2_INIT_STATUS   TSB_DME_ES3_INIT_STATUS
#endif

uint32_t unipro_read(uint32_t offset) {
    return getreg32((volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
}

void unipro_write(uint32_t offset, uint32_t v) {
    putreg32(v, (volatile unsigned int*)(AIO_UNIPRO_BASE + offset));
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

uint32_t unipro_p2p_get_boot_status(void) {
    uint32_t boot_status = INIT_STATUS_UNINITIALIZED;
    int rc = unipro_attr_peer_read(DME_DDBL2_INIT_STATUS, &boot_status, 0 /* selector */);
    if (rc) {
        lldbg("Failed to read boot mode: %d\n", rc);
    } else {
        boot_status = TSB_READ_STATUS(boot_status);
    }

    return boot_status;
}

void unipro_p2p_setup_connection(unsigned int cport) {
    /* Set-up the L4 attributes */
    uint32_t cport_flags = CPORT_FLAGS_CSV_N;
    const uint32_t traffic_class = CPORT_TC0;
    const uint32_t token_value = 0x20;
    const uint32_t max_segment = 0x118;
    const uint32_t default_buffer_space = 0x240;

    uint32_t boot_mode = unipro_p2p_get_boot_status();
    if (boot_mode == INIT_STATUS_UNIPRO_BOOT_STARTED ||
        boot_mode == INIT_STATUS_FALLLBACK_UNIPRO_BOOT_STARTED) {
        cport_flags |= CPORT_FLAGS_CSD_N;
    } else {
        cport_flags |= CPORT_FLAGS_E2EFC;
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

void unipro_p2p_reset_connection(unsigned int cport) {
    _unipro_reset_cport(cport);
}

bool unipro_p2p_is_link_up(void) {
    return unipro_read(LUP_INT_BEF) & 0x1;
}

void unipro_p2p_setup(void) {
    /* Layer 1.5 attributes */
    unipro_attr_local_write(PA_TXTERMINATION, 1, 0 /* selector */);
    unipro_attr_peer_write(PA_TXTERMINATION, 1, 0 /* selector */);

    unipro_attr_local_write(PA_RXTERMINATION, 1, 0 /* selector */);
    unipro_attr_peer_write(PA_RXTERMINATION, 1, 0 /* selector */);

    unipro_attr_local_write(PA_SCRAMBLING, 1, 0 /* selector */);
    unipro_attr_peer_write(PA_SCRAMBLING, 1, 0 /* selector */);

    /* Layer 3 attributes */
    unipro_attr_local_write(N_DEVICEID, CONFIG_UNIPRO_LOCAL_DEVICEID, 0 /* selector */);
    unipro_attr_local_write(N_DEVICEID_VALID, 1, 0 /* selector */);

    unipro_attr_peer_write(N_DEVICEID, CONFIG_UNIPRO_REMOTE_DEVICEID, 0 /* selector */);
    unipro_attr_peer_write(N_DEVICEID_VALID, 1, 0 /* selector */);
}

int unipro_p2p_detect_linkloss(bool enable) {
    int retval;

    uint32_t mask = 0;
    if (enable) {
        mask = TSB_INTERRUPTSTATUS_LINKLOSTIND |
               TSB_INTERRUPTSTATUS_POWERMODIND |
               TSB_INTERRUPTSTATUS_ERRORPHYIND |
               TSB_INTERRUPTSTATUS_ERRORPAIND |
               TSB_INTERRUPTSTATUS_ERRORDIND |
               TSB_INTERRUPTSTATUS_ERRORNIND |
               TSB_INTERRUPTSTATUS_ERRORTIND |
               TSB_INTERRUPTSTATUS_PAINITERR |
               TSB_INTERRUPTSTATUS_MAILBOX;
    }

    retval = unipro_attr_local_write(TSB_INTERRUPTENABLE, mask, 0);
    if (retval) {
        lldbg("Failed to set tsb interrupt mask: %08x\n", mask);
    }

    return retval;
}

#if CONFIG_UNIPRO_TEST_CPORTS
void unipro_p2p_setup_test_connection(unsigned int cport, unsigned int test_port, unsigned int src_from_local, unsigned int enable_e2efc) {
    /* Set-up the L4 attributes */
    uint32_t cport_flags = CPORT_FLAGS_CSV_N|CPORT_FLAGS_E2EFC;;
    const uint32_t traffic_class = CPORT_TC0;
    const uint32_t token_value = 0x20;
    const uint32_t max_segment = 0x118;
    const uint32_t default_buffer_space = 0x240;

    if (enable_e2efc) {
        cport_flags |= CPORT_FLAGS_E2EFC;
    } else {
        cport_flags |= CPORT_FLAGS_CSD_N;
    }

    /* Disable CPORTs before making changes. */
    unipro_attr_local_write(T_CONNECTIONSTATE, 0 /* disconnected */, cport);
    unipro_attr_peer_write(T_CONNECTIONSTATE, 0 /* disconnected */, cport);

    /* Disable test ports before making changes. */
    unipro_attr_local_write(T_TSTSRCON, 0 /* disabled */, test_port);
    unipro_attr_peer_write(T_TSTSRCON, 0 /* disabled */, test_port);

    unipro_attr_local_write(T_TSTDSTON, 0 /* disabled */, test_port);
    unipro_attr_peer_write(T_TSTDSTON, 0 /* disabled */, test_port);

    /* Test Ports */
    unipro_attr_local_write(T_TSTCPORTID, cport, test_port);
    unipro_attr_peer_write(T_TSTCPORTID, cport, test_port);

    /* Test Source */
    unipro_attr_local_write(T_TSTSRCMESSAGECOUNT, 0 /* infinite */, test_port);
    unipro_attr_peer_write(T_TSTSRCMESSAGECOUNT, 0 /* infinite */, test_port);

    /* Test Destination */
    unipro_attr_local_write(T_TSTDSTMESSAGECOUNT, 0 /* clear */, test_port);
    unipro_attr_peer_write(T_TSTDSTMESSAGECOUNT, 0 /* clear */, test_port);

    unipro_attr_local_write(T_TSTDSTERRORDETECTIONENABLE, 1 /* enable */, test_port);
    unipro_attr_peer_write(T_TSTDSTERRORDETECTIONENABLE, 1 /* enable */, test_port);

    if (src_from_local) {
        unipro_attr_local_write(T_TSTSRCON, 1 /* enabled */, test_port);
        unipro_attr_peer_write(T_TSTDSTON, 1 /* enabled */, test_port);
    } else /* src_from_peer */ {
        unipro_attr_peer_write(T_TSTSRCON, 1 /* enabled */, test_port);
        unipro_attr_local_write(T_TSTDSTON, 1 /* enabled */, test_port);
    }

    /* L4 */
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

    unipro_attr_local_write(T_CPORTMODE, CPORT_MODE_UNDER_TEST, cport);
    unipro_attr_peer_write(T_CPORTMODE, CPORT_MODE_UNDER_TEST, cport);

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

    unipro_attr_peer_write(T_LOCALBUFFERSPACE, local, cport);
    unipro_attr_local_write(T_PEERBUFFERSPACE, peer, cport);

    unipro_attr_local_write(T_LOCALBUFFERSPACE, peer, cport);
    unipro_attr_peer_write(T_PEERBUFFERSPACE, local, cport);

    unipro_attr_local_write(TSB_MAXSEGMENTCONFIG, max_segment, cport);
    unipro_attr_peer_write(TSB_MAXSEGMENTCONFIG, max_segment, cport);

    /* Configure the local cport's registers. */
    unipro_enable_cport(cport);

    /* Notify the remote to configure it's cport registers. */
    unipro_mbox_enable_cport(cport);
}
#endif

static int unipro_enter_test_mode(void) {
    uint32_t v;

    lldbg("Stop auto-link\n");
    unipro_write(LUP_LINKSUP_STOP, 0x1);

    v = unipro_read(LUP_LINKSUP_STOP);
    if (v != 1) {
        lldbg("FAIL: Auto-link: 0x%08x\n", v);
        return -EFAULT;
    }
    lldbg("Auto-link is stopped\n");

    lldbg("DME_ResetReq\n");
    unipro_attr_write(TSB_DME_RESETREQ, 1, 0, 0);

    unipro_attr_read(TSB_DME_RESETCNF, &v, 0, 0);
    if (v != 1) {
        lldbg("FAIL: DME_ResetCnf: 0x%08x\n", v);
        return -EFAULT;
    }
    lldbg("DME_ResetCnf\n");

    lldbg("DME_EnableReq\n");
    unipro_attr_write(TSB_DME_LAYERENABLEREQ, 1, 0, 0);

    unipro_attr_read(TSB_DME_LAYERENABLECNF, &v, 0, 0);
    if (v != 0x1) {
        lldbg("FAIL: DME_EnableCnf: 0x%08x\n", v);
        return -EFAULT;
    }
    lldbg("DME_EnableCnf\n");

    return 0;
}

static int unipro_test_mphy_source(uint8_t param0, uint8_t param1, uint8_t param2) {
    int ret;

    ret = unipro_enter_test_mode();
    if (ret) {
        return ret;
    }

    lldbg("UNIPRO_INT_EN\n");
    unipro_write(UNIPRO_INT_EN, 1);

    lldbg("TSB_InterruptEnable\n");
    unipro_attr_write(TSB_INTERRUPTENABLE, TSB_INTERRUPTSTATUS_TESTMODEIND, 0, 0);

    return 0;
}

static int unipro_test_mphy_load(uint8_t param0, uint8_t param1, uint8_t param2) {
    int ret;
    uint32_t v;

    ret = unipro_enter_test_mode();
    if (ret) {
        return ret;
    }

    lldbg("TX_HIBERN8_CONTROL_TestMode\n");
    unipro_attr_write(0x8000|TX_HIBERN8_CONTROL, 0, 0, 0);

    lldbg("TSB_MPHYCFGUPDT\n");
    unipro_attr_write(TSB_MPHYCFGUPDT, 0, 0, 0);

    lldbg("DME_TestModeReq\n");
    unipro_attr_write(TSB_DME_TESTMODEREQ, 1, 0, 0);

    unipro_attr_read(TSB_DME_TESTMODECNF, &v, 0, 0);
    if (v != 1) {
        lldbg("TSB_DME_TESTMODECNF: 0x%08X\n", v);
        return -EFAULT;
    }
    lldbg("TSB_DME_TESTMODECNF\n");

    return 0;
}

static int unipro_test_start(uint8_t speed, uint8_t pattern, uint8_t burst) {
    uint32_t v;

    const uint32_t burstBit = (burst ? (1 << 0) : 0);
    const uint32_t cfgReadyBit = (burst ? (1 << 1) : 0);
    const uint32_t transmitBit = (burst ? (1 << 3) : 0);
    const uint32_t patternBit = (pattern ? (1 << 4) : 0);

    lldbg("TX_HIBERN8_CONTROL\n");
    unipro_attr_write(TX_HIBERN8_CONTROL, 0, 0, 0);

    v = cfgReadyBit;
    lldbg("PA_PHYTESTCONTROL local\n");
    unipro_attr_write(PA_PHYTESTCONTROL, v, 0, 0);

    v = burstBit;
    lldbg("PA_PHYTESTCONTROL local\n");
    unipro_attr_write(PA_PHYTESTCONTROL, v, 0, 0);
    lldbg("PA_PHYTESTCONTROL peer\n");
    unipro_attr_write(PA_PHYTESTCONTROL, v, 0, 1);

    uint8_t gear;
    uint8_t mode;
    uint8_t series;
    switch (speed) {
    /* PWM */
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
        mode = 1;
        gear = speed;
        series = 0;
        break;

    /* HS */
    case 8:
        mode = 2;
        gear = 1;
        series = UNIPRO_HS_SERIES_A;
        break;
    case 9:
        mode = 2;
        gear = 1;
        series = UNIPRO_HS_SERIES_B;
        break;
    case 10:
        mode = 2;
        gear = 2;
        series = UNIPRO_HS_SERIES_A;
        break;
    case 11:
        mode = 2;
        series = UNIPRO_HS_SERIES_B;
        gear = 2;
        break;
    case 12:
        mode = 2;
        gear = 3;
        series = UNIPRO_HS_SERIES_A;
        break;
    case 13:
        mode = 2;
        gear = 3;
        series = UNIPRO_HS_SERIES_B;
        break;
    default:
        lldbg("ERROR: Invalid speed\n");
        return -EFAULT;
    }

    lldbg("PA_HSSERIES peer\n");
    unipro_attr_write(PA_HSSERIES, series, 0, 1);
    lldbg("PA_HSSERIES local\n");
    unipro_attr_write(PA_HSSERIES, series, 0, 0);

    lldbg("TX_MODE local\n");
    unipro_attr_write(TX_MODE, mode, 0, 0);
    lldbg("TX_HSGEAR local\n");
    unipro_attr_write(TX_HSGEAR, gear, 0, 0);
    lldbg("TX_HSRATE_SERIES local\n");
    unipro_attr_write(TX_HSRATE_SERIES, series, 0, 0);
    lldbg("TX_PWMGEAR local\n");
    unipro_attr_write(TX_PWMGEAR, gear, 0, 0);

    lldbg("RX_MODE local\n");
    unipro_attr_write(RX_MODE, mode, 4, 0);
    lldbg("RX_HSGEAR local\n");
    unipro_attr_write(RX_HSGEAR, gear, 4, 0);
    lldbg("RX_HSRATE_SERIES local\n");
    unipro_attr_write(RX_HSRATE_SERIES, series, 0, 0);
    lldbg("RX_PWMGEAR local\n");
    unipro_attr_write(RX_PWMGEAR, gear, 4, 0);


    lldbg("TX_MODE peer\n");
    unipro_attr_write(TX_MODE, mode, 0, 1);
    lldbg("TX_HSGEAR peer\n");
    unipro_attr_write(TX_HSGEAR, gear, 0, 1);
    lldbg("TX_HSRATE_SERIES peer\n");
    unipro_attr_write(TX_HSRATE_SERIES, series, 0, 1);
    lldbg("TX_PWMGEAR peer\n");
    unipro_attr_write(TX_PWMGEAR, gear, 0, 1);

    lldbg("RX_MODE peer\n");
    unipro_attr_write(RX_MODE, mode, 4, 1);
    lldbg("RX_HSGEAR peer\n");
    unipro_attr_write(RX_HSGEAR, gear, 4, 1);
    lldbg("RX_HSRATE_SERIES peer\n");
    unipro_attr_write(RX_HSRATE_SERIES, series, 0, 1);
    lldbg("RX_PWMGEAR peer\n");
    unipro_attr_write(RX_PWMGEAR, gear, 4, 1);

    v = burstBit|cfgReadyBit;
    lldbg("PA_PHYTESTCONTROL local\n");
    unipro_attr_write(PA_PHYTESTCONTROL, v, 0, 0);
    lldbg("PA_PHYTESTCONTROL peer\n");
    unipro_attr_write(PA_PHYTESTCONTROL, v, 0, 1);

    usleep(5000000);

    v = 0;
    lldbg("PA_PHYTESTCONTROL peer\n");
    unipro_attr_write(PA_PHYTESTCONTROL, v, 0, 1);
    lldbg("PA_PHYTESTCONTROL local\n");
    unipro_attr_write(PA_PHYTESTCONTROL, v, 0, 0);

    lldbg("DL_TC0REPLAYTIMEOUTVAL\n");
    unipro_attr_write(DL_TC0REPLAYTIMEOUTVAL, 0x07ff, 0, 1);
    unipro_attr_read(DL_TC0REPLAYTIMEOUTVAL, &v, 0, 1);
    if (v != 0x7ff) {
        lldbg("DL_TC0REPLAYTIMEOUTVAL: 0x%08X\n", v);
    } else {
        lldbg("DL_TC0REPLAYTIMEOUTVAL correct\n");
    }

    lldbg("PA_PACPFRAMECOUNT local\n");
    unipro_attr_write(PA_PACPFRAMECOUNT, 0, 0, 0);
    lldbg("PA_PACPERRORCOUNT local\n");
    unipro_attr_write(PA_PACPERRORCOUNT, 0, 0, 0);
    lldbg("PA_PACPFRAMECOUNT peer\n");
    unipro_attr_write(PA_PACPFRAMECOUNT, 0, 0, 1);
    lldbg("PA_PACPERRORCOUNT peer\n");
    unipro_attr_write(PA_PACPERRORCOUNT, 0, 0, 1);

    v = burstBit|patternBit|transmitBit;
    lldbg("PA_PHYTESTCONTROL peer\n");
    unipro_attr_write(PA_PHYTESTCONTROL, v, 0, 1);
    lldbg("PA_PHYTESTCONTROL local\n");
    unipro_attr_write(PA_PHYTESTCONTROL, v, 0, 0);

    return 0;
}

int unipro_test_command(unsigned int command_mask) {
    uint8_t command = (command_mask >> 0) & 0xff;
    uint8_t param0 = (command_mask >>  8) & 0xff;
    uint8_t param1 = (command_mask >> 16) & 0xff;
    uint8_t param2 = (command_mask >> 24) & 0xff;

    lldbg("cmd=0x%02x, param0=0x%02x, param1=0x%02x, param2=0x%02x\n",
        command, param0, param1, param2);

    switch (command) {
    case 0:
        return unipro_test_mphy_load(param0, param1, param2);
    case 1:
        return unipro_test_mphy_source(param0, param1, param2);
    case 2:
        return unipro_test_start(param0, param1, param2);
    default:
        return -EINVAL;
    }
}
