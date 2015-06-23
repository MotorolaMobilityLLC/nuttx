/*
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
 */

/**
 * @author: Perry Hung
 */

#define DBG_COMP    DBG_SWITCH

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>

#include <arch/byteorder.h>

#include <errno.h>
#include <string.h>

#include "up_debug.h"
#include "tsb_switch.h"
#include "tsb_switch_driver_es1.h"

#define ES1_I2C_ADDR        (0x44)
#define ES1_I2C_FREQUENCY   (400000)

#define CHECK_VALID_ENTRY(entry) \
    (valid_bitmask[15 - ((entry) / 8)] & (1 << ((entry)) % 8))

static int es1_transfer(struct tsb_switch *sw,
                        uint8_t *tx_buf,
                        size_t tx_size,
                        uint8_t *rx_buf,
                        size_t rx_size) {
    struct i2c_dev_s *i2c_dev = sw->priv;
    int rc;

    dbg_insane("\t%s(): TX buffer:\n", __func__);
    dbg_print_buf(DBG_INSANE, tx_buf, tx_size);

    struct i2c_msg_s msgs[] = {
        {
            .addr = ES1_I2C_ADDR,
            .flags = 0,
            .buffer = tx_buf,
            .length = tx_size,
        },
        {
            .addr = ES1_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = rx_buf,
            .length = rx_size,
        }
    };
    rc = I2C_TRANSFER(i2c_dev, msgs, 2);

    dbg_insane("\t%s(): RX buffer:\n", __func__);
    dbg_print_buf(DBG_INSANE, rx_buf, rx_size);

    if (rc < 0) {
        dbg_error("%s(): error %d\n", __func__, rc);
        return rc;
    }

    return 0;
}

static int es1_dev_id_mask_get(struct tsb_switch *sw,
                               uint8_t unipro_portid,
                               uint8_t *dst) {
    int rc;
    uint8_t getreq[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_GETDEVICEIDMASKREQ
    };

    struct __attribute__ ((__packed__)) getcnf {
        uint8_t rc;
        uint8_t fid;
        uint8_t mask[16];
    } getcnf;

    rc = es1_transfer(sw,
                      getreq,
                      sizeof(getreq),
                      (uint8_t*)&getcnf,
                      sizeof(getcnf));
    if (rc) {
        return rc;
    }

    if (getcnf.fid != NCP_GETDEVICEIDMASKCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return getcnf.rc;
    }

    memcpy(dst, &getcnf.mask, sizeof(getcnf.mask));

    dbg_verbose("%s(): device_id_mask[]:\n", __func__);
    dbg_print_buf(DBG_VERBOSE, dst, sizeof(getcnf.mask));

    return 0;
}

static int es1_set(struct tsb_switch *sw,
                   uint8_t portid,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t attr_value) {
    uint8_t setreq[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_SETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
        ((attr_value >> 24) & 0xff),
        ((attr_value >> 16) & 0xff),
        ((attr_value >> 8) & 0xff),
        (attr_value & 0xff)
    };

    struct __attribute__ ((__packed__)) setcnf {
        uint8_t portid;
        uint8_t fid;
        uint8_t reserved;
        uint8_t rc;
    } setcnf;

    int rc;

    rc = es1_transfer(sw,
                      setreq,
                      sizeof(setreq),
                      (uint8_t*)&setcnf,
                      sizeof(setcnf));
    if (rc) {
        return rc;
    }

    if (setcnf.fid != NCP_SETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return setcnf.rc;
    }

    dbg_verbose("%s(): portid=%u, (attr,sel)=(0x%04x,%d)=0x%04x, ret=0x%02x\n",
                __func__,
                portid,
                attrid,
                select_index,
                attr_value,
                setcnf.rc);

    return setcnf.rc;
}

static int es1_get(struct tsb_switch *sw,
                   uint8_t portid,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t *attr_value) {
    int rc;
    uint8_t getreq[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_GETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };

    struct __attribute__ ((__packed__)) getcnf {
        uint8_t portid;
        uint8_t fid;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } getcnf;

    rc = es1_transfer(sw,
                      getreq,
                      sizeof(getreq),
                      (uint8_t*)&getcnf,
                      sizeof(getcnf));
    if (rc) {
        return rc;
    }

    if (getcnf.fid != NCP_GETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return getcnf.rc;
    }

    *attr_value = be32_to_cpu(getcnf.attr_val);

    dbg_verbose("%s(): portid=%u, (attr,sel)=(0x%04x,%d)=0x%04x, ret=0x%02x\n",
                __func__,
                portid,
                attrid,
                select_index,
                *attr_value,
                getcnf.rc);

    return getcnf.rc;
}


static int es1_peer_set(struct tsb_switch *sw,
                        uint8_t portid,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t attr_value) {

    uint8_t setreq[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_PEERSETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
        ((attr_value >> 24) & 0xff),
        ((attr_value >> 16) & 0xff),
        ((attr_value >> 8) & 0xff),
        (attr_value & 0xff)
    };

    struct __attribute__ ((__packed__)) peer_setcnf {
        uint8_t portid;
        uint8_t fid;
        uint8_t reserved;
        uint8_t rc;
    } setcnf;

    int rc;

    rc = es1_transfer(sw,
                      setreq,
                      sizeof(setreq),
                      (uint8_t*)&setcnf,
                      sizeof(setcnf));
    if (rc) {
        return rc;
    }

    if (setcnf.fid != NCP_PEERSETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return setcnf.rc;
    }

    dbg_verbose("%s(): portid=%u, (attr,sel)=(0x%04x,%d)=0x%04x, ret=0x%02x\n",
                __func__,
                portid,
                attrid,
                select_index,
                attr_value,
                setcnf.rc);

    return setcnf.rc;
}

static int es1_peer_get(struct tsb_switch *sw,
                        uint8_t portid,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t *attr_value) {
    int rc;
    uint8_t getreq[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_PEERGETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };

    struct __attribute__ ((__packed__)) getcnf {
        uint8_t portid;
        uint8_t fid;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } getcnf;

    rc = es1_transfer(sw,
                      getreq,
                      sizeof(getreq),
                      (uint8_t*)&getcnf,
                      sizeof(getcnf));
    if (rc) {
        return rc;
    }

    if (getcnf.fid != NCP_PEERGETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return getcnf.rc;
    }

    *attr_value = be32_to_cpu(getcnf.attr_val);

    dbg_verbose("%s(): portid=%u, (attr,sel)=(0x%04x,%d)=0x%04x, ret=0x%02x\n",
                __func__,
                portid,
                attrid,
                select_index,
                *attr_value,
                getcnf.rc);

    return getcnf.rc;
}


static int es1_switch_attr_get(struct tsb_switch *sw,
                                   uint16_t attrid,
                                   uint32_t *val) {
    uint8_t getreq[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_SWITCHATTRGETREQ,
        (attrid >> 8),
        (attrid & 0xFF),
    };

    struct __attribute__ ((__packed__)) switch_attr_cnf {
        uint8_t rc;
        uint8_t functionid;
        uint32_t attr_val;
    } getcnf;

    int rc;

    rc = es1_transfer(sw,
                      getreq,
                      sizeof(getreq),
                      (uint8_t*)&getcnf,
                      sizeof(getcnf));
    if (rc) {
        return rc;
    }

    if (getcnf.functionid != NCP_SWITCHATTRGETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return getcnf.rc;
    }

    if (val) {
        *val = be32_to_cpu(getcnf.attr_val);
    }

    dbg_verbose("%s(): attr(0x%04x)=0x%08x, ret=0x%02x\n",
                __func__,
                attrid,
                *val,
                getcnf.rc);

    return getcnf.rc;
}

static int es1_lut_get(struct tsb_switch *sw,
                       uint8_t unipro_portid,
                       uint8_t addr,
                       uint8_t *dst_portid) {
    int rc;
    uint8_t getreq[] = {
        SWITCH_DEVICE_ID,
        addr,
        NCP_LUTGETREQ,
    };

    struct __attribute__ ((__packed__)) getcnf {
        uint8_t rc;
        uint8_t fid;
        uint8_t reserved;
        uint8_t dst_portid;
    } getcnf;

    rc = es1_transfer(sw,
                      getreq,
                      sizeof(getreq),
                      (uint8_t*)&getcnf,
                      sizeof(getcnf));
    if (rc) {
        return rc;
    }

    if (getcnf.fid != NCP_LUTGETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return getcnf.rc;
    }

    *dst_portid = getcnf.dst_portid;

    dbg_verbose("%s(): addr=%u, dst_portid=%u\n", __func__, addr, *dst_portid);
    return getcnf.rc;
}



static int es1_lut_set(struct tsb_switch *sw,
                       uint8_t unipro_portid,
                       uint8_t addr,
                       uint8_t dst_portid) {
    int rc;
    uint8_t setreq[] = {
        SWITCH_DEVICE_ID,
        addr,
        NCP_LUTSETREQ,
        NCP_RESERVED,
        dst_portid
    };

    struct __attribute__ ((__packed__)) switch_id_setcnf {
        uint8_t rc;
        uint8_t fid;
    } setcnf;

    rc = es1_transfer(sw,
                      setreq,
                      sizeof(setreq),
                      (uint8_t*)&setcnf,
                      sizeof(setcnf));
    if (rc) {
        return rc;
    }

    if (setcnf.fid != NCP_LUTSETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
    }

    dbg_verbose("%s(): addr=%u, dst_portid=%u, ret=0x%02x\n",
                __func__, addr, dst_portid, setcnf.rc);
    return setcnf.rc;
}

static int es1_switch_id_set(struct tsb_switch *sw,
                             uint8_t cportid,
                             uint8_t peer_cportid,
                             uint8_t dis,
                             uint8_t irt) {
    uint8_t setreq[] = {
        SWITCH_DEVICE_ID,
        (dis << 2) | (irt << 0),
        NCP_SWITCHIDSETREQ,
        SWITCH_DEVICE_ID,
        cportid,        // L4 CPortID
        SWITCH_DEVICE_ID,
        peer_cportid,
        NCP_RESERVED,
        SWITCH_PORT_ID      // Source portID
    };

    struct __attribute__ ((__packed__)) switch_id_setcnf {
        uint8_t rc;
        uint8_t fid;
    } setcnf;

    int rc;

    dbg_insane("%s(): cportid: %u peer_cportid: %u dis: %u irt: %u\n",
               __func__,
               cportid,
               peer_cportid,
               dis,
               irt);

    rc = es1_transfer(sw,
                      setreq,
                      sizeof(setreq),
                      (uint8_t*)&setcnf,
                      sizeof(setcnf));
    if (rc) {
        return rc;
    }

    if (setcnf.fid != NCP_SWITCHIDSETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return setcnf.rc;
    }

    dbg_verbose("%s(): switchDeviceId=0x%01x, cPortId=0x%01x -> peerCPortId=0x%01x, ret=0x%02x\n",
                __func__,
                SWITCH_DEVICE_ID,
                cportid,
                peer_cportid,
                setcnf.rc);
    return setcnf.rc;
}

/**
 * @brief Dump routing table to low level console
 */
static int es1_dump_routing_table(struct tsb_switch *sw) {
    int i, j, idx, rc;
    uint8_t p = 0, valid_bitmask[16];
    char msg[64];

    rc = switch_dev_id_mask_get(sw, SWITCH_PORT_ID, valid_bitmask);
    if (rc) {
        dbg_error("%s() Failed to retrieve routing table.\n", __func__);
        return rc;
    }

    dbg_info("%s(): Routing table\n", __func__);
    dbg_info("======================================================\n");

    /* Replace invalid entries with 'XX' */
    for (i = 0; i < 8; i++) {
        /* Build a line with the offset, 8 entries, a '|' then 8 entries */
        idx = 0;
        rc = sprintf(msg, "%3d: ", i * 16);
        if (rc <= 0)
            goto out;
        else
            idx += rc;

        for (j = 0; j < 16; j++) {
            if (CHECK_VALID_ENTRY(i * 16 + j)) {
                switch_lut_get(sw, SWITCH_PORT_ID, i * 16 + j, &p);
                rc = sprintf(msg + idx, "%2u ", p);
                if (rc <= 0)
                    goto out;
                else
                    idx += rc;
            } else {
                rc = sprintf(msg + idx, "XX ");
                if (rc <= 0)
                    goto out;
                else
                    idx += rc;
            }

            if (j == 7) {
                rc = sprintf(msg + idx, "| ");
                if (rc <= 0)
                    goto out;
                else
                    idx += rc;
            }
        }

        rc = sprintf(msg + idx, "\n");
        if (rc <= 0)
            goto out;
        else
            idx += rc;
        msg[idx] = 0;

        /* Output the line */
        dbg_info("%s", msg);
    }

out:
    dbg_info("======================================================\n");

    return 0;
}

static struct tsb_switch_ops es1_ops = {
    .set                   = es1_set,
    .get                   = es1_get,
    .peer_set              = es1_peer_set,
    .peer_get              = es1_peer_get,

    .lut_set               = es1_lut_set,
    .lut_get               = es1_lut_get,
    .dump_routing_table    = es1_dump_routing_table,

    .dev_id_mask_get       = es1_dev_id_mask_get,

    .switch_attr_get       = es1_switch_attr_get,
    .switch_id_set         = es1_switch_id_set,
};

int tsb_switch_es1_init(struct tsb_switch *sw, unsigned int i2c_bus) {
    struct i2c_dev_s *i2c_dev;

    dbg_info("Initializing ES1 switch...\n");

    i2c_dev = up_i2cinitialize(i2c_bus);
    if (!i2c_dev) {
        return -ENODEV;
    }

    I2C_SETFREQUENCY(i2c_dev, ES1_I2C_FREQUENCY);

    sw->priv = i2c_dev;
    sw->ops = &es1_ops;

    dbg_info("... Done!\n");

    return 0;
}

void tsb_switch_es1_exit(struct tsb_switch *sw) {
    up_i2cuninitialize((struct i2c_dev_s*) sw->priv);
    sw->priv = NULL;
    sw->ops = NULL;
}

