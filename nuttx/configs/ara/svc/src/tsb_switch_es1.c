/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted
 * @author: Perry Hung
 */

#define DBG_COMP    DBG_SWITCH

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>

#include <arch/armv7-m/byteorder.h>

#include <errno.h>
#include <string.h>

#include "up_debug.h"
#include "tsb_switch.h"
#include "tsb_switch_driver_es1.h"

#define ES1_I2C_ADDR        (0x44)
#define ES1_I2C_FREQUENCY   (400000)
#define SWITCH_DEVICE_ID    (0)
#define SWITCH_PORT_ID      (14)

static int es1_transfer(struct tsb_switch_driver *drv,
                        uint8_t *tx_buf,
                        size_t tx_size,
                        uint8_t *rx_buf,
                        size_t rx_size) {
    struct i2c_dev_s *i2c_dev = drv->priv;
    int rc;

    dbg_insane("%s() txsize: %u rxsize: %u\n", __func__, tx_size, rx_size);

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
    if (rc < 0) {
        dbg_error("%s write fail\n", __func__);
        return rc;
    }

    return 0;
}

static int es1_dev_id_mask_get(struct tsb_switch_driver *drv,
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

    rc = es1_transfer(drv,
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

    return 0;
}

static int es1_set(struct tsb_switch_driver *drv,
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

    rc = es1_transfer(drv,
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

    return setcnf.rc;
}

static int es1_get(struct tsb_switch_driver *drv,
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

    rc = es1_transfer(drv,
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

    return getcnf.rc;
}


static int es1_peer_set(struct tsb_switch_driver *drv,
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

    rc = es1_transfer(drv,
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

    dbg_insane("%s(): ret=0x%02x, attr(0x%04x,%d)=0x%04x\n",
               __func__,
               setcnf.rc,
               attrid,
               select_index,
               attr_value);

    return setcnf.rc;
}

static int es1_peer_get(struct tsb_switch_driver *drv,
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

    rc = es1_transfer(drv,
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

    return getcnf.rc;
}


static int es1_switch_attr_get(struct tsb_switch_driver *drv,
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

    dbg_insane("%s: attrid: %x\n", __func__, attrid);

    rc = es1_transfer(drv,
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

    dbg_insane("%s(): ret=0x%02x, attr(0x%04x)=0x%08x\n",
               __func__,
               getcnf.rc,
               attrid,
               *val);

    return getcnf.rc;
}

static int es1_lut_get(struct tsb_switch_driver *drv,
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

    rc = es1_transfer(drv,
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

    return getcnf.rc;
}



static int es1_lut_set(struct tsb_switch_driver *drv,
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

    rc = es1_transfer(drv,
                      setreq,
                      sizeof(setreq),
                      (uint8_t*)&setcnf,
                      sizeof(setcnf));
    if (rc) {
        return rc;
    }

    if (setcnf.fid != NCP_LUTSETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return setcnf.rc;
    }

    return 0;
}

static int es1_switch_id_set(struct tsb_switch_driver *drv,
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

    dbg_insane("%s: cportid: %u peer_cportid: %u dis: %u irt: %u\n",
               __func__,
               cportid,
               peer_cportid,
               dis,
               irt);

    rc = es1_transfer(drv,
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

    dbg_insane("%s(): ret=0x%02x, switchDeviceId=0x%01x, cPortId=0x%01x -> peerCPortId=0x%01x\n",
               __func__,
               setcnf.rc,
               SWITCH_DEVICE_ID,
               cportid,
               peer_cportid);

    return setcnf.rc;
}

static struct tsb_switch_driver es1_drv = {
    .set                   = es1_set,
    .get                   = es1_get,
    .peer_set              = es1_peer_set,
    .peer_get              = es1_peer_get,

    .lut_set               = es1_lut_set,
    .lut_get               = es1_lut_get,

    .dev_id_mask_get       = es1_dev_id_mask_get,

    .switch_attr_get       = es1_switch_attr_get,
    .switch_id_set         = es1_switch_id_set,
};

struct tsb_switch_driver *tsb_switch_es1_init(unsigned int i2c_bus) {
    struct i2c_dev_s *i2c_dev;

    i2c_dev = up_i2cinitialize(i2c_bus);
    if (!i2c_dev) {
        return NULL;
    }

    I2C_SETFREQUENCY(i2c_dev, ES1_I2C_FREQUENCY);

    es1_drv.priv = i2c_dev;
    return &es1_drv;
}

void tsb_switch_es1_exit(void) {
    up_i2cuninitialize((struct i2c_dev_s*)es1_drv.priv);
    es1_drv.priv = NULL;
}

