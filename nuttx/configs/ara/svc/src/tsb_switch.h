/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted
 * @author: Perry Hung
 */

#ifndef  _TSB_SWITCH_H_
#define  _TSB_SWITCH_H_

#include "tsb_unipro.h"
#include "unipro.h"

/* Switch internal attributes */
#define SWVER                       (0x0000)
#define SWSTA                       (0x0003)
#define SWINT                       (0x0010)

/* NCP commands */
#define NCP_SETREQ                  (0x00)
#define NCP_SETCNF                  (0x01)
#define NCP_PEERSETREQ              (0x02)
#define NCP_PEERSETCNF              (0x03)
#define NCP_GETREQ                  (0x04)
#define NCP_GETCNF                  (0x05)
#define NCP_PEERGETREQ              (0x06)
#define NCP_PEERGETCNF              (0x07)
#define NCP_LUTSETREQ               (0x10)
#define NCP_LUTSETCNF               (0x11)
#define NCP_LUTGETREQ               (0x12)
#define NCP_LUTGETCNF               (0x13)
#define NCP_SETDEVICEIDMASKREQ      (0x14)
#define NCP_SETDEVICEIDMASKCNF      (0x15)
#define NCP_GETDEVICEIDMASKREQ      (0x16)
#define NCP_GETDEVICEIDMASKCNF      (0x17)
#define NCP_SWITCHATTRSETREQ        (0x20)
#define NCP_SWITCHATTRSETCNF        (0x21)
#define NCP_SWITCHATTRGETREQ        (0x22)
#define NCP_SWITCHATTRGETCNF        (0x23)
#define NCP_SWITCHIDSETREQ          (0x24)
#define NCP_SWITCHIDSETCNF          (0x25)

/* NCP field values */
#define NCP_RESERVED                (0x00)
#define NCP_SELINDEX_NULL           (0x00)

/* Unipro attributes values */
#define PA_CONN_TX_DATA_LANES_NR        (2)
#define PA_CONN_RX_DATA_LANES_NR        (2)
#define PA_ACTIVE_TX_DATA_LANES_NR      (2)
#define PA_ACTIVE_RX_DATA_LANES_NR      (2)
#define MAX_SEGMENT_CONFIG              (280)
#define PA_FASTMODE_RXTX                (0x11)
#define PA_FASTAUTOMODE_RXTX            (0x44)
#define PA_GEAR                         (0x1)
#define TSB_DME_POWERMODEIND_SUCCESS    (2)

#define SWITCH_PORT_MAX             (14)

struct tsb_switch_driver {
    void *priv;

    int (*init_comm)(struct tsb_switch_driver*);
    int (*set)(struct tsb_switch_driver*,
               uint8_t portid,
               uint16_t attrid,
               uint16_t select_index,
               uint32_t attr_value);
    int (*get)(struct tsb_switch_driver*,
               uint8_t portid,
               uint16_t attrid,
               uint16_t select_index,
               uint32_t *attr_value);
    int (*peer_set)(struct tsb_switch_driver*,
                    uint8_t portid,
                    uint16_t attrid,
                    uint16_t select_index,
                    uint32_t attr_value);
    int (*peer_get)(struct tsb_switch_driver*,
                    uint8_t portid,
                    uint16_t attrid,
                    uint16_t select_index,
                    uint32_t *attr_value);

    int (*lut_set)(struct tsb_switch_driver*,
                   uint8_t addr,
                   uint8_t dst_portid);
    int (*lut_get)(struct tsb_switch_driver*,
                   uint8_t addr,
                   uint8_t *dst_portid);
    int (*switch_attr_get)(struct tsb_switch_driver*,
                           uint16_t attrid,
                           uint32_t *val);
    int (*switch_attr_set)(struct tsb_switch_driver *drv,
                           uint16_t attrid,
                           uint32_t val);
    int (*switch_id_set)(struct tsb_switch_driver*,
                         uint8_t cportid,
                         uint8_t peer_cportid,
                         uint8_t dis,
                         uint8_t irt);
    int (*dev_id_mask_get)(struct tsb_switch_driver*,
                           uint8_t *dst);
    int (*dev_id_mask_set)(struct tsb_switch_driver* drv,
                           uint8_t *mask);
};

struct tsb_switch {
    struct tsb_switch_driver *drv;
    unsigned int vreg_1p1;
    unsigned int vreg_1p8;
    unsigned int irq;
    unsigned int reset;
    uint8_t dev_ids[SWITCH_PORT_MAX];
};

int switch_if_dev_id_set(struct tsb_switch *sw,
                         uint8_t port_id,
                         uint8_t dev_id);
int switch_connection_create(struct tsb_switch *sw,
                             uint8_t port_id1,
                             uint8_t cport_id1,
                             uint8_t port_id2,
                             uint8_t cport_id2,
                             uint8_t tc,
                             uint8_t flags);

static inline int switch_connection_std_create(struct tsb_switch *sw,
                                               uint8_t port_id1,
                                               uint8_t cport_id1,
                                               uint8_t port_id2,
                                               uint8_t cport_id2) {

    return switch_connection_create(sw,
                                    port_id1,
                                    cport_id1,
                                    port_id2,
                                    cport_id2,
                                    CPORT_TC0,
                                    CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N);
};

void switch_dump_routing_table(struct tsb_switch*);

struct tsb_switch *switch_init(struct tsb_switch_driver*,
                               unsigned int vreg_1p1,
                               unsigned int vreg_1p8,
                               unsigned int reset,
                               unsigned int irq);
void switch_exit(struct tsb_switch*);

/*
 * Low-level DME access
 */

int switch_dme_set(struct tsb_switch *sw,
                   uint8_t portid,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t attr_value);

int switch_dme_get(struct tsb_switch *sw,
                   uint8_t portid,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t *attr_value);

int switch_dme_peer_set(struct tsb_switch *sw,
                        uint8_t portid,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t attr_value);

int switch_dme_peer_get(struct tsb_switch *sw,
                        uint8_t portid,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t *attr_value);

#endif
