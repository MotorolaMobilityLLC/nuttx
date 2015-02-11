/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted
 * @author: Perry Hung
 */

#ifndef  _TSB_SWITCH_H_
#define  _TSB_SWITCH_H_

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
#define NCP_GETDEVICEIDMASKREQ      (0x16)
#define NCP_GETDEVICEIDMASKCNF      (0x17)
#define NCP_SWITCHATTRGETREQ        (0x22)
#define NCP_SWITCHATTRGETCNF        (0x23)
#define NCP_SWITCHIDSETREQ          (0x24)
#define NCP_SWITCHIDSETCNF          (0x25)

/* Unipro DME attributes */
#define T_REGACCCTRL_TESTONLY       (0x007F)
#define PA_ACTIVETXDATALANES        (0x1560)
#define PA_CONNECTEDTXDATALANES     (0x1561)
#define PA_TXGEAR                   (0x1568)
#define PA_TXTERMINATION            (0x1569)
#define PA_HSSERIES                 (0x156A)
#define PA_PWRMODE                  (0x1571)
#define PA_ACTIVERXDATALANES        (0x1580)
#define PA_CONNECTEDRXDATALANES     (0x1581)
#define PA_RXGEAR                   (0x1583)
#define PA_RXTERMINATION            (0x1584)
#define PA_PWRMODEUSERDATA0         (0x15B0)
#define N_DEVICEID                  (0x3000)
#define N_DEVICEID_VALID            (0x3001)
#define T_NUMCPORTS                 (0x4000)
#define T_CONNECTIONSTATE           (0x4020)
#define T_PEERDEVICEID              (0x4021)
#define T_PEERCPORTID               (0x4022)
#define T_TRAFFICCLASS              (0x4023)
    #define CPORT_TC0               (0x0)
    #define CPORT_TC1               (0x1)
#define T_CPORTFLAGS                (0x4025)
    #define CPORT_FLAGS_E2EFC       (1)
    #define CPORT_FLAGS_CSD_N       (2)
    #define CPORT_FLAGS_CSV_N       (4)
#define DME_DDBL1_REVISION          (0x5000)
#define DME_DDBL1_LEVEL             (0x5001)
#define DME_DDBL1_DEVICECLASS       (0x5002)
#define DME_DDBL1_MANUFACTUREID     (0x5003)
#define DME_DDBL1_PRODUCTID         (0x5004)
#define DME_DDBL1_LENGTH            (0x5005)
#define DME_DDBL2_VID               (0x6000)
#define DME_DDBL2_PID               (0x6001)
#define TSB_MAILBOX                 (0xA000)
#define TSB_MAXSEGMENTCONFIG        (0xD089)
#define DME_POWERMODEIND            (0xD040)

/* NCP field values */
#define NCP_RESERVED                (0x00)
#define NCP_SELINDEX_NULL           (0x00)

/* Unipro attributes values */
#define PA_CONN_TX_DATA_LANES_NR    (2)
#define PA_CONN_RX_DATA_LANES_NR    (2)
#define PA_ACTIVE_TX_DATA_LANES_NR  (2)
#define PA_ACTIVE_RX_DATA_LANES_NR  (2)
#define MAX_SEGMENT_CONFIG          (280)
#define PA_FASTMODE_RXTX            (0x11)
#define PA_FASTAUTOMODE_RXTX        (0x44)
#define PA_GEAR                     (0x1)
#define DME_POWERMODEIND_SUCCESS    (2)

#define SWITCH_PORT_MAX             (14)

struct tsb_switch_driver {
    void *priv;

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
    int (*switch_id_set)(struct tsb_switch_driver*,
                         uint8_t cportid,
                         uint8_t peer_cportid,
                         uint8_t dis,
                         uint8_t irt);
    int (*dev_id_mask_get)(struct tsb_switch_driver*,
                           uint8_t *dst);
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

#endif
