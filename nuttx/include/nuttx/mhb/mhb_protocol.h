/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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

#ifndef MHB_PROTOCOL_H
#define MHB_PROTOCOL_H

#define MHB_MAX_MSG_SIZE (2048)

struct mhb_hdr
{
	uint16_t length;
	uint8_t rsvd;
	uint8_t addr;
	uint8_t type;
	uint8_t result;
} __attribute__((packed));

#define MHB_HDR_SIZE (sizeof(struct mhb_hdr))

/* rsvd */
/* Must be 0. */

/* addr */
#define MHB_PEER_MASK      (0x80)
#define MHB_PEER_SHIFT     (7)
#define MHB_PEER_BIT       (0x1)
#define MHB_PEER_WIDTH     (1)

#define MHB_INSTANCE_MASK  (0x60)
#define MHB_INSTANCE_SHIFT (5)
#define MHB_INSTANCE_BIT   (0x3)
#define MHB_INSTANCE_WIDTH (2)

#define MHB_FUNC_MASK      (0x1f)
#define MHB_FUNC_SHIFT     (0)
#define MHB_FUNC_BIT       (0x1f)
#define MHB_FUNC_WIDTH     (5)

enum MHB_FUNC {
	MHB_FUNC_PM     = 0,
	MHB_FUNC_UART   = 1,
	MHB_FUNC_UNIPRO = 2,
	MHB_FUNC_CDSI   = 3,
	MHB_FUNC_I2S    = 4,
	MHB_FUNC_HSIC   = 5,
	MHB_FUNC_DIAG   = 6,
	MHB_FUNC_MAX,
};

#define MHB_ADDR(peer, inst, func) \
	((peer << MHB_PEER_SHIFT)     | \
	(inst << MHB_INSTANCE_SHIFT) | \
	(func << MHB_FUNC_SHIFT))

enum MHB_ADDR {
	MHB_ADDR_PM     = MHB_ADDR(0, 0, MHB_FUNC_PM),
	MHB_ADDR_UART   = MHB_ADDR(0, 0, MHB_FUNC_UART),
	MHB_ADDR_UNIPRO = MHB_ADDR(0, 0, MHB_FUNC_UNIPRO),
	MHB_ADDR_CDSI0  = MHB_ADDR(0, 0, MHB_FUNC_CDSI),
	MHB_ADDR_CDSI1  = MHB_ADDR(0, 1, MHB_FUNC_CDSI),
	MHB_ADDR_I2S    = MHB_ADDR(0, 0, MHB_FUNC_I2S),
	MHB_ADDR_HSIC   = MHB_ADDR(0, 0, MHB_FUNC_HSIC),
	MHB_ADDR_DIAG   = MHB_ADDR(0, 0, MHB_FUNC_DIAG),

	MHB_ADDR_PEER_PM     = MHB_ADDR(1, 0, MHB_FUNC_PM),
	MHB_ADDR_PEER_UART   = MHB_ADDR(1, 0, MHB_FUNC_UART),
	MHB_ADDR_PEER_UNIPRO = MHB_ADDR(1, 0, MHB_FUNC_UNIPRO),
	MHB_ADDR_PEER_CDSI0  = MHB_ADDR(1, 0, MHB_FUNC_CDSI),
	MHB_ADDR_PEER_CDSI1  = MHB_ADDR(1, 1, MHB_FUNC_CDSI),
	MHB_ADDR_PEER_I2S    = MHB_ADDR(1, 0, MHB_FUNC_I2S),
	MHB_ADDR_PEER_HSIC   = MHB_ADDR(1, 0, MHB_FUNC_HSIC),
	MHB_ADDR_PEER_DIAG   = MHB_ADDR(1, 0, MHB_FUNC_DIAG),
};

/* type */
#define MHB_RSP_MASK  (0x80)
#define MHB_RSP_SHIFT (7)
#define MHB_RSP_BIT   (0x1)
#define MHB_RSP_WIDTH (1)

#define MHB_NOT_MASK  (0x40)
#define MHB_NOT_SHIFT (6)
#define MHB_NOT_BIT   (0x1)
#define MHB_NOT_WIDTH (1)

#define MHB_MSG_MASK  (0x3f)
#define MHB_MSG_SHIFT (0)
#define MHB_MSG_BIT   (0x3f)
#define MHB_MSG_WIDTH (6)

/* PM */
#define MHB_TYPE_PM_SLEEP_REQ (0)
#define MHB_TYPE_PM_SLEEP_RSP (MHB_RSP_MASK|MHB_TYPE_PM_SLEEP_REQ)

#define MHB_TYPE_PM_WAKE_REQ (1)
#define MHB_TYPE_PM_WAKE_RSP (MHB_RSP_MASK|MHB_TYPE_PM_WAKE_REQ)

#define MHB_TYPE_PM_STATUS_REQ (2)
#define MHB_TYPE_PM_STATUS_RSP (MHB_RSP_MASK|MHB_TYPE_PM_STATUS_REQ)
#define MHB_TYPE_PM_STATUS_NOT (MHB_NOT_MASK|MHB_TYPE_PM_STATUS_REQ)

/* UART */
#define MHB_TYPE_UART_CONFIG_REQ (0)
#define MHB_TYPE_UART_CONFIG_RSP (MHB_RSP_MASK|MHB_TYPE_UART_CONFIG_REQ)

#define MHB_TYPE_UART_CONTROL_REQ (1)
#define MHB_TYPE_UART_CONTROL_RSP (MHB_RSP_MASK|MHB_TYPE_UART_CONTROL_REQ)

#define MHB_TYPE_UART_STATUS_REQ (2)
#define MHB_TYPE_UART_STATUS_RSP (MHB_RSP_MASK|MHB_TYPE_UART_STATUS_REQ)
#define MHB_TYPE_UART_STATUS_NOT (MHB_NOT_MASK|MHB_TYPE_UART_STATUS_REQ)

/* UniPro */
#define MHB_TYPE_UNIPRO_CONFIG_REQ (0)
#define MHB_TYPE_UNIPRO_CONFIG_RSP (MHB_RSP_MASK|MHB_TYPE_UNIPRO_CONFIG_REQ)

#define MHB_TYPE_UNIPRO_CONTROL_REQ (1)
#define MHB_TYPE_UNIPRO_CONTROL_RSP (MHB_RSP_MASK|MHB_TYPE_UNIPRO_CONTROL_REQ)

#define MHB_TYPE_UNIPRO_STATUS_REQ (2)
#define MHB_TYPE_UNIPRO_STATUS_RSP (MHB_RSP_MASK|MHB_TYPE_UNIPRO_STATUS_REQ)
#define MHB_TYPE_UNIPRO_STATUS_NOT (MHB_NOT_MASK|MHB_TYPE_UNIPRO_STATUS_REQ)

#define MHB_TYPE_UNIPRO_READ_ATTR_REQ (3)
#define MHB_TYPE_UNIPRO_READ_ATTR_RSP \
	(MHB_RSP_MASK|MHB_TYPE_UNIPRO_READ_ATTR_REQ)

#define MHB_TYPE_UNIPRO_WRITE_ATTR_REQ (4)
#define MHB_TYPE_UNIPRO_WRITE_ATTR_RSP \
	(MHB_RSP_MASK|MHB_TYPE_UNIPRO_WRITE_ATTR_REQ)

#define MHB_TYPE_UNIPRO_STATS_REQ (5)
#define MHB_TYPE_UNIPRO_STATS_RSP (MHB_RSP_MASK|MHB_TYPE_UNIPRO_STATS_REQ)
#define MHB_TYPE_UNIPRO_STATS_NOT (MHB_NOT_MASK|MHB_TYPE_UNIPRO_STATS_REQ)

/* CDSI */
#define MHB_TYPE_CDSI_CONFIG_REQ (0)
#define MHB_TYPE_CDSI_CONFIG_RSP (MHB_RSP_MASK|MHB_TYPE_CDSI_CONFIG_REQ)

#define MHB_TYPE_CDSI_CONTROL_REQ (1)
#define MHB_TYPE_CDSI_CONTROL_RSP (MHB_RSP_MASK|MHB_TYPE_CDSI_CONTROL_REQ)

#define MHB_TYPE_CDSI_STATUS_REQ (2)
#define MHB_TYPE_CDSI_STATUS_RSP (MHB_RSP_MASK|MHB_TYPE_CDSI_STATUS_REQ)
#define MHB_TYPE_CDSI_STATUS_NOT (MHB_NOT_MASK|MHB_TYPE_CDSI_STATUS_REQ)

#define MHB_TYPE_CDSI_READ_CMDS_REQ (3)
#define MHB_TYPE_CDSI_READ_CMDS_RSP (MHB_RSP_MASK|MHB_TYPE_CDSI_READ_CMDS_REQ)

#define MHB_TYPE_CDSI_WRITE_CMDS_REQ (4)
#define MHB_TYPE_CDSI_WRITE_CMDS_RSP (MHB_RSP_MASK|MHB_TYPE_CDSI_WRITE_CMDS_REQ)

#define MHB_TYPE_CDSI_READ_PANEL_INFO_REQ (5)
#define MHB_TYPE_CDSI_READ_PANEL_INFO_RSP \
    (MHB_RSP_MASK|MHB_TYPE_CDSI_READ_PANEL_INFO_REQ)

#define MHB_TYPE_CDSI_UNCONFIG_REQ (6)
#define MHB_TYPE_CDSI_UNCONFIG_RSP (MHB_RSP_MASK|MHB_TYPE_CDSI_UNCONFIG_REQ)

/* I2S */
#define MHB_TYPE_I2S_CONFIG_REQ (0)
#define MHB_TYPE_I2S_CONFIG_RSP (MHB_RSP_MASK|MHB_TYPE_I2S_CONFIG_REQ)

#define MHB_TYPE_I2S_CONTROL_REQ (1)
#define MHB_TYPE_I2S_CONTROL_RSP (MHB_RSP_MASK|MHB_TYPE_I2S_CONTROL_REQ)

#define MHB_TYPE_I2S_STATUS_REQ (2)
#define MHB_TYPE_I2S_STATUS_RSP (MHB_RSP_MASK|MHB_TYPE_I2S_STATUS_REQ)
#define MHB_TYPE_I2S_STATUS_NOT (MHB_NOT_MASK|MHB_TYPE_I2S_STATUS_REQ)

/* HSIC */
#define MHB_TYPE_HSIC_CONFIG_REQ (0)
#define MHB_TYPE_HSIC_CONFIG_RSP (MHB_RSP_MASK|MHB_TYPE_HSIC_CONFIG_REQ)

#define MHB_TYPE_HSIC_CONTROL_REQ (1)
#define MHB_TYPE_HSIC_CONTROL_RSP (MHB_RSP_MASK|MHB_TYPE_HSIC_CONTROL_REQ)

#define MHB_TYPE_HSIC_STATUS_REQ (2)
#define MHB_TYPE_HSIC_STATUS_RSP (MHB_RSP_MASK|MHB_TYPE_HSIC_STATUS_REQ)
#define MHB_TYPE_HSIC_STATUS_NOT (MHB_NOT_MASK|MHB_TYPE_HSIC_STATUS_REQ)

/* Diagnostics */
#define MHB_TYPE_DIAG_LOG_REQ (0)
#define MHB_TYPE_DIAG_LOG_RSP (MHB_RSP_MASK|MHB_TYPE_DIAG_LOG_REQ)
#define MHB_TYPE_DIAG_LOG_NOT (MHB_NOT_MASK|MHB_TYPE_DIAG_LOG_REQ)

#define MHB_TYPE_DIAG_LAST_REQ (1)
#define MHB_TYPE_DIAG_LAST_RSP (MHB_RSP_MASK|MHB_TYPE_DIAG_LAST_REQ)

#define MHB_TYPE_DIAG_MODE_REQ (2)
#define MHB_TYPE_DIAG_MODE_RSP (MHB_RSP_MASK|MHB_TYPE_DIAG_MODE_REQ)

#define MHB_TYPE_DIAG_LOOP_REQ (3)
#define MHB_TYPE_DIAG_LOOP_RSP (MHB_RSP_MASK|MHB_TYPE_DIAG_LOOP_REQ)

#define MHB_TYPE_DIAG_CONTROL_REQ (4)
#define MHB_TYPE_DIAG_CONTROL_RSP (MHB_RSP_MASK|MHB_TYPE_DIAG_COMMAND_REQ)

#define MHB_TYPE_DIAG_REG_LOG_REQ (5)
#define MHB_TYPE_DIAG_REG_LOG_RSP (MHB_RSP_MASK|MHB_TYPE_DIAG_REG_LOG_REQ)

#define MHB_TYPE_DIAG_ID_REQ (6)
#define MHB_TYPE_DIAG_ID_NOT (MHB_NOT_MASK|MHB_TYPE_DIAG_ID_REQ)

/* result */
enum MHB_RESULT {
	MHB_RESULT_SUCCESS       = 0x00,
	MHB_RESULT_INTERRUPTED   = 0x01,
	MHB_RESULT_TIMEOUT       = 0x02,
	MHB_RESULT_NO_MEMORY     = 0x03,
	MHB_RESULT_PROTOCOL_BAD  = 0x04,
	MHB_RESULT_OVERFLOW      = 0x05,
	MHB_RESULT_INVALID       = 0x06,
	MHB_RESULT_RETRY         = 0x07,
	MHB_RESULT_NONEXISTENT   = 0x08,
	MHB_RESULT_UNKNOWN_ERROR = 0xfe,
	MHB_RESULT_INTERNAL      = 0xff,
};

/* crc */
#define MHB_CRC_SIZE (sizeof(uint16_t))

/* payloads */
#define MHB_MAX_PAYLOAD_SIZE (MHB_MAX_MSG_SIZE - MHB_HDR_SIZE - MHB_CRC_SIZE)

/* PM */
enum {
	MHB_PM_STATUS_PEER_NONE         = 0,
	MHB_PM_STATUS_PEER_ON           = 1,
	MHB_PM_STATUS_PEER_RESET        = 2,
	MHB_PM_STATUS_PEER_CONNECTED    = 3,
	MHB_PM_STATUS_PEER_DISCONNECTED = 4,
};

struct mhb_pm_status_not {
	uint32_t status;
} __attribute__((packed));

/* UART */
struct mhb_uart_config_req {
	uint32_t baud;
} __attribute__((packed));

struct mhb_uart_config_rsp {
	uint32_t baud;
} __attribute__((packed));

/* UNIPRO */
struct mhb_unipro_gear {
	uint8_t pwrmode;
	uint8_t tx;
	uint8_t rx;
	uint8_t series;
} __attribute__((packed));

struct mhb_unipro_control_req {
	struct mhb_unipro_gear gear;
} __attribute__((packed));

struct mhb_unipro_status_rsp {
	uint32_t status;
} __attribute__((packed));

struct mhb_unipro_read_attr_req {
	uint16_t attribute;
	uint16_t selector;
	uint8_t peer;
} __attribute__((packed));

struct mhb_unipro_read_attr_rsp {
	uint32_t value;
} __attribute__((packed));

struct mhb_unipro_write_attr_req {
	uint16_t attribute;
	uint16_t selector;
	uint8_t peer;
	uint32_t value;
} __attribute__((packed));

struct mhb_unipro_stats {
	/* L1 */
	uint32_t phy_lane_err;
	/* L1.5 */
	uint32_t pa_lane_reset_tx;
	uint32_t pa_lane_reset_rx;
	/* L2 */
	uint32_t d_nac_received;
	uint32_t d_tcx_replay_timer_expired;
	uint32_t d_afcx_request_timer_expired;
	uint32_t d_fcx_protectiong_timer_expired;
	uint32_t d_crc_error;
	uint32_t d_rx_buffer_overflow;
	uint32_t d_max_frame_length_exceeded;
	uint32_t d_wrong_sequence_number;
	uint32_t d_afc_frame_syntax_error;
	uint32_t d_nac_frame_syntax_error;
	uint32_t d_eof_syntax_error;
	uint32_t d_frame_syntax_error;
	uint32_t d_bad_control_symbol_type;
	uint32_t d_pa_init_error;
	uint32_t d_pa_error_ind_received;
	/* L3 */
	uint32_t n_unsupported_header_type;
	uint32_t n_bad_device_id_encoding;
	uint32_t n_lhdr_trap_packet_dropping;
	/* L4 */
	uint32_t t_unsupported_header_type;
	uint32_t t_unknown_cport_id;
	uint32_t t_no_connection_rx;
	uint32_t t_controlled_segment_dropping;
	uint32_t t_bad_tc;
	uint32_t t_e2e_credit_overflow;
	uint32_t t_safety_valve_dropping;
} __attribute__((packed));

struct mhb_unipro_stats_rsp {
	struct mhb_unipro_stats stats;
} __attribute__((packed));

struct mhb_unipro_stats_not {
	struct mhb_unipro_stats stats;
} __attribute__((packed));

/* CDSI */
enum mhb_csi_vsync_mode {
    MHB_CDSI_VSYNC_MODE_NONE = 0x00,
    MHB_CDSI_VSYNC_MODE_GPIO = 0x01,
    MHB_CDSI_VSYNC_MODE_DCS  = 0x02,
};

enum mhb_cdsi_eot_mode {
    MHB_CDSI_EOT_MODE_NONE   = 0x00,
    MHB_CDSI_EOT_MODE_APPEND = 0x01,
};

enum mhb_cdsi_traffic_mode {
    MHB_CDSI_TRAFFIC_MODE_NON_BURST_SYNC_PULSE = 0x00,
    MHB_CDSI_TRAFFIC_MODE_NON_BURST_SYNC_EVENT = 0x01,
    MHB_CDSI_TRAFFIC_MODE_BURST                = 0x02,
};

/* config */
struct mhb_cdsi_config {
	uint8_t direction; /* RX: 0 (CDSI -> UniPro), TX: 1 (UniPro -> CDSI) */
	uint8_t mode;                    /* DSI: 0, CSI: 1 */

	uint8_t rx_num_lanes;            /* 1 to 4 */
	uint8_t tx_num_lanes;            /* 1 to 4 */
	uint32_t rx_bits_per_lane;       /* bits-per-lane */
	uint32_t tx_bits_per_lane;       /* bits-per-lane */

	uint32_t hs_rx_timeout;

	uint32_t pll_frs;
	uint32_t pll_prd;
	uint32_t pll_fbd;

	uint32_t framerate;              /* frames-per-second */

	uint32_t width;                  /* pixels */
	uint32_t height;                 /* pixels */
	uint16_t physical_width;         /* millimeters */
	uint16_t physical_height;        /* millimeters */

	uint32_t bpp;                    /* bits-per-pixel */

	uint32_t vss_control_payload;
	uint8_t bta_enabled;             /* 0: disabled, 1: enabled */
	uint8_t continuous_clock;        /* 0: off, 1: on */
	uint8_t blank_packet_enabled;
	uint8_t video_mode;              /* 0: command, 1: video */
	uint8_t color_bar_enabled;       /* 0: disabled */
	uint8_t keep_alive;              /* 0: disabled */

	uint8_t t_clk_pre;               /* nanoseconds */
	uint8_t t_clk_post;              /* nanoseconds */

	uint8_t horizontal_front_porch;  /* pixels */
	uint8_t horizontal_back_porch;   /* pixels */
	uint8_t horizontal_pulse_width;  /* pixels */
	uint8_t horizontal_sync_skew;    /* pixels */
	uint8_t horizontal_left_border;  /* pixels */
	uint8_t horizontal_right_border; /* pixels */

	uint8_t vertical_front_porch;    /* lines */
	uint8_t vertical_back_porch;     /* lines */
	uint8_t vertical_pulse_width;    /* lines */
	uint8_t vertical_top_border;     /* lines */
	uint8_t vertical_bottom_border;  /* lines */

	uint8_t vsync_mode;              /* mhb_csi_vsync_mode */
	uint8_t eot_mode;                /* mhb_cdsi_eot_mode */
	uint8_t traffic_mode;            /* mhb_cdsi_traffic_mode */

	uint8_t reserved[6];
} __attribute__((packed));

struct mhb_cdsi_config_req {
	struct mhb_cdsi_config cfg;
} __attribute__((packed));

/* control */
enum {
	MHB_CDSI_COMMAND_NONE  = 0,
	MHB_CDSI_COMMAND_STOP  = 1,
	MHB_CDSI_COMMAND_START = 2,
};

struct mhb_cdsi_control_req {
	uint8_t command;
} __attribute__((packed));

/* DSI and DCS */
#define MHB_CTYPE_HS_FLAG    (0x0)
#define MHB_CTYPE_LP_FLAG    (0x8)
#define MHB_CTYPE_SHORT_FLAG (0x0)
#define MHB_CTYPE_LONG_FLAG  (0x4)

#define MHB_CTYPE_HS_SHORT (MHB_CTYPE_HS_FLAG|MHB_CTYPE_SHORT_FLAG)
#define MHB_CTYPE_HS_LONG  (MHB_CTYPE_HS_FLAG|MHB_CTYPE_LONG_FLAG)
#define MHB_CTYPE_LP_SHORT (MHB_CTYPE_LP_FLAG|MHB_CTYPE_SHORT_FLAG)
#define MHB_CTYPE_LP_LONG  (MHB_CTYPE_LP_FLAG|MHB_CTYPE_LONG_FLAG)

/* Data Types for Processor-sourced Packets */
#define MHB_DTYPE_GEN_SHORT_WRITE0 0x03
#define MHB_DTYPE_GEN_SHORT_WRITE1 0x13
#define MHB_DTYPE_GEN_SHORT_WRITE2 0x23
#define MHB_DTYPE_GEN_READ0        0x04
#define MHB_DTYPE_GEN_READ1        0x14
#define MHB_DTYPE_GEN_READ2        0x24
#define MHB_DTYPE_DCS_WRITE0       0x05
#define MHB_DTYPE_DCS_WRITE1       0x15
#define MHB_DTYPE_DCS_READ0        0x06
#define MHB_DTYPE_MAX_RET_PKT      0x37
#define MHB_DTYPE_GEN_LONG_WRITE   0x29
#define MHB_DTYPE_DCS_LONG_WRITE   0x39

/* read/write command */
struct mhb_cdsi_cmd {
	uint8_t ctype;  /* MHB_CTYPE_* */
	uint8_t dtype;  /* MHB_DTYPE_* */
	uint16_t length;
	uint32_t delay; /* minimum microseconds to wait after command */
	union {
		uint16_t spdata;
		uint32_t lpdata[2];
	} u;
} __attribute__((packed));

struct mhb_cdsi_read_cmds_req {
	struct mhb_cdsi_cmd cmds[0];
} __attribute__((packed));

struct mhb_cdsi_write_cmds_req {
	struct mhb_cdsi_cmd cmds[0];
} __attribute__((packed));

/* read panel info */
struct mhb_dsi_panel_info {
	uint16_t supplier_id;
	uint8_t id0;
	uint8_t id1;
	uint8_t id2;
} __attribute__((packed));

struct mhb_cdsi_write_cmds_rsp {
	struct mhb_dsi_panel_info info;
} __attribute__((packed));

/* I2S */
#define MHB_I2S_EDGE_RISING  0
#define MHB_I2S_EDGE_FALLING 1

#define MHB_I2S_WCLK_POLARITY_NORMAL  0
#define MHB_I2S_WCLK_POLARITY_REVERSE 1

#define MHB_I2S_PROTOCOL_I2S       0
#define MHB_I2S_PROTOCOL_PCM       1
#define MHB_I2S_PROTOCOL_LR_STEREO 2

#define MHB_I2S_ROLE_SLAVE   0
#define MHB_I2S_ROLE_MASTER  1

/* config */
struct mhb_i2s_config {
	uint32_t sample_rate;    /* in Hz (e.g. 48000) */
	uint8_t sample_size;     /* in bits (e.g. 8, 16) */
	uint8_t num_channels;    /* 1: mono, 2: stereo */
	uint8_t wclk_polarity;   /* MHB_I2S_WCLK_POLARITY_* */
	uint8_t protocol;        /* MHB_I2S_PROTOCOL_*/
	uint8_t rx_edge;         /* MHB_I2S_EDGE_* */
	uint8_t tx_edge;         /* MHB_I2S_EDGE_* */
	uint8_t wclk_edge;       /* MHB_I2S_EDGE_* */
	uint8_t clk_role;        /* MHB_I2S_ROLE_*/
} __attribute__((packed));

struct mhb_i2s_config_req {
	struct mhb_i2s_config cfg;
} __attribute__((packed));

/* control */
#define MHB_I2S_COMMAND_NONE      0
/*
 * The following commands are used in normal operation to start and stop
 * I2S tunneling.
 */
#define MHB_I2S_COMMAND_TX_STOP   1  /* Stop transmit and shutdown all clocks. */
#define MHB_I2S_COMMAND_TX_START  2  /* Enable, Arm, and start transmit. */
#define MHB_I2S_COMMAND_RX_STOP   3  /* Stop receive and shutdown all clocks. */
#define MHB_I2S_COMMAND_RX_START  4  /* Enable, Arm, and start receive. */
/*
 * The following commands are used in test modes to verify I2S tunneling is
 * operating as expected on hardware.  They do not need to be used in normal
 * operation.
 */
#define MHB_I2S_COMMAND_ENABLE    5  /* Enable the clock to the module, thus power will be drawn. */
#define MHB_I2S_COMMAND_DISABLE   6  /* Disable the clock to the module, thus power will be reduced. */
#define MHB_I2S_COMMAND_ARM       7  /* Arm the system which starts the PLL and it will start
										 on the first packet or MANUAL_START. */
#define MHB_I2S_COMMAND_DISARM    8  /* Disarm the system which stops transmitting/receiving and
										 shuts down the PLL. */
#define MHB_I2S_COMMAND_TRIGGER   9  /* Starts the system after it has been armed. */

struct mhb_i2s_control_req {
	uint8_t command;
} __attribute__((packed));

/* HSIC */

#define MHB_HSIC_COMMAND_NONE     0
#define MHB_HSIC_COMMAND_STOP     1  /* Stop USB tunneling over HSIC */
#define MHB_HSIC_COMMAND_START    2  /* Start USB tunneling over HSIC */

struct mhb_hsic_control_req {
	uint8_t command;
} __attribute__((packed));

/* Diag */
struct mhb_diag_mode_req {
	uint32_t mode;
} __attribute__((packed));

struct mhb_diag_control_req {
	uint8_t command;
} __attribute__((packed));

#define MHB_DIAG_CONTROL_NONE           0
#define MHB_DIAG_CONTROL_REGLOG_FIFO    1  /* Set the reglog mode to FIFO. */
#define MHB_DIAG_CONTROL_REGLOG_STACK   2  /* Set the reglog mode to stack. */

struct mhb_diag_id_not {
	uint32_t unipro_mid;
	uint32_t unipro_pid;
	uint32_t vid;
	uint32_t pid;
	uint16_t major_version;
	uint16_t minor_version;
	char build[32];
	char reserved[32];
} __attribute__((packed));

#endif
