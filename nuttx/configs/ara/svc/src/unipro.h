/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 *
 * FIXME: this isn't really Google Confidential -- bridge ASIC code
 * will eventually want it, too.
 */

#ifndef _ARA_UNIPRO_H_
#define _ARA_UNIPRO_H_

#include <stdint.h>

/**
 * UniPro power mode.
 *
 * The TX and RX directions of a UniPro link can have different power
 * modes.
 */
enum unipro_pwr_mode {
    /** Permanently in FAST_STATE; i.e. a high speed (HS) M-PHY gear. */
    UNIPRO_FAST_MODE = 1,
    /** Permanently in SLOW_STATE; i.e. a PWM M-PHY gear. */
    UNIPRO_SLOW_MODE = 2,
    /** Alternating automatically between FAST_STATE (HS) and SLEEP_STATE. */
    UNIPRO_FASTAUTO_MODE = 4,
    /** Alternating automatically between SLOW_STATE (PWM) and SLEEP_STATE. */
    UNIPRO_SLOWAUTO_MODE = 5,

    /** Hibernate mode */
    UNIPRO_HIBERNATE_MODE = -1,
    /** Powered off */
    UNIPRO_OFF_MODE = -2,

    /**
     * Special value to use when you don't want to change one link
     * direction's power mode */
    UNIPRO_MODE_UNCHANGED = 7,
};

/** @brief UniPro frequency series in high speed mode. */
enum unipro_hs_series {
    UNIPRO_HS_SERIES_UNCHANGED = 0,
    UNIPRO_HS_SERIES_A = 1,
    UNIPRO_HS_SERIES_B = 2,
};

/**
 * @brief User-data for peer DME during power mode reconfiguration.
 *
 * These are used to set remote timeout values during the link
 * configuration procedure.
 */
struct unipro_pwr_user_data {
    /* Try to set peer DL_FC0PROTECTIONTIMEOUTVAL */
#   define UPRO_PWRF_FC0  (1U << 0)
    /* Try to set peer DL_TC0REPLAYTIMEOUTVAL */
#   define UPRO_PWRF_TC0  (1U << 1)
    /* Try to set peer DL_AFC0REQTIMEOUTVAL */
#   define UPRO_PWRF_AFC0 (1U << 2)
    /* Try to set peer DL_FC1PROTECTIONTIMEOUTVAL */
#   define UPRO_PWRF_FC1  (1U << 3)
    /* Try to set peer DL_TC1REPLAYTIMEOUTVAL */
#   define UPRO_PWRF_TC1  (1U << 4)
    /* Try to set peer DL_AFC1REQTIMEOUTVAL */
#   define UPRO_PWRF_AFC1 (1U << 5)
    uint32_t flags;
    uint16_t upro_pwr_fc0_protection_timeout;
    uint16_t upro_pwr_tc0_replay_timeout;
    uint16_t upro_pwr_afc0_req_timeout;
    uint16_t upro_pwr_fc1_protection_timeout;
    uint16_t upro_pwr_tc1_replay_timeout;
    uint16_t upro_pwr_afc1_req_timeout;
    const uint16_t reserved_tc2[3];
    const uint16_t reserved_tc3[3];
};

/**
 * @brief UniPro link per-direction configuration.
 *
 * This is used to configure a unipro link in one direction.
 */
struct unipro_pwr_cfg {
    enum unipro_pwr_mode  upro_mode;   /**< Power mode to set. */
    unsigned int          upro_gear;   /**< M-PHY gear to use. */
    unsigned int          upro_nlanes; /**< Number of active data lanes. */
};

/**
 * @brief UniPro link configuration.
 */
struct unipro_link_cfg {
    enum unipro_hs_series upro_hs_ser; /**< Frequency series in HS mode. */
    struct unipro_pwr_cfg upro_tx_cfg; /**< Configuration for TX direction. */
    struct unipro_pwr_cfg upro_rx_cfg; /**< Configuration for RX direction. */

    /** User-data (e.g. L2 timers) */
    struct unipro_pwr_user_data upro_user;

#   define UPRO_LINKF_TX_TERMINATION (1U << 0) /**< TX termination is on. */
#   define UPRO_LINKF_RX_TERMINATION (1U << 1) /**< RX termination is on. */
#   define UPRO_LINKF_SCRAMBLING     (1U << 2) /**< Scrambling request. */
    unsigned int flags;
};

#define UNIPRO_PWR_CFG(_mode, gear, nlanes)                             \
    {                                                                   \
        .upro_mode = (_mode),                                           \
        .upro_gear = (gear),                                            \
        .upro_nlanes = (nlanes),                                        \
    }

#define UNIPRO_FAST_PWR_CFG(auto_, gear, nlanes)                        \
    UNIPRO_PWR_CFG((auto_) ? UNIPRO_FASTAUTO_MODE : UNIPRO_FAST_MODE,   \
                   (gear),                                              \
                   (nlanes))

#define UNIPRO_SLOW_PWR_CFG(auto_, gear, nlanes)                        \
    UNIPRO_PWR_CFG((auto_) ? UNIPRO_SLOWAUTO_MODE : UNIPRO_SLOW_MODE,   \
                   (gear),                                              \
                   (nlanes))

#endif  /* _ARA_UNIPRO_H_ */
