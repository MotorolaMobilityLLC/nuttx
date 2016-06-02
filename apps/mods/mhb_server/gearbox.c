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

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/util.h>

#include <nuttx/unipro/unipro.h>

#include "gearbox.h"

/* Union to hold the concrete gear requests that gearbox makes to the
 * UniPro stack.  The union is packed into a 32-bit value with a raw member
 * for efficient compares.
 */
union gear_state {
    struct {
        /* rx */
        uint32_t rx_gear:3;     /* 0-7 */
        uint32_t rx_mode:1;     /* 0: PWM, 1: High-Speed (HS) */
        uint32_t rx_auto:1;     /* 0: fixed, 1: auto */
        /* tx */
        uint32_t tx_gear:3;     /* 0-7 */
        uint32_t tx_mode:1;     /* 0: PWM, 1: High-Speed (HS) */
        uint32_t tx_auto:1;     /* 0: fixed, 1: auto */
        /* common */
        uint32_t series:1;      /* 0: A, 1: B */
        uint32_t termination:1; /* 0: no termination, 1: termination */
    };
    uint32_t raw;
};

const static union gear_state INITIAL_STATE = {
    .rx_gear = 1, .rx_mode = 0, .rx_auto = 0,
    .tx_gear = 1, .tx_mode = 0, .tx_auto = 0,
    .series = 0, .termination = 0, };

const static struct gear_request NULL_REQUEST =
    { .rx_max_mbps = 0, .rx_auto = 1,
      .tx_max_mbps = 0, .tx_auto = 1, };

enum {
    GEARBOX_STATE_DOWN = 0,
    GEARBOX_STATE_IDLE,
    GEARBOX_STATE_SHIFTING,
    GEARBOX_STATE_RETRY,
};

struct gearbox {
    sem_t mutex;

    /* Gear shifting is asynchronous and can fail.  At any given time, there
     * are four states:
          IDLE:     All clients are satisfied with the current gear
          SHIFTING: One or more clients requested a gear shift and it is in
                    progress.  No additional gear shifts are required.
          RETRY:    The previous gear shift failed.  Gearbox will try again
                    after a timeout or immediately if another client requests a
                    change.
          DOWN:     The UniPro link is down.
     */
    uint32_t state;

    /* The last successfully completed request. */
    union gear_state current;
    /* The request that is in-progress. */
    union gear_state in_progress;
    /* Current requests for each client. */
    struct gear_request client_requests[5];
    /* The next available client id. */
    int next_id;
};

struct mbps_to_gear {
    uint32_t max_mbps;
    uint8_t gear:3;
    uint8_t mode:1;
    uint8_t series:1;
};

/* Macro to add reduce the theoretical max to a practical max */
#define GMAX(x) (x * 0.75)

/* Table that maps abstract megabits-per-second to concrete gear parameters. */
static const struct mbps_to_gear MBPS_TO_GEAR[] =
{
    /* PWM-G0    { .max_mbps =    GMAX(3), .gear = 0, .mode = 0, .series = 0, }, */
    /* PWM-G1 */ { .max_mbps =    GMAX(9), .gear = 1, .mode = 0, .series = 0, },
    /* PWM-G2 */ { .max_mbps =   GMAX(18), .gear = 2, .mode = 0, .series = 0, },
    /* PWM-G3 */ { .max_mbps =   GMAX(36), .gear = 3, .mode = 0, .series = 0, },
    /* PWM-G4 */ { .max_mbps =   GMAX(72), .gear = 4, .mode = 0, .series = 0, },
    /* PWM-G5    { .max_mbps =  GMAX(144), .gear = 5, .mode = 0, .series = 0, }, */
    /* PWM-G6    { .max_mbps =  GMAX(288), .gear = 6, .mode = 0, .series = 0, }, */
    /* PWM-G7    { .max_mbps =  GMAX(576), .gear = 7, .mode = 0, .series = 0, }, */
    /* HS-G1a */ { .max_mbps = GMAX(1248), .gear = 1, .mode = 1, .series = 0, },
    /* HS-G1b */ { .max_mbps = GMAX(1458), .gear = 1, .mode = 1, .series = 1, },
    /* HS-G2a */ { .max_mbps = GMAX(2496), .gear = 2, .mode = 1, .series = 0, },
    /* HS-G2b */ { .max_mbps = GMAX(2915), .gear = 2, .mode = 1, .series = 1, },
    /* HS-G3a */ { .max_mbps = GMAX(4992), .gear = 3, .mode = 1, .series = 0, },
    /* HS-G3b */ { .max_mbps = GMAX(5830), .gear = 3, .mode = 1, .series = 1, },
    /* Always top-out at High-Speed gear 3 Series B */
    { .max_mbps = 0xffffffff, .gear = 3, .mode = 1, .series = 1, },
};

#if CONFIG_DEBUG_VERBOSE
static inline void _gearbox_dump_gear_state(union gear_state state) {
    dbg("rx=%d:%d:%d, tx=%d:%d:%d, series=%d, termination=%d\n",
        state.rx_gear, state.rx_mode, state.rx_auto,
        state.tx_gear, state.tx_mode, state.tx_auto,
        state.series, state.termination);
}

static inline void _gearbox_dump_gear_request(struct gear_request *request) {
    dbg("rx=%d:%d, tx=%d:%d\n",
        request->rx_max_mbps, request->rx_auto,
        request->tx_max_mbps, request->tx_auto);
}

static void _gearbox_dump(struct gearbox *gearbox) {
    dbg("state=%d, next_id=%d\n", gearbox->state, gearbox->next_id);
    dbg("current:\n");
    _gearbox_dump_gear_state(gearbox->current);
    dbg("in_progress:\n");
    _gearbox_dump_gear_state(gearbox->in_progress);

    size_t i;
    for (i = 0; i < ARRAY_SIZE(gearbox->client_requests); i++) {
        dbg("[%d]:\n", i);
        _gearbox_dump_gear_request(&gearbox->client_requests[i]);
    }
}
#endif

static union gear_state _gearbox_request_to_state(struct gear_request *request) {
    /* Scan the table from beginning to end and find the first gear that
     * has enough bandwidth in each direction.  Once both directions are
     * satisfied, break out.
     */
    ssize_t i;
    const struct mbps_to_gear *rx_row = NULL;
    const struct mbps_to_gear *tx_row = NULL;
    for (i = 0; i < ARRAY_SIZE(MBPS_TO_GEAR); i++) {
        const struct mbps_to_gear *row = &MBPS_TO_GEAR[i];

        if (!rx_row) {
            if (request->rx_max_mbps < row->max_mbps) {
                rx_row = row;
            }
        }

        if (!tx_row) {
            if (request->tx_max_mbps < row->max_mbps) {
                tx_row = row;
            }
        }

        if (rx_row && tx_row) {
            /* Both rx and tx are complete. */
            break;
        }
    }

    if (!rx_row || !tx_row) {
        /* Something has gone horribly wrong. */
        dbg("ERROR: Failed to find gears.\n");
    }

    union gear_state state;

    state.rx_gear = rx_row->gear;
    state.rx_mode = rx_row->mode;
    /* Use the auto-mode directly */
    state.rx_auto = request->rx_auto;

    state.tx_gear = tx_row->gear;
    state.tx_mode = tx_row->mode;
    /* Use the auto-mode directly */
    state.tx_auto = request->tx_auto;

    /* Use Series B if either direction requires it. */
    state.series = rx_row->series || tx_row->series;

    /* Always use termination */
    state.termination = 1;

    return state;
}

static inline bool _gearbox_validate_id(struct gearbox *gearbox, int id) {
    if (id < 0) {
        return false;
    }

    if (id >= ARRAY_SIZE(gearbox->client_requests)) {
        return false;
    }

    return true;
}

static int _gearbox_get_current(struct gearbox *gearbox) {
    int result;

    uint32_t rx_mode;
    uint32_t rx_pwm_gear;
    uint32_t rx_hs_gear;
    uint32_t rx_series;

    uint32_t tx_mode;
    uint32_t tx_pwm_gear;
    uint32_t tx_hs_gear;
    uint32_t tx_series;

    uint32_t pwrmode;
    uint32_t series;

    /* Selectors */
    const uint16_t RX_SEL = 4;
    const uint16_t TX_SEL = 0;
    const uint16_t PA_SEL = 0;

    /* Read current attribute values */
    result = unipro_attr_local_read(RX_MODE, &rx_mode, RX_SEL);
    if (result) {
        goto error;
    }
    vdbg("rx_mode=%d\n", rx_mode);

    result = unipro_attr_local_read(RX_PWMGEAR, &rx_pwm_gear, RX_SEL);
    if (result) {
        goto error;
    }
    vdbg("rx_pwm_gear=%d\n", rx_pwm_gear);

    result = unipro_attr_local_read(RX_HSGEAR, &rx_hs_gear, RX_SEL);
    if (result) {
        goto error;
    }
    vdbg("rx_hs_gear=%d\n", rx_hs_gear);

    result = unipro_attr_local_read(RX_HSRATE_SERIES, &rx_series, RX_SEL);
    if (result) {
        goto error;
    }
    vdbg("rx_series=%d\n", rx_series);

    result = unipro_attr_local_read(TX_MODE, &tx_mode, TX_SEL);
    if (result) {
        goto error;
    }
    vdbg("tx_mode=%d\n", tx_mode);

    result = unipro_attr_local_read(TX_PWMGEAR, &tx_pwm_gear, TX_SEL);
    if (result) {
        goto error;
    }
    vdbg("tx_pwm_gear=%d\n", tx_pwm_gear);

    result = unipro_attr_local_read(TX_HSGEAR, &tx_hs_gear, TX_SEL);
    if (result) {
        goto error;
    }
    vdbg("tx_hs_gear=%d\n", tx_hs_gear);

    result = unipro_attr_local_read(TX_HSRATE_SERIES, &tx_series, TX_SEL);
    if (result) {
        goto error;
    }
    vdbg("tx_series=%d\n", tx_series);

    result = unipro_attr_local_read(PA_PWRMODE, &pwrmode, PA_SEL);
    if (result) {
        goto error;
    }
    vdbg("pwrmode=%d\n", pwrmode);

    result = unipro_attr_local_read(PA_HSSERIES, &series, PA_SEL);
    if (result) {
        goto error;
    }
    vdbg("series=%d\n", series);

    /* Convert attribute values into gear_state */
    if (rx_mode == 1) {
        gearbox->current.rx_mode = 0;
        gearbox->current.rx_gear = rx_pwm_gear;
    } else if (rx_mode == 2) {
        gearbox->current.rx_mode = 1;
        gearbox->current.rx_gear = rx_hs_gear;
    }

    switch ((pwrmode >> 4) & 0xf) {
    case UNIPRO_FASTAUTO_MODE:
    case UNIPRO_SLOWAUTO_MODE:
        gearbox->current.rx_auto =  1;
        break;
    case UNIPRO_FAST_MODE:
    case UNIPRO_SLOW_MODE:
        gearbox->current.rx_auto =  0;
        break;
    }

    if (tx_mode == 1) {
        gearbox->current.tx_mode = 0;
        gearbox->current.tx_gear = tx_pwm_gear;
    } else if (tx_mode == 2) {
        gearbox->current.tx_mode = 1;
        gearbox->current.tx_gear = tx_hs_gear;
    }

    switch (pwrmode & 0xf) {
    case UNIPRO_FASTAUTO_MODE:
    case UNIPRO_SLOWAUTO_MODE:
        gearbox->current.tx_auto =  1;
        break;
    case UNIPRO_FAST_MODE:
    case UNIPRO_SLOW_MODE:
        gearbox->current.tx_auto =  0;
        break;
    }

    gearbox->current.series = tx_series == UNIPRO_HS_SERIES_A ? 0 : 1;
    gearbox->current.termination = 1;

    return 0;

error:
    dbg("ERROR: Failed to read current gear.\n");
    return result;
}

static inline uint32_t _gearbox_lookup_mode(int mode, int _auto) {
    if (mode && _auto) {
        return UNIPRO_FASTAUTO_MODE;
    } else if (mode && !_auto) {
        return UNIPRO_FAST_MODE;
    } else if (!mode && _auto) {
        return UNIPRO_SLOWAUTO_MODE;
    } else if (!mode && !_auto) {
        return UNIPRO_SLOW_MODE;
    }

    __builtin_unreachable();
}

static int _gearbox_start_shift(struct gearbox *gearbox) {
    /* Calculate power-mode */
    uint32_t pwrmode = _gearbox_lookup_mode(gearbox->in_progress.rx_mode,
                                           gearbox->in_progress.rx_auto);
    pwrmode <<= 4;
    pwrmode |= _gearbox_lookup_mode(gearbox->in_progress.tx_mode,
                                    gearbox->in_progress.tx_auto);

    uint32_t series = gearbox->in_progress.series ?
                                    UNIPRO_HS_SERIES_B : UNIPRO_HS_SERIES_A;

    /* Start the gear shift. */

    /* TX */
    unipro_attr_local_write(PA_TXGEAR, gearbox->in_progress.tx_gear, 0);
    unipro_attr_local_write(PA_TXTERMINATION, gearbox->in_progress.termination, 0);
    unipro_attr_local_write(PA_HSSERIES, series, 0);
    unipro_attr_local_write(PA_ACTIVETXDATALANES, 1, 0);

    /* RX */
    unipro_attr_local_write(PA_RXGEAR, gearbox->in_progress.rx_gear, 0);
    unipro_attr_local_write(PA_RXTERMINATION, gearbox->in_progress.termination, 0);
    unipro_attr_local_write(PA_ACTIVERXDATALANES, 1, 0);

    /* Power Mode */
    unipro_attr_local_write(PA_PWRMODE, pwrmode, 0);
    return 0;
}

static int _gearbox_recalculate(struct gearbox *gearbox) {
    int result;

    struct gear_request request;
    request.tx_max_mbps = 0;
    request.rx_max_mbps = 0;
    /* Try auto-mode if possible. */
    request.tx_auto = 1;
    request.rx_auto = 1;

    /* Determine the aggregate request. */
    size_t i;
    for (i = 0; i < ARRAY_SIZE(gearbox->client_requests); i++) {
        struct gear_request *client_request = &gearbox->client_requests[i];
        request.rx_max_mbps += client_request->rx_max_mbps;
        /* The client only gets a say about auto if the client requested some bandwidth. */
        if (client_request->rx_max_mbps && !client_request->rx_auto) {
            request.rx_auto = 0;
        }

        request.tx_max_mbps += client_request->tx_max_mbps;
        /* The client only gets a say about auto if the client requested some bandwidth. */
        if (client_request->tx_max_mbps && !client_request->tx_auto) {
            request.tx_auto = 0;
        }
    }

    union gear_state state = _gearbox_request_to_state(&request);
#if CONFIG_DEBUG_VERBOSE
    _gearbox_dump_gear_state(state);
#endif

    switch (gearbox->state) {
    case GEARBOX_STATE_IDLE:
    case GEARBOX_STATE_RETRY:
        if (gearbox->current.raw != state.raw) {
            /* Gearbox is not shifting so save this request as the in-progress
             * request and start shifting.  If gearbox failed the previous
             * shift, try again with this one.  Maybe things are better now.
             */
            gearbox->in_progress = state;
            gearbox->state = GEARBOX_STATE_SHIFTING;
            _gearbox_start_shift(gearbox);
            result = true;
        }
        result = 0;
        break;
    case GEARBOX_STATE_SHIFTING:
        /* Gearbox will recalculate when the shift is complete. */
        result = 0;
        break;
    case GEARBOX_STATE_DOWN:
        /* ignore */
        result = -EIO;
        break;
    default:
        dbg("ERROR: Invalid gearbox state: %d\n", gearbox->state);
        result = -EINVAL;
        break;
    }

#if CONFIG_DEBUG_VERBOSE
    _gearbox_dump(gearbox);
#endif

    return result;
}

/* System API */
struct gearbox *gearbox_initialize(void) {
    vdbg("\n");

    static struct gearbox g_gearbox;

    struct gearbox *gearbox = &g_gearbox;

    memset(gearbox, 0, sizeof(*gearbox));
    sem_init(&gearbox->mutex, 0, 1);
    gearbox->state = GEARBOX_STATE_IDLE;

    return gearbox;
}

void gearbox_uninitialize(struct gearbox *gearbox) {
    vdbg("\n");
    sem_destroy(&gearbox->mutex);
}

/* Client API */
int gearbox_register(struct gearbox *gearbox) {
    vdbg("\n");

    if (!gearbox) {
        return -ENODEV;
    }

    if (!_gearbox_validate_id(gearbox, gearbox->next_id)) {
        return -ENOTBLK;
    }

    sem_wait(&gearbox->mutex);

    int id = gearbox->next_id++;

    sem_post(&gearbox->mutex);

    return id;
}

int gearbox_request(struct gearbox *gearbox, int id,
    const struct gear_request *request) {
    vdbg("\n");

    if (!gearbox) {
        return -ENODEV;
    }

    if (!_gearbox_validate_id(gearbox, id)) {
        return -ENOTBLK;
    }

    /* Treat a NULL as a 'release' of the current request. */
    if (!request) {
        request = &NULL_REQUEST;
    }

    sem_wait(&gearbox->mutex);

    /* Save request */
    gearbox->client_requests[id] = *request;

    /* Process requests and start a gear shift if necessary. */
    int result = _gearbox_recalculate(gearbox);

    sem_post(&gearbox->mutex);

    return result;
}

int gearbox_unregister(struct gearbox *gearbox, int id) {
    vdbg("\n");

    if (!gearbox) {
        return -ENODEV;
    }

    if (!_gearbox_validate_id(gearbox, id)) {
        return -ENOTBLK;
    }

    /* Clear out the client's request and start a gear shift if necessary. */
    gearbox_request(gearbox, id, NULL);

    return 0;
}

/* UniPro events */
int gearbox_link_up(struct gearbox *gearbox) {
    vdbg("\n");

    if (!gearbox) {
        return -ENODEV;
    }

    sem_wait(&gearbox->mutex);

    gearbox->state = GEARBOX_STATE_IDLE;

    /* Reset the gear states. */
    gearbox->current = INITIAL_STATE;
    gearbox->in_progress = INITIAL_STATE;

    /* Clear all of the client requests. */
    memset(gearbox->client_requests, 0, sizeof(gearbox->client_requests));

    _gearbox_get_current(gearbox);

#if CONFIG_DEBUG_VERBOSE
    _gearbox_dump(gearbox);
#endif

    sem_post(&gearbox->mutex);

    return 0;
}

int gearbox_link_down(struct gearbox *gearbox) {
    vdbg("\n");

    if (!gearbox) {
        return -ENODEV;
    }

    sem_wait(&gearbox->mutex);

    gearbox->state = GEARBOX_STATE_DOWN;

    sem_post(&gearbox->mutex);

    return 0;
}

int gearbox_shift_complete(struct gearbox *gearbox, int err) {
    vdbg("\n");

    if (!gearbox) {
        return -ENODEV;
    }

    sem_wait(&gearbox->mutex);

    switch (gearbox->state) {
    case GEARBOX_STATE_SHIFTING:
        if (!err) {
            dbg("Gear shift successful\n");

            /* Update the current state. */
            gearbox->current = gearbox->in_progress;

            gearbox->state = GEARBOX_STATE_IDLE;

            /* Check if any new requests arrived while shifting. */
            _gearbox_recalculate(gearbox);
        } else {
            dbg("ERROR: gear shift failed: %d\n", err);

            /* Clear out the failed state. */
            gearbox->in_progress = gearbox->current;

            gearbox->state = GEARBOX_STATE_RETRY;
        }
        break;
    case GEARBOX_STATE_IDLE:
    case GEARBOX_STATE_RETRY:
        dbg("NOTE: Unexpected gear shift, read current\n", gearbox->state);

        /* The gear might have been changed by another mechanism,
         * read the current gearbox state. */
        _gearbox_get_current(gearbox);
        break;
    case GEARBOX_STATE_DOWN:
        /* ignore */
        break;
    default:
        dbg("ERROR: Invalid gearbox state: %d\n", gearbox->state);
        break;
    }

#if CONFIG_DEBUG_VERBOSE
    _gearbox_dump(gearbox);
#endif

    sem_post(&gearbox->mutex);

    return 0;
}
