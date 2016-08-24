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

#ifndef MHB_CSI_CAMERA_SM_H
#define MHB_CSI_CAMERA_SM_H

typedef enum {
    MHB_CAMERA_STATE_INVALID = -1,
    MHB_CAMERA_STATE_OFF = 0,
    MHB_CAMERA_STATE_WAIT_POWER_ON,
    MHB_CAMERA_STATE_ON,
    MHB_CAMERA_STATE_WAIT_STREAM,
    MHB_CAMERA_STATE_STREAMING,
    MHB_CAMERA_STATE_WAIT_STREAM_CLOSE,
    MHB_CAMERA_STATE_WAIT_OFF,
    MHB_CAMERA_STATE_MAX,
} mhb_camera_sm_state_t;

typedef enum {
    MHB_CAMERA_EV_FAIL = -1,
    MHB_CAMERA_EV_NONE = 0,
    MHB_CAMERA_EV_POWER_ON_REQ,
    MHB_CAMERA_EV_POWER_OFF_REQ,
    MHB_CAMERA_EV_STREAM_ON_REQ,
    MHB_CAMERA_EV_STREAM_OFF_REQ,
    MHB_CAMERA_EV_POWERED_ON,
    MHB_CAMERA_EV_CONFIGURED,
    MHB_CAMERA_EV_DECONFIGURED,
    MHB_CAMERA_EV_WAIT_OVER,
    MHB_CAMERA_EV_MAX,
} mhb_camera_sm_event_t;

#ifdef DEBUG
static char *mhb_camera_sm_states_str[] = {
    "MHB_CAMERA_STATE_OFF",
    "MHB_CAMERA_STATE_WAIT_POWER_ON",
    "MHB_CAMERA_STATE_ON",
    "MHB_CAMERA_STATE_WAIT_STREAM",
    "MHB_CAMERA_STATE_STREAMING",
    "MHB_CAMERA_STATE_WAIT_STREAM_CLOSE",
    "MHB_CAMERA_STATE_WAIT_OFF",
};

static char *mhb_camera_sm_events_str[] = {
    "MHB_CAMERA_EV_NONE",
    "MHB_CAMERA_EV_POWER_ON_REQ",
    "MHB_CAMERA_EV_POWER_OFF_REQ",
    "MHB_CAMERA_EV_STREAM_ON_REQ",
    "MHB_CAMERA_EV_STREAM_OFF_REQ",
    "MHB_CAMERA_EV_POWERED_ON",
    "MHB_CAMERA_EV_CONFIGURED",
    "MHB_CAMERA_EV_DECONFIGURED",
    "MHB_CAMERA_EV_WAIT_OVER",
};

static inline char* mhb_camera_sm_event_str(int event) {
    if (event < 0 || event >= MHB_CAMERA_EV_MAX) return "FAILED_EVENT";
    return mhb_camera_sm_events_str[event];
}

static inline char* mhb_camera_sm_state_str(int state) {
    if (state < 0 || state >= MHB_CAMERA_STATE_MAX) return "INVALID_STATE";
    return mhb_camera_sm_states_str[state];
}
#endif


typedef mhb_camera_sm_event_t (*mhb_camera_command_func)(void);

mhb_camera_sm_state_t mhb_camera_sm_get_state(void);
int mhb_camera_sm_init(void);
int mhb_camera_sm_execute(mhb_camera_sm_event_t event);

mhb_camera_sm_event_t mhb_camera_power_on(void);
mhb_camera_sm_event_t mhb_camera_power_off(void);
mhb_camera_sm_event_t mhb_camera_stream_on(void);
mhb_camera_sm_event_t mhb_camera_stream_off(void);
mhb_camera_sm_event_t mhb_camera_lens_retract(void);
mhb_camera_sm_event_t mhb_camera_lens_extend(void);
void mhb_camera_process_ctrl_cache(int dump);

#endif
