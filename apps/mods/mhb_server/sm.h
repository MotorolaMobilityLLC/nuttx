/*
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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

#if !defined(app_mods_sm_h)
#define app_mods_sm_h

enum svc_event {
    SVC_EVENT_MASTER_STARTED,
    SVC_EVENT_SLAVE_STARTED,
    SVC_EVENT_TEST_MODE_STARTED,
    SVC_EVENT_MOD_DETECTED,
    SVC_EVENT_UNIPRO_LINK_TIMEOUT,
    SVC_EVENT_MOD_TIMEOUT,
    SVC_EVENT_UNIPRO_LINK_UP,
    SVC_EVENT_UNIPRO_LINK_DOWN,
    SVC_EVENT_GEAR_SHIFT_DONE,
    SVC_EVENT_QUEUE_STATS,
    SVC_EVENT_SEND_STATS,
    SVC_EVENT_CPORTS_DONE,

    SVC_EVENT_MAX,
};

int svc_init(void);
int svc_send_event(enum svc_event event, void *parameter0, void *parameter1, void *parameter2);

#endif

