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

#if !defined(app_mods_gearbox_h)
#define app_mods_gearbox_h

#define GEARBOX_HS_MIN (1000)

struct gearbox;

/* Structure to hold the abstract gear requests that clients make to gearbox. */
struct gear_request {
    /* Clients request the maximum bandwith they require in megabits-per-second.
     * This should only include the application-level requirements.  Additional
     * bandwidth required by the UniPro stack will be automatically included.
     *
     * All client bandwidth requests will be added to determine the target gear,
     * even if some clients are not using the maximum at all times.
     *
     * If the bandwidth requests exceed available bandwidth, gearbox will use
     * the maximum available gear and hope for the best.
     */
    /* From APBA to APBE in megabits-per-second */
    uint32_t rx_max_mbps;
    /* From APBE to APBA in megabits-per-second */
    uint32_t tx_max_mbps;

    /* UniPro can automatically switch between active and low-power states.
     * This can be useful for intermittent or bursty traffic (e.g. HSIC).
     * However, it can be unnecessary for continuous traffic (e.g. IS2 or CDSI).
     * Allow the client to optionally request auto-mode.  Auto-mode will only
     * be used if all clients request it.
     */
    /* 0: continuous, 1: burst (auto) */
    uint32_t rx_auto:1;
    /* 0: continuous, 1: burst (auto) */
    uint32_t tx_auto:1;
};

/* System API */
struct gearbox *gearbox_initialize(void);
void gearbox_uninitialize(struct gearbox *gearbox);

/* Client API */
int gearbox_register(struct gearbox *gearbox);
int gearbox_unregister(struct gearbox *gearbox, int id);
int gearbox_request(struct gearbox *gearbox,
        int id, const struct gear_request *request);

/* Events from the UniPro stack. */
int gearbox_link_up(struct gearbox *gearbox);
int gearbox_link_down(struct gearbox *gearbox);
int gearbox_shift_complete(struct gearbox *gearbox, int err);

#endif