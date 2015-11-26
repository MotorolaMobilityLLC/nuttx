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

#ifndef _ARA_COMMON_ES2_MPHY_FIXUPS_H_
#define _ARA_COMMON_ES2_MPHY_FIXUPS_H_

#include <stdint.h>

/*
 * This specifies an M-PHY "fixup"; i.e., a value that must be set to
 * a DME attribute while the link is still in PWM-G1, before
 * transitioning to HS link power modes.
 *
 * Use tsb_mphy_r1_fixup_is_magic() to test if a register 1 map fixup
 * must have its value drawn from M-PHY trim values or magic debug
 * registers (switch ports and bridges handle this case differently).
 */
struct tsb_mphy_fixup {
    uint16_t attrid;
    uint16_t select_index;
    uint32_t value;

#define TSB_MPHY_FIXUP_LAST     0x1
#define TSB_MPHY_FIXUP_MAGIC_R1 0x2
    uint32_t flags;
};

/*
 * Use tsb_mphy_fixup_is_last() to test if a fixup is the last one in
 * each of these array.
 */
extern const struct tsb_mphy_fixup tsb_register_1_map_mphy_fixups[];
extern const struct tsb_mphy_fixup tsb_register_2_map_mphy_fixups[];

static inline int tsb_mphy_fixup_is_last(const struct tsb_mphy_fixup *fu) {
    return !!(fu->flags & TSB_MPHY_FIXUP_LAST);
}

static inline int tsb_mphy_r1_fixup_is_magic(const struct tsb_mphy_fixup *fu) {
    return !!(fu->flags & TSB_MPHY_FIXUP_MAGIC_R1);
}

void es2_apply_mphy_fixup(void);

#endif
