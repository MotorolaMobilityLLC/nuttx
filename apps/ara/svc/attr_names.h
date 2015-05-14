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

#ifndef __SVC_ATTR_NAMES_H_
#define __SVC_ATTR_NAMES_H_

#include <stdint.h>

struct attr_name {
    uint16_t attr;
    const char *name;
};

struct attr_name_group {
    /*
     * This is an array, terminated with an element whose "name"
     * field is null.
     */
    const struct attr_name *attr_names;
    /*
     * A printable name of this group of attributes.
     */
    const char *group_name;
};

extern const struct attr_name_group unipro_l1_attr_group;
extern const struct attr_name_group unipro_l1_5_attr_group;
extern const struct attr_name_group unipro_l2_attr_group;
extern const struct attr_name_group unipro_l3_attr_group;
extern const struct attr_name_group unipro_l4_attr_group;
extern const struct attr_name_group unipro_dme_attr_group;
extern const struct attr_name_group unipro_tsb_attr_group;

const char* attr_get_name(uint16_t attr);

#endif
