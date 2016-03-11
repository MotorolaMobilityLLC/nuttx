/*
 * Copyright (c) 2014-2015 Google Inc.
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

#ifndef  _BYTEORDER_H_
#define  _BYTEORDER_H_

#ifdef CONFIG_ENDIAN_BIG
#error "big-endian unsupported"
#endif

#if defined(CONFIG_ARCH_CORTEXM0) || defined(CONFIG_ARCH_CORTEXM1) || \
    defined(CONFIG_ARCH_CORTEXM3) || defined(CONFIG_ARCH_CORTEXM4)
#include <arch/armv7-m/byteorder.h>
#endif

#define be32_to_cpu(v) __swap32(v)
#define cpu_to_be32(v) __swap32(v)
#define be16_to_cpu(v) __swap16(v)
#define cpu_to_be16(v) __swap16(v)
#define le32_to_cpu(v) (v)
#define cpu_to_le32(v) (v)
#define le16_to_cpu(v) (uint16_t)(v)
#define cpu_to_le16(v) (uint16_t)(v)
#define cpu_to_le64(v) (v)

#endif

