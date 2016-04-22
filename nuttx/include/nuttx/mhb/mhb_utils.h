
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
#ifndef _MHB_UTIL_
# define _MHB_UTIL_
# include <arch/byteorder.h>

#if !defined(TO_STR_HELPER) && !defined(TO_STR)
# define TO_STR_HELPER(x) #x
# define TO_STR(x) TO_STR_HELPER(x)
#endif

/*
 * These can be used on types with an unknown size.  The compiler will optimize
 * them down to the correct macro during compile.
 */
#define TO_LE(x) if (sizeof(x) == 4) \
                     x = cpu_to_le32(x); \
                 else if (sizeof(x) == 2) \
                     x = cpu_to_le16(x); \
                 else \
                     x = x
#define FROM_LE(x) if (sizeof(x) == 4) \
                       x = le32_to_cpu(x); \
                   else if (sizeof(x) == 2) \
                       x = le16_to_cpu(x); \
                   else \
                       x = x
#define TO_BE(x) if (sizeof(x) == 4) \
                     x = cpu_to_be32(x); \
                 else if (sizeof(x) == 2) \
                     x = cpu_to_be16(x); \
                 else \
                     x = x
#define FROM_BE(x) if (sizeof(x) == 4) \
                       x = be32_to_cpu(x); \
                   else if (sizeof(x) == 2) \
                       x = be16_to_cpu(x); \
                   else \
                       x = x
#endif
