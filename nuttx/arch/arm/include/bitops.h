
/*
 * Copyright (c) 2016 Motorola Mobility, LLC
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
#ifndef _BITOPS_H_
# define _BITOPS_H_

# if defined(CONFIG_ARCH_CORTEXM0) || defined(CONFIG_ARCH_CORTEXM1) || \
     defined(CONFIG_ARCH_CORTEXM3) || defined(CONFIG_ARCH_CORTEXM4)
#  include <arch/armv6-m/bitops.h>
# elif defined (__GNUC__)
/*
 * Use the GCC built in functions if an architecture has not specified the
 * functions.
 */
# define __clz(value) __builtin_clz(value)
# define __fls(value) (32 - __builtin_clz(value))
# define __ffs(value) __builtin_ffs(value)
# else
#  error "bitops are not defined."
# endif

/**
 * @brief Return the number of leading zeros in a 32 bit value.
 *
 * Returns the number of leading zeros in a 32 bit value.  This starts
 * counting at the MSB, so 0x80000000 returns 0 and 0x1 returns 31.  0
 * returns 32.
 *
 * @param value  The number to count the leading zeros on.
 */

# define clz(value) __clz(value)

/**
 * @brief Return the last bit set in a 32 bit number.
 *
 * Returns the bit number of the most significant bit set in a 32 bit
 * number.  For example 0xFFFFFFFF returns 32 and 0x00000001 returns 1.
 * 0 returns 0.
 *
 * @param value The number to find the most significant bit set in.
 */
# define fls(value) __fls(value)

/**
 * @brief Return the first bit set in a 32 bit number.
 *
 * Returns the bit number of the least significant bit set in a 32 bit
 * number.  For example 0xFFFFFFFF returns 0 and 0x8000000 returns 31.
 * 0 returns 32.
 *
 * @param value The number to find the most significant bit set in.
 */
# define ffs(value) __fls(value)
#endif
