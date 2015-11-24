/*
 * Copyright (c) 2004, 2005 Topspin Communications.  All rights reserved.
 * Copyright (c) 2005, 2006, 2007 Cisco Systems.  All rights reserved.
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
/*
 * Copyright (c) 2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

/**
 * @author: Perry Hung
 */

#ifndef  _UTIL_H_
#define  _UTIL_H_

#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef BIT
    #define BIT(n)  (1UL << (n))
#endif

#ifndef ONE_BIT_IS_SET
    #define ONE_BIT_IS_SET(x)   ((x) && !((x) & ((x) - 1)))
#endif

/* Wouldn't work for types bigger than 4 bytes */
#ifndef ALIGN
    #define _ALIGNBYTES    (sizeof(uint32_t) - 1)
    #define ALIGN(p)       ((typeof(p))(((unsigned)(p) + _ALIGNBYTES) & \
                                        ~_ALIGNBYTES))
#endif

#ifndef MIN
    #define MIN(a, b) ({        \
        typeof(a) _a = (a);     \
        typeof(b) _b = (b);     \
        _a < _b ? _a : _b;      \
    })
#endif

#ifndef MAX
    #define MAX(a, b) ({        \
        typeof(a) _a = (a);     \
        typeof(b) _b = (b);     \
        _a > _b ? _a : _b;      \
    })
#endif

#define CONTAINER_OF(ptr, type, mbr) ({                      \
        const typeof( ((type *)0)->mbr ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,mbr) );})

#endif

/*
 * Use this macro to divide x by d (d must be positive) and round it to
 * closest integer instead of rounding down as usual when performing
 * integer division.
 */
#define DIV_ROUND_CLOSEST(x, d) ({                                      \
    typeof(x) __x = x;                                                  \
    typeof(d) __d = d;                                                  \
    typeof(x + 0) __result;                                             \
    if (((typeof(x))-1) > 0 || ((typeof(d))-1) > 0 || (__x) > 0)        \
        __result = (((__x) + ((__d) / 2)) / (__d));                     \
    else                                                                \
        __result = (((__x) - ((__d) / 2)) / (__d));                     \
    (__result);                                                         \
})
