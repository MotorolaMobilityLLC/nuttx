/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#ifndef CAMERA_EXT_DBG_H
#define CAMERA_EXT_DBG_H

#include <stdio.h>

#ifdef CONFIG_NSH_CONSOLE
    #ifdef CONFIG_DEBUG_TIMESTAMP
    #define DEBUG_PRINT(fmt, ...) \
        do {                                                             \
            struct timespec __time;                                      \
            up_rtc_gettime(&__time);                                     \
            lowsyslog("[%u.%03u] %s: " fmt, __time.tv_sec,               \
                   (__time.tv_nsec / 1000000), __func__, ##__VA_ARGS__); \
            printf("[%u.%03u] %s: " fmt, __time.tv_sec,                  \
                   (__time.tv_nsec / 1000000), __func__, ##__VA_ARGS__); \
        } while (0)
    #else
    #define DEBUG_PRINT(fmt, ...) \
        do {                                               \
            printf("%s: " fmt, __func__, ##__VA_ARGS__);   \
            lowsyslog(EXTRA_FMT fmt EXTRA_ARG, ##__VA_ARGS__);           \
        } while (0)
    #endif
#else
    #define DEBUG_PRINT(fmt, ...) \
        lowsyslog(EXTRA_FMT fmt EXTRA_ARG, ##__VA_ARGS__)
#endif

#if (defined(CONFIG_DEBUG) && defined(DEBUG))
    #define CAM_DBG(fmt, ...) DEBUG_PRINT(fmt, ##__VA_ARGS__)
    #ifdef DEBUG_CTRL
        #define CTRL_DBG(fmt, ...) DEBUG_PRINT(fmt, ##__VA_ARGS__)
    #else
        #define CTRL_DBG(x...)
    #endif
#else
    #define CAM_DBG(x...)
    #define CTRL_DBG(x...)
#endif

#define CAM_ERR(fmt, ...)   DEBUG_PRINT(fmt, ##__VA_ARGS__)
#define CAM_INFO(fmt, ...)   DEBUG_PRINT(fmt, ##__VA_ARGS__)

#endif
