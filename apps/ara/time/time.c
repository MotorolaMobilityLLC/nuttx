/**
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

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

int time_main(int argc, char **argv)
{
    struct timespec ts;
    struct timeval tv;
    int rv1, rv2, opt;

    if (argc < 2)
        goto help;

    optind = -1;
    while ((opt = getopt (argc, argv, "unt")) != -1) {
        switch (opt) {
        case 'u':
            gettimeofday(&tv, NULL);
            printf("%u.%ld\n", tv.tv_sec, tv.tv_usec);
            break;
        case 'n':
            clock_gettime(CLOCK_REALTIME, &ts);
            printf("%u.%ld\n", ts.tv_sec, ts.tv_nsec);
            break;
        case 't':
            rv1 = clock_gettime(CLOCK_REALTIME, &ts);
            rv2 = gettimeofday(&tv, NULL);

            printf("clock_gettime(CLOCK_REALTIME, {%u, %ld}) = %d\n",
                   ts.tv_sec, ts.tv_nsec, rv1);
            printf("gettimeofday({%u, %ld}, NULL) = %d\n",
                   tv.tv_sec, tv.tv_usec, rv2);
            break;
        default:
            goto help;
        }
    }

    return EXIT_SUCCESS;

help:
    printf("Print current system time in various formats\n\n"
           "\tusage: time [OPTS]\n"
           "\tOPTS:\n"
           "\t\t-u - display seconds and microseconds\n"
           "\t\t-n - display seconds and nanoseconds\n"
           "\t\t-t - display strace like output for internal funcs\n\n");

    return EXIT_FAILURE;
}
