/*
 * Copyright (c) 2014-2015 Google, Inc.
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

#include <nuttx/config.h>

#include <arch/irq.h>

#include "chip.h"
#include "exc_return.h"

#define IDLE_STACK		(_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)
#define HEAP_BASE		(_ebss+CONFIG_IDLETHREAD_STACKSIZE)

.syntax		unified
.file		"tsb_vectors.S"

.text
.thumb
.thumb_func
.align	2
.globl	Reset_Handler
.type	Reset_Handler, %function
Reset_Handler:
    ldr sp, =IDLE_STACK
    b tsb_start


.section	.rodata, "a"

/*
 * Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the
 * end of the BSS regsion (see ld.script). The idle task stack starts at the
 * end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is
 * the thread that the system boots on and, eventually, becomes the idle, do
 * nothing task that runs only when there is nothing else to run.  The heap
 * continues from there until the end of memory.  See g_idle_topstack below.
 */

.globl	g_idle_topstack
.type	g_idle_topstack, object
g_idle_topstack:
    .word	HEAP_BASE
    .size	g_idle_topstack, .-g_idle_topstack

.section .bootstrap.vector
    .word   IDLE_STACK
    .word __start

.end
