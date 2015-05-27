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
