#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "up_arch.h"

/* 96 MHz */
#define SYSTICK_RELOAD 960000

int up_timerisr(int irq, uint32_t *regs)
{
   /* Process timer interrupt */
   sched_process_timer();
   return 0;
}

void up_timer_initialize(void) {
    putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

    irq_attach(TSB_IRQ_SYSTICK, (xcpt_t)up_timerisr);

    putreg32(NVIC_SYSTICK_CTRL_CLKSOURCE |
             NVIC_SYSTICK_CTRL_TICKINT   |
             NVIC_SYSTICK_CTRL_ENABLE,
             NVIC_SYSTICK_CTRL);
}
