/****************************************************************************
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 ****************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>

#include <debug.h>
#include <arch/irq.h>

#include "up_internal.h"
#include "stm32_pm.h"
#include "stm32_rcc.h"
#include "stm32_exti.h"

/****************************************************************************
 * Name: idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void idlepm(void)
{
  static enum pm_state_e oldstate = PM_NORMAL;
  enum pm_state_e newstate;
  irqstate_t flags;
  int ret;

  /* Decide, which power saving level can be obtained */
  newstate = pm_checkstate();

  /* Check for state changes */

  if (newstate != oldstate)
    {
      flags = irqsave();

      /* Force the global state change */

      ret = pm_changestate(newstate);
      if (ret < 0)
        {
          /* The new state change failed, revert to the preceding state */
          (void)pm_changestate(oldstate);

          /* No state change... */
          goto errout;
        }

      llvdbg("%d -> %d\n", oldstate, newstate);

      /* Then perform board-specific, state-dependent logic here */

      switch (newstate)
        {
        case PM_NORMAL:
        case PM_IDLE:
          break;

        case PM_STANDBY:
        case PM_SLEEP:
          {
#ifdef CONFIG_DEBUG
            /* Use STOP 1 mode when debugging so serial console can wake */
            stm32_pmstop(true);
#else
            /* Use STOP 2 mode to achieve lower current drain */
            stm32_pmstop2();
#endif

            /* Resume normal operation */
            stm32_clockenable();
          }
          break;

        default:
          break;
        }

      /* Save the new state */
      oldstate = newstate;

errout:
      irqrestore(flags);
    }
}
#else
#  define idlepm()
#endif

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
  /* Perform IDLE mode power management */
  idlepm();

  /* Sleep until an interrupt occurs to save power. */
  asm("WFI");
}
