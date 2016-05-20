/****************************************************************************
 * arch/arm/src/chip/tsb_watchdog.c
 *
 *   Copyright (C) 2016 Motorola Mobility. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/watchdog.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "tsb_tmr.h"
#include "tsb_scm.h"
#include "tsb_watchdog.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_TSB_WATCHDOG)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
// milliseconds
#define WDG_MAXTIMEOUT (1000 * 60)

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog
 * driver.  NOTE: that only lldbg types are used so that the output is
 * immediately available.
 */

#ifdef CONFIG_DEBUG_WATCHDOG
#  define wddbg    lldbg
#  define wdvdbg   llvdbg
#else
#  define wddbg(x...)
#  define wdvdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct tsb_lowerhalf_s
{
  const struct watchdog_ops_s  *ops;  /* Lower half operations */
  struct tsb_tmr_ctx *ctx;
  uint32_t timeout;   /* The (actual) selected timeout */
  uint32_t lastreset; /* The last reset time */
  bool     started;   /* true: The watchdog timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

/* "Lower half" driver methods **********************************************/

static int      tsb_start(struct watchdog_lowerhalf_s *lower);
static int      tsb_stop(struct watchdog_lowerhalf_s *lower);
static int      tsb_keepalive(struct watchdog_lowerhalf_s *lower);
static int      tsb_getstatus(struct watchdog_lowerhalf_s *lower,
                  struct watchdog_status_s *status);
static int      tsb_settimeout(struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = tsb_start,
  .stop       = tsb_stop,
  .keepalive  = tsb_keepalive,
  .getstatus  = tsb_getstatus,
  .settimeout = tsb_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct tsb_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: tsb_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tsb_start(struct watchdog_lowerhalf_s *lower)
{
  struct tsb_lowerhalf_s *priv = (struct tsb_lowerhalf_s *)lower;
  irqstate_t flags;

  wdvdbg("Entry: started=%d\n");
  DEBUGASSERT(priv);

  /* Have we already been started? */

  if (!priv->started)
    {
      flags           = irqsave();
      priv->lastreset = clock_systimer();
      tsb_tmr_start(priv->ctx);
      priv->started   = true;
      irqrestore(flags);
    }
  return OK;
}

/****************************************************************************
 * Name: tsb_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tsb_stop(struct watchdog_lowerhalf_s *lower)
{
  struct tsb_lowerhalf_s *priv = (struct tsb_lowerhalf_s *)lower;
  irqstate_t flags;

  wdvdbg("Entry\n");
  DEBUGASSERT(priv);

  flags = irqsave();
  tsb_tmr_cancel(priv->ctx);
  irqrestore(flags);

  return OK;
}

/****************************************************************************
 * Name: tsb_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tsb_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct tsb_lowerhalf_s *priv = (struct tsb_lowerhalf_s *)lower;
  irqstate_t flags;

  wdvdbg("Entry\n");
  DEBUGASSERT(priv);

  flags = irqsave();
  priv->lastreset = clock_systimer();

  tsb_tmr_start(priv->ctx);

  irqrestore(flags);

  return OK;
}

/****************************************************************************
 * Name: tsb_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tsb_getstatus(struct watchdog_lowerhalf_s *lower,
                           struct watchdog_status_s *status)
{
  struct tsb_lowerhalf_s *priv = (struct tsb_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  wdvdbg("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the actual timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */

  ticks   = clock_systimer() - priv->lastreset;
  elapsed = (int32_t)TICK2MSEC(ticks);

  if (elapsed > priv->timeout)
    {
      elapsed = priv->timeout;
    }

  /* Return the approximate time until the watchdog timer expiration */

  status->timeleft = priv->timeout - elapsed;

  wdvdbg("Status     :\n");
  wdvdbg("  flags    : %08x\n", status->flags);
  wdvdbg("  timeout  : %d\n", status->timeout);
  wdvdbg("  timeleft : %d\n", status->timeleft);

  return OK;
}

/****************************************************************************
 * Name: tsb_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in millisecnds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tsb_settimeout(struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct tsb_lowerhalf_s *priv = (struct tsb_lowerhalf_s *)lower;

  wdvdbg("Entry: timeout=%d\n", timeout);
  DEBUGASSERT(priv);

  /* Can this timeout be represented? */
  if (timeout < 1 || timeout > WDG_MAXTIMEOUT)
    {
      lldbg("Cannot represent timeout=%d > %d\n",
            timeout, WDG_MAXTIMEOUT);
      return -ERANGE;
    }

  if (priv->started)
    {
      wddbg("Timer is already started\n");
      return -EBUSY;
    }

  priv->timeout = timeout;

  tsb_tmr_set_time(priv->ctx, timeout * 1000);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tsb_iwdginitialize
 *
 * Description:
 *   Initialize the IWDG watchdog time.  The watchdog timer is initialized and
 *   registers as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void tsb_wdginitialize(const char *devpath)
{
  struct tsb_lowerhalf_s *priv = &g_wdgdev;

  wdvdbg("Entry: devpath=%s\n", devpath);

  /* enable watchdog to trigger warm reset */
  tsb_set_system_conf(TSB_SYS_CONF_WDT_IRQ_EN);

  /* Initialize the driver state structure. */

  priv->ops     = &g_wdgops;
  priv->started = false;

  priv->ctx = tsb_tmr_get(TSB_TMR_TMR0);
  tsb_tmr_configure(priv->ctx, TSB_TMR_MODE_WATCHDOG, NULL /* ISR */);

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet.
   */

  tsb_settimeout((struct watchdog_lowerhalf_s *)priv, CONFIG_TSB_WDG_DEFTIMEOUT);

  (void)watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG_TSB_WATCHDOG */
