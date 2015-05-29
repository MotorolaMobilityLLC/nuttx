/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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

#include <nuttx/config.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/i2c.h>
#include <nuttx/list.h>
#include <nuttx/power/pm.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arch/stm32/slice.h>

#include <apps/greybus-utils/manifest.h>
#include <apps/greybus-utils/svc.h>
#include <apps/greybus-utils/utils.h>

#include "bus.h"
#include "cmd_main.h"
#include "wdog.h"

/*
 * Interface ID of the slice. This is required for the Greybus SVC message to the
 * base. Slices (currently) don't have multiple modules, so always send ID 1.
 */
#define SLICE_IID           "IID-1"

/*
 * Priority to report to PM framework when reporting activity. Report high
 * activity since user-initiated.
 */
#define CONFIG_PM_SLICE_ACTIVITY 10

#ifdef CONFIG_EXAMPLES_NSH
extern int nsh_main(int argc, char *argv[]);
#endif

struct slice_cmd_data
{
  pthread_t base_present_thread;
  sem_t base_present_lock;
  bool base_attached;
};

static struct slice_cmd_data slice_cmd_self;

#ifdef CONFIG_PM
static int slice_cmd_pm_prepare(struct pm_callback_s *cb, enum pm_state_e state)
{
  // Do not allow standby when attached to a base. Need to stay awake to reply
  // to commands
  if ((state >= PM_STANDBY) && slice_cmd_self.base_attached)
      return -EIO;

  return OK;
}

static struct pm_callback_s slice_cmd_pmcb =
{
  // Only need to receive prepare callback
  .prepare = slice_cmd_pm_prepare,
};
#endif

static void *slice_cmd_base_present_worker(void *v)
{
  struct slice_cmd_data *slf = (struct slice_cmd_data *)v;
  bool base_present;

  while (1)
    {
      pm_activity(CONFIG_PM_SLICE_ACTIVITY);
      usleep(100000);

      base_present = slice_is_base_present();

      if (slf->base_attached != base_present)
        {
          logd("base_present=%d\n", base_present);
          slf->base_attached = base_present;

          if (base_present)
            {
              send_svc_handshake();
              send_svc_event(0, SLICE_IID, NULL);
              enable_cports();
            }
          else
            {
              // Ensure that base charging is disabled
              slice_vbus_en_sw(false);

              // Cleanup any messages that were never sent to base
              bus_cleanup();

              // Cleanup manifest
              release_manifest_blob(get_manifest_blob(NULL));
            }
        }

      sem_wait(&slf->base_present_lock);
    }

  return NULL;
}

static int slice_cmd_base_present_isr(int irq, void *context)
{
  sem_post(&slice_cmd_self.base_present_lock);
  return OK;
}

static void slice_cmd_init(void)
{
  sem_init(&slice_cmd_self.base_present_lock, 0, 0);

  if (pthread_create(&slice_cmd_self.base_present_thread, NULL,
                     slice_cmd_base_present_worker, &slice_cmd_self))
    {
      logd("Failed to create pthread for interrupts!\n");
    }
  else if (slice_init(slice_cmd_base_present_isr) != OK)
    {
      logd("Failed to configure GPIOs!\n");
    }
#ifdef CONFIG_PM
  else if (pm_register(&slice_cmd_pmcb) != OK)
    {
      logd("Failed register to power management!\n");
    }
#endif
}

static int slice_cmd_listen(unsigned int cport)
{
  logd("cport=%d\n", cport);
  return 0;
}

struct gb_transport_backend slice_cmd_greybus_backend = {
    .init = slice_cmd_init,
    .listen = slice_cmd_listen,
    .send = bus_greybus_to_base,
};

int slice_cmd_main(int argc, char *argv[])
{
  wdog_init();
  bus_init();
  svc_register(bus_svc_to_base);
  gb_init(&slice_cmd_greybus_backend);

#ifdef CONFIG_EXAMPLES_NSH
  logd("Calling NSH\n");
  return nsh_main(argc, argv);
#else
  return 0;
#endif
}

