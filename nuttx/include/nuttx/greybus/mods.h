/*
 * Copyright (C) 2015-2016 Motorola Mobility, LLC.
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

#ifndef _GREYBUS_MODS_H_
#define _GREYBUS_MODS_H_

#include <nuttx/notifier.h>

/*
 * Base attach logic assumes this enum order. Do not change without changing
 * both!
 */

enum base_attached_e
{
  BASE_DETACHED,
  BASE_ATTACHED,
  BASE_ATTACHED_OFF,
  BASE_INVALID,
};

/****************************************************************************
 * Name: mods_attach_init
 *
 * Description:
 *   Initialize the Mods attach notifications.
 *
 * Input Parameter:
 *   (None)
 *
 * Returned Value:
 *   0 on success or negative errno on failure
 *
 ****************************************************************************/

int mods_attach_init(void);

/****************************************************************************
 * Name: mods_attach_register
 *
 * Description:
 *   Register for Mods attach/detach notifications.
 *
 * Input Parameters:
 *   callback - Function to call on Mods attach/detach events.
 *   arg      - Parameter to pass to callback function.
 *
 * Returned Value:
 *   0 on success or negative errno on failure
 *
 ****************************************************************************/

int mods_attach_register(notifier_fn_t callback, void *arg);

/****************************************************************************
 * Name: mods_network_init
 *
 * Description:
 *   Initialize the Mods network layer.
 *
 * Input Parameter:
 *   (None)
 *
 * Returned Value:
 *   0 on success or negative errno on failure
 *
 ****************************************************************************/

int mods_network_init(void);

#endif /* _GREYBUS_MODS_H_ */
