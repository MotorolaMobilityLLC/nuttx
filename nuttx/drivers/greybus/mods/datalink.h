/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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

#ifndef _GREYBUS_MODS_DATALINK_H_
#define _GREYBUS_MODS_DATALINK_H_

/*
 * Maximum total size (in bytes) of Mods message that can be sent to data
 * link layer.
 */
#define MODS_DL_PAYLOAD_MAX_SZ (2048)

/****************************************************************************
 * Name: MODS_DL_SEND
 *
 * Description:
 *   Send data over physical layer to the base. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   buf - A pointer to the buffer of data to be sent
 *   len - The length of the buffer to send
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 *
 ****************************************************************************/

#define MODS_DL_SEND(d,b,l) ((d)->ops->send(d,b,l))

struct mods_dl_s;

typedef int (*buf_t)(FAR struct mods_dl_s *dev, FAR const void *buf, size_t len);

struct mods_dl_ops_s
{
  buf_t send;
};

struct mods_dl_cb_s
{
  buf_t recv;
};

/*
 * Mods data link private data.  This structure only defines the initial
 * fields of the structure visible to the client.  The specific implementation
 * may add additional, data link specific fields.
 */
struct mods_dl_s
{
  FAR struct mods_dl_ops_s *ops;
};

/****************************************************************************
 * Name: mods_dl_init
 *
 * Description:
 *   Initialize the Mods data link layer.
 *
 * Input Parameter:
 *   cb - callback structure
 *
 * Returned Value:
 *   Valid Mods data link structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct mods_dl_s *mods_dl_init(struct mods_dl_cb_s *cb);

#endif /* _GREYBUS_MODS_DATALINK_H_ */
