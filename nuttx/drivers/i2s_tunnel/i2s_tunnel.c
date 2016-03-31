/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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
 *
 * This code moves data from the APBA I2S in slave mode to the APBE I2S in
 * master mode.  Essentially it tunnels I2S data from the ap to the mod.
 */

#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/i2s_tunnel/i2s_tunnel.h>
#include <nuttx/i2s_tunnel/i2s_unipro.h>
#include <nuttx/wqueue.h>
#include <apps/ice/ipc.h>
#include <arch/bitops.h>

#define I2S_TUNNEL_IPC_TIMEOUT_MS 100

/*
 * If in round trip loopback mode, both the APBA and APBE operate in master
 * mode mode.  If in normal operation the APBA and APBE must be in opposite modes.
 */
#if defined(CONFIG_I2S_TUNNEL_ROUNDTRIP_LOOPBACK)
# define APBA_MASTER(master) (master)
#else
# define APBA_MASTER(master) (!master)
#endif

typedef enum
{
    I2S_TUNNEL_IPC_MSG_ENABLE,
    I2S_TUNNEL_IPC_MSG_CONFIG,
    I2S_TUNNEL_IPC_MSG_ARM,
    I2S_TUNNEL_IPC_MSG_INFO,
    I2S_TUNNEL_IPC_MSG_END
} I2S_TUNNEL_IPC_MSG_T;

struct i2s_tunnel_ipc_msg_s
{
    I2S_TUNNEL_IPC_MSG_T type;
    union i2s_tunnel_ipc_msg_data_u
    {
        bool enable;
        bool arm;
        struct i2s_tunnel_ipc_msg_config_s
        {
            unsigned int sample_rate;
            uint8_t sample_size_bits;
            I2S_TUNNEL_I2S_MODE_T mode;
            I2S_TUNNEL_I2S_EDGE_T edge;
            bool master;
        } config;
    } data;
};

struct i2s_tunnel_ipc_rsp_s
{
    I2S_TUNNEL_IPC_MSG_T type;
    int ret;
    union i2s_tunnel_ips_rep_u
    {
        struct i2s_tunnel_info_s info;
    } data;
};

static struct i2s_tunnel_ipc_rsp_s g_i2s_tunnel_rsp;

#if defined(CONFIG_UNIPRO_P2P_APBE)
struct i2s_tunnel_wq_s
{
    struct work_s wq;
    bool cmd_done;
    struct i2s_tunnel_ipc_msg_s msg;
    struct i2s_tunnel_ipc_rsp_s *rsp;
    int ret;
};

static struct i2s_tunnel_wq_s g_i2s_tunnel_wq;

static void i2s_tunnel_wq_send_msg(void *arg)
{
    struct i2s_tunnel_wq_s *wq_p = (struct i2s_tunnel_wq_s *)arg;
    uint32_t rsp_sz;

    if (wq_p != NULL)
    {
        wq_p->ret = ipc_request_sync(IPC_APP_ID_I2S,
                                     (void *)&wq_p->msg,
                                     sizeof(wq_p->msg),
                                     (void **)&wq_p->rsp,
                                     &rsp_sz,
                                     I2S_TUNNEL_IPC_TIMEOUT_MS);
        lldbg("Packet size: %u\n", wq_p->rsp->data.info.i2s_tunnel_packet_size);
        if ((wq_p->ret != OK) || (rsp_sz != sizeof(*wq_p->rsp)))
        {
            lldbg("Error: ipc_request_sync failed with return: %d and size: %d\n",
                  wq_p->ret, rsp_sz);
        }
        wq_p->cmd_done = true;
    }
}

static int i2s_tunnel_wq_wait_for_complete(void)
{
    size_t failsafe_ms = 10000;

    /*
     * Wait for the work queue to complete.  work_available() is not used
     * for this since in some cases it will return true before the processing
     * has completed.
     */
    while ((g_i2s_tunnel_wq.cmd_done == false) && (--failsafe_ms > 0))
    {
        usleep(1000);
    }
    if (failsafe_ms > 0)
    {
        lldbg("Packet size: %u\n", g_i2s_tunnel_wq.rsp->data.info.i2s_tunnel_packet_size);
        memcpy(&g_i2s_tunnel_rsp, g_i2s_tunnel_wq.rsp, sizeof(g_i2s_tunnel_rsp));
        release_response(g_i2s_tunnel_wq.rsp);
        g_i2s_tunnel_wq.rsp = NULL;
        return g_i2s_tunnel_wq.ret;
    }
    lldbg("Warning: Timed out waiting for workqueue.");
    return -ETIMEDOUT;
}


static int i2s_tunnel_send_msg_bool(I2S_TUNNEL_IPC_MSG_T type, bool val)
{
    g_i2s_tunnel_wq.cmd_done = false;
    g_i2s_tunnel_wq.msg.type = type;
    g_i2s_tunnel_wq.msg.data.enable = val;
    return work_queue(HPWORK, &g_i2s_tunnel_wq.wq, i2s_tunnel_wq_send_msg,
                      (void *)&g_i2s_tunnel_wq, 0);
}

static int i2s_tunnel_send_msg_config(unsigned int sample_rate,
                                      uint8_t sample_size_bits,
                                      I2S_TUNNEL_I2S_MODE_T mode,
                                      I2S_TUNNEL_I2S_EDGE_T edge,
                                      bool master)
{
    g_i2s_tunnel_wq.cmd_done = false;
    g_i2s_tunnel_wq.msg.type = I2S_TUNNEL_IPC_MSG_CONFIG;
    g_i2s_tunnel_wq.msg.data.config.sample_rate = sample_rate;
    g_i2s_tunnel_wq.msg.data.config.sample_size_bits = sample_size_bits;
    g_i2s_tunnel_wq.msg.data.config.mode = mode;
    g_i2s_tunnel_wq.msg.data.config.edge = edge;
    g_i2s_tunnel_wq.msg.data.config.master = master;
    return work_queue(HPWORK, &g_i2s_tunnel_wq.wq, i2s_tunnel_wq_send_msg,
                      (void *)&g_i2s_tunnel_wq, 0);
}

static int i2s_tunnel_send_msg_info(void)
{
    g_i2s_tunnel_wq.cmd_done = false;
    g_i2s_tunnel_wq.msg.type = I2S_TUNNEL_IPC_MSG_INFO;
    return work_queue(HPWORK, &g_i2s_tunnel_wq.wq, i2s_tunnel_wq_send_msg,
                      (void *)&g_i2s_tunnel_wq, 0);
}


int i2s_tunnel_enable(bool enable)
{
    int error;

    error = i2s_tunnel_send_msg_bool(I2S_TUNNEL_IPC_MSG_ENABLE, enable);
    if (!error)
    {
        error = i2s_unipro_tunnel_enable(enable);
        (void)i2s_tunnel_wq_wait_for_complete();
    }
    return error;
}

int i2s_tunnel_i2s_config(
    unsigned int sample_rate,
    uint8_t sample_size_bits,
    I2S_TUNNEL_I2S_MODE_T mode,
    I2S_TUNNEL_I2S_EDGE_T edge,
    bool master)
{
    int error;

    error = i2s_tunnel_send_msg_config(sample_rate,
                                       sample_size_bits,
                                       mode,
                                       edge,
                                       APBA_MASTER(master));
    if (!error)
    {
        error = i2s_unipro_tunnel_i2s_config(sample_rate,
                                             sample_size_bits,
                                             mode,
                                             edge,
                                             master);
        (void)i2s_tunnel_wq_wait_for_complete();
    }
    return error;
}

int i2s_tunnel_arm(bool enable)
{
    int error;

    error = i2s_tunnel_send_msg_bool(I2S_TUNNEL_IPC_MSG_ARM, enable);
    if (!error)
    {
        error = i2s_unipro_tunnel_arm(enable);
        (void)i2s_tunnel_wq_wait_for_complete();
    }
    return error;
}

void i2s_tunnel_start(bool start)
{
    /*
     * Start only needs to be run on the APBE, a message will be sent to the
     * APBA internally.
     */
    i2s_unipro_tunnel_start(true);
}

int i2s_tunnel_get_info(struct i2s_tunnel_info_s *local, struct i2s_tunnel_info_s *remote)
{
    int error = -EINVAL;

    if (local != NULL)
    {
        error = i2s_unipro_tunnel_info(local);
    }
    if ((remote != NULL) && (error == OK))
    {
        error = i2s_tunnel_send_msg_info();
        if (error == OK)
        {
            (void)i2s_tunnel_wq_wait_for_complete();
            error = g_i2s_tunnel_rsp.ret;
            lldbg("Received %d from APAB (packet size: %u)\n",
                  error, g_i2s_tunnel_rsp.data.info.i2s_tunnel_packet_size);
            memcpy(remote, &g_i2s_tunnel_rsp.data.info, sizeof(*remote));
        }
    }
    return error;
}

#else

static uint32_t i2s_tunnel_ipc_handler(void *in,
                                       uint32_t in_len,
                                       void **out,
                                       uint32_t *out_len)
{
    int  retval = -EINVAL;
    struct i2s_tunnel_ipc_msg_s *i2s_tunnel_msg = (struct i2s_tunnel_ipc_msg_s *)in;

    /*
     * At this time all messages are the same length.  This will have to change if
     * any of the messages become longer than 10's of bytes.
     */
    lldbg("Received message.\n");
    if ((in != NULL) && (in_len == sizeof(*i2s_tunnel_msg)))
    {
        i2s_tunnel_msg = (struct i2s_tunnel_ipc_msg_s *)in;
        g_i2s_tunnel_rsp.type = i2s_tunnel_msg->type;
        g_i2s_tunnel_rsp.ret = -EINVAL;
        *out = (void *)&g_i2s_tunnel_rsp;
        *out_len = sizeof(g_i2s_tunnel_rsp);
        retval = 0;

        lldbg("Msg: %d\n", i2s_tunnel_msg->type);
        switch (i2s_tunnel_msg->type)
        {
            case I2S_TUNNEL_IPC_MSG_ENABLE:
                g_i2s_tunnel_rsp.ret = i2s_unipro_tunnel_enable(i2s_tunnel_msg->data.enable);
                break;

            case I2S_TUNNEL_IPC_MSG_CONFIG:
                g_i2s_tunnel_rsp.ret =
                    i2s_unipro_tunnel_i2s_config(i2s_tunnel_msg->data.config.sample_rate,
                                                 i2s_tunnel_msg->data.config.sample_size_bits,
                                                 i2s_tunnel_msg->data.config.mode,
                                                 i2s_tunnel_msg->data.config.edge,
                                                 i2s_tunnel_msg->data.config.master);
                break;

            case I2S_TUNNEL_IPC_MSG_ARM:
                g_i2s_tunnel_rsp.ret = i2s_unipro_tunnel_arm(i2s_tunnel_msg->data.arm);
                break;

            case I2S_TUNNEL_IPC_MSG_INFO:
                g_i2s_tunnel_rsp.ret = i2s_unipro_tunnel_info(&g_i2s_tunnel_rsp.data.info);
                lldbg("Received packet size of: %u\n",
                      g_i2s_tunnel_rsp.data.info.i2s_tunnel_packet_size);
                break;

            default:
                retval = -EINVAL;
                break;
        }
    }
    return (uint32_t)retval;
}

static int i2s_tunnel_ipc_svc_init(void)
{
    return register_ipc_handler(IPC_APP_ID_I2S, i2s_tunnel_ipc_handler, NULL);
}

#endif

int i2s_tunnel_init(void)
{
#if defined(CONFIG_UNIPRO_P2P_APBE)
    memset(&g_i2s_tunnel_wq, 0, sizeof(g_i2s_tunnel_wq));
#else
    (void)i2s_tunnel_ipc_svc_init();
#endif
    return i2s_unipro_tunnel_init();
}
