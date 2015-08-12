/*
 * Copyright (c) 2015 Google Inc.
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

#define DBG_COMP DBG_SVC

#include <errno.h>
#include <string.h>

#include <nuttx/config.h>
#include <nuttx/greybus/greybus.h>

#include "svc.h"
#include "up_debug.h"
#include "tsb_switch.h"
#include "gb_svc.h"

/*
 * FIXME: use the hardcoded endo id used in the kernel for now
 */
#define GB_ENDO_ID         0x4755

static unsigned int g_svc_cport;

/*
 * SVC Protocol Requests
 */
int gb_svc_protocol_version(void) {
    struct gb_operation *op_req;
    struct gb_operation *op_resp;
    struct gb_svc_protocol_version_request *version_request;
    struct gb_svc_protocol_version_response *version_response;

    op_req = gb_operation_create(g_svc_cport, GB_SVC_TYPE_PROTOCOL_VERSION,
                                 sizeof(*version_request));
    if (!op_req) {
        return -ENOMEM;
    }
    version_request = gb_operation_get_request_payload(op_req);
    version_request->major = GB_SVC_VERSION_MAJOR;
    version_request->minor = GB_SVC_VERSION_MINOR;

    gb_operation_send_request_sync(op_req);

    op_resp = gb_operation_get_response_op(op_req);
    if (!op_resp) {
        return GB_OP_PROTOCOL_BAD;
    }
    version_response = gb_operation_get_request_payload(op_resp);

    if (version_response->major > GB_SVC_VERSION_MAJOR) {
        dbg_error("unsupported major version: %u\n", version_response->major);
        return -EPROTO;
    }
    dbg_info("SVC Protocol version_major = %u version_minor = %u\n",
             version_response->major, version_response->minor);

    gb_operation_destroy(op_req);

    return 0;
}

int gb_svc_hello(uint8_t ap_intf_id) {
    struct gb_operation *op_req;
    struct gb_svc_hello_request *req;

    op_req = gb_operation_create(g_svc_cport, GB_SVC_TYPE_HELLO,
                                 sizeof(*req));
    if (!op_req) {
        return -EPROTO;
    }

    req = gb_operation_get_request_payload(op_req);
    req->endo_id = GB_ENDO_ID;
    req->interface_id = ap_intf_id;

    gb_operation_send_request_sync(op_req);
    gb_operation_destroy(op_req);

    return 0;
}

int gb_svc_intf_hotplug(uint32_t intf_id, uint32_t unipro_mfg_id,
                        uint32_t unipro_prod_id, uint32_t ara_vend_id,
                        uint32_t ara_prod_id) {
    struct gb_operation *op_req;
    struct gb_svc_intf_hotplug_request *req;

    op_req = gb_operation_create(g_svc_cport, GB_SVC_TYPE_INTF_HOTPLUG, sizeof(*req));
    if (!op_req) {
        return -ENOMEM;
    }

    req = gb_operation_get_request_payload(op_req);
    req->intf_id = intf_id;
    req->data.unipro_mfg_id = unipro_mfg_id;
    req->data.unipro_prod_id = unipro_prod_id;
    req->data.ara_vend_id = ara_vend_id;
    req->data.ara_prod_id = ara_prod_id;

    gb_operation_send_request_sync(op_req);
    gb_operation_destroy(op_req);

    return 0;
}


/*
 * SVC Protocol Request handlers
 */
static uint8_t gb_svc_intf_device_id(struct gb_operation *op) {
    struct gb_svc_intf_device_id_request *req;
    u8 intf_id;
    u8 dev_id;
    int rc;

    req = gb_operation_get_request_payload(op);
    intf_id = req->intf_id;
    dev_id  = req->device_id;

    rc = svc_intf_device_id(intf_id, dev_id);

    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_connection_create(struct gb_operation *op) {
    struct gb_svc_conn_create_request *req;
    int rc;

    req = gb_operation_get_request_payload(op);
    rc = svc_connection_create(req->intf1_id, req->cport1_id,
                               req->intf2_id, req->cport2_id,
                               req->tc, req->flags);

    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_route_create(struct gb_operation *op) {
    struct gb_svc_route_create_request *req;
    int rc;

    req = gb_operation_get_request_payload(op);
    rc = svc_route_create(req->intf1_id, req->dev1_id,
                          req->intf2_id, req->dev2_id);

    return gb_errno_to_op_result(rc);
}

static struct gb_operation_handler gb_svc_handlers[] = {
    GB_HANDLER(GB_SVC_TYPE_INTF_DEVICE_ID, gb_svc_intf_device_id),
    GB_HANDLER(GB_SVC_TYPE_CONN_CREATE, gb_svc_connection_create),
    GB_HANDLER(GB_SVC_TYPE_ROUTE_CREATE, gb_svc_route_create),
};

struct gb_driver svc_driver = {
    .op_handlers = (struct gb_operation_handler*) gb_svc_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_svc_handlers),
};

void gb_svc_register(int cport) {
    g_svc_cport = cport;
    gb_register_driver(cport, &svc_driver);
}
