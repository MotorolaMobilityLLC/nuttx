/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>

#include "svc_msg.h"
#include "greybus_manifest.h"

int verbose;
static int state = GBEMU_IDLE;

size_t(*svc_int_write) (void *data, size_t size);

void send_svc_handshake(void)
{
    uint8_t buf[256];
    struct svc_msg *m = (struct svc_msg *)buf;

    m->header.function_id = SVC_FUNCTION_HANDSHAKE;
    m->header.message_type = SVC_MSG_DATA;
    m->header.payload_length = htole16(HS_PAYLOAD_SIZE);
    m->handshake.version_major = 0;
    m->handshake.version_minor = 0;
    m->handshake.handshake_type = SVC_HANDSHAKE_SVC_HELLO;

    svc_int_write(m, HS_MSG_SIZE);
    gb_debug("SVC->AP handshake sent\n");
}

void send_hot_plug(char *hpe, int mid)
{
    struct svc_msg *msg = (struct svc_msg *)hpe;
    struct greybus_manifest_header *mh =
        (struct greybus_manifest_header *)(hpe + HP_BASE_SIZE);

    msg->header.function_id = SVC_FUNCTION_HOTPLUG;
    msg->header.message_type = SVC_MSG_DATA;
    msg->header.payload_length = mh->size + 2;
    msg->hotplug.hotplug_event = SVC_HOTPLUG_EVENT;
    msg->hotplug.module_id = mid;

    /* Write out hotplug message with manifest payload */
    svc_int_write(hpe, HP_BASE_SIZE + mh->size);

    gb_debug("SVC->AP hotplug event (plug) sent\n");
}

void send_hot_unplug(int mid)
{
    struct svc_msg msg;

    msg.header.function_id = SVC_FUNCTION_HOTPLUG;
    msg.header.message_type = SVC_MSG_DATA;
    msg.header.payload_length = 2;
    msg.hotplug.hotplug_event = SVC_HOTUNPLUG_EVENT;
    msg.hotplug.module_id = mid;

    /* Write out hotplug message */
    svc_int_write(&msg, HP_BASE_SIZE);

    gb_debug("SVC->AP hotplug event (unplug) sent\n");
}

void send_link_up(int mid, int iid, int did)
{
    struct svc_msg msg;

    msg.header.function_id = SVC_FUNCTION_UNIPRO_NETWORK_MANAGEMENT;
    msg.header.message_type = SVC_MSG_DATA;
    msg.header.payload_length = htole16(LU_PAYLOAD_SIZE);
    msg.management.management_packet_type = SVC_MANAGEMENT_LINK_UP;
    msg.management.link_up.module_id = mid;
    msg.management.link_up.interface_id = iid;
    msg.management.link_up.device_id = did;

    /* Write out hotplug message */
    svc_int_write(&msg, LU_MSG_SIZE);

    gb_debug("SVC -> AP Link Up (%d:%d:%d) message sent\n", mid, iid, did);
}

void send_ap_id(int mid)
{
    struct svc_msg msg;

    msg.header.function_id = SVC_FUNCTION_UNIPRO_NETWORK_MANAGEMENT;
    msg.header.message_type = SVC_MSG_DATA;
    msg.header.payload_length = htole16(APID_PAYLOAD_SIZE);
    msg.management.management_packet_type = SVC_MANAGEMENT_AP_ID;
    msg.management.ap_id.module_id = mid;
    msg.management.ap_id.device_id = 1;

    /* Write out hotplug message */
    svc_int_write(&msg, APID_MSG_SIZE);

    gb_debug("SVC -> AP ID (MID:%d DID:1) message sent\n", mid);
}

int svc_handle(void *payload, int size)
{
    struct svc_msg *m = (struct svc_msg *)payload;

    switch (m->header.function_id) {
    case SVC_FUNCTION_HANDSHAKE:
        if (HS_VALID(m)) {
            gb_info("AP handshake complete\n");
            state = GBEMU_HS_COMPLETE;
            send_ap_id(0);
        } else
            gb_error("AP handshake invalid");
        break;
    case SVC_FUNCTION_UNIPRO_NETWORK_MANAGEMENT:
        gb_debug("AP -> SVC set route to Device ID %d\n",
                 m->management.set_route.device_id);
        break;
    default:
        gb_error("SVC message ID invalid");
        return -1;
    }
    return size;
}

static int get_module_id(char *fname)
{
    char *mid_str;
    int mid = 0;
    char tmp[256];

    strcpy(tmp, fname);
    mid_str = strtok(tmp, "-");
    if (!strncmp(mid_str, "MID", 3))
        mid = strtol(fname + 4, NULL, 0);

    return mid;
}

void send_svc_event(int type, char *name, void *priv)
{
    char *hpe;

    hpe = get_manifest_blob(priv);
    if (type == 0) {
        if (hpe) {
            parse_manifest_blob(hpe);
            int mid = get_module_id(name);
            if (mid > 0) {
                gb_info("%s module inserted\n", name);
                send_hot_plug(hpe, mid);
                /*
                 * FIXME: hardcoded
                 * interface and device
                 * ID
                 */
                send_link_up(mid, 0, 2);
            } else
                gb_error("invalid module ID, no hotplug plug event sent\n");
        } else
            gb_error("missing manifest blob, no hotplug event sent\n");
    } else if (type == 1) {
        int mid = get_module_id(name);
        if (mid > 0) {
            manifest_release(hpe);
            send_hot_unplug(mid);
            gb_info("%s module removed\n", name);
        } else
            gb_error("invalid module ID, no hotplug unplug event sent\n");
    }
    free(hpe);
}

void svc_register(size_t(*handler) (void *data, size_t size))
{
    svc_int_write = handler;
}
