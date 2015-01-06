/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#ifndef _GREYBUS_UTILS_SVC_H_
#define _GREYBUS_UTILS_SVC_H_

void svc_register(int (*handler)(void *data, size_t size));
void send_svc_handshake(void);
void send_svc_event(int type, char *name, void *priv);
int svc_handle(void *payload, int size);

#endif

