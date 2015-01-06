/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#ifndef _GREYBUS_UTILS_MANIFEST_H_
#define _GREYBUS_UTILS_MANIFEST_H_

void foreach_manifest(void (manifest_handler)(unsigned char *manifest_file, int manifest_number));
void enable_cports(void);
void parse_manifest_blob(char *hpe);

#endif

