/*
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

#include "up_switch.h"


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int svc_main(int argc, char *argv[])
#endif
{
	int state = 0;

	if (argc < 2)
		return ERROR;

	state = strtol(argv[1], NULL, 10);

	switch_control(state);

	return 0;
}
