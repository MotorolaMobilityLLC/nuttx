/*
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef CONFIG_ARCH_BOARD_ARA_SVC
#include "up_switch.h"
#else
#include "svc.h"
#endif


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

#ifndef CONFIG_ARCH_BOARD_ARA_SVC
    switch_control(state);
#else
    if (state) {
        svc_init();
    } else {
        svc_exit();
    }
#endif

	return 0;
}
