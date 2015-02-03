/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 */

#ifndef  _ARA_BOARD_H_
#define  _ARA_BOARD_H_

struct ara_board_info {
    struct interface **interfaces;
    size_t nr_interfaces;

    /* Switch data */
    struct tsb_switch_driver *sw_drv;
    unsigned int sw_1p1;
    unsigned int sw_1p8;
    unsigned int sw_reset;
    unsigned int sw_irq;

    unsigned int svc_irq;
};

struct ara_board_info *board_init(void);
void board_exit(void);

#endif


