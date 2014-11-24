/****************************************************************************
 * configs/bdb/svc/src/up_epm.h
 * BDB/SVC EPM support
 *
 * There are control signals for only 2 EPMS: A1, A2.
 *
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 ****************************************************************************/
#ifndef __CONFIGS_BDB_INCLUDE_UP_EPM_H
#define __CONFIGS_BDB_INCLUDE_UP_EPM_H

#define NR_EPMS     2

enum epm_id {
    EPM_A1          = 0,
    EPM_A2,
    EPM_STATE_28V   = 31,
};

int epm_get_28v_state(void);
int epm_activate_28v(uint32_t state);
int epm_get_state(uint32_t epm_nr);
int epm_set_state(uint32_t epm_nr, uint32_t state);
int epm_init(void);

#endif // __CONFIGS_BDB_INCLUDE_UP_EPM_H
