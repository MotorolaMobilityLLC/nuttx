/****************************************************************************
 * configs/bdb/svc/src/up_power.h
 * BDB/SVC power support
 *
 * Copyright (C) 2014 Google, Inc.
 *
 ****************************************************************************/
#ifndef __CONFIGS_BDB_INCLUDE_UP_POWER_H
#define __CONFIGS_BDB_INCLUDE_UP_POWER_H

void bdb_apb1_init(void);
void bdb_apb1_enable(void);
void bdb_apb1_disable(void);
void bdb_apb2_init(void);
void bdb_apb2_enable(void);
void bdb_apb2_disable(void);
void bdb_apb3_init(void);
void bdb_apb3_enable(void);
void bdb_apb3_disable(void);
void bdb_gpb1_init(void);
void bdb_gpb1_enable(void);
void bdb_gpb1_disable(void);
void bdb_gpb2_init(void);
void bdb_gpb2_enable(void);
void bdb_gpb2_disable(void);

#endif	// __CONFIGS_BDB_INCLUDE_UP_POWER_H
