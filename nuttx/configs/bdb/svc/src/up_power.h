/*
 * Copyright (c) 2014 Google Inc.
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

/****************************************************************************
 * configs/bdb/svc/src/up_power.h
 * BDB/SVC power support
 *
 * @author: Jean Pihet <jean.pihet@newoldbits.com>
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

void validate_wake_detect(void);
#endif // __CONFIGS_BDB_INCLUDE_UP_POWER_H
