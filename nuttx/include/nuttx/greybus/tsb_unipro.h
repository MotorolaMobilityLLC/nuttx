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

/**
 * @author: Marti Bolivar
 */

#ifndef _NUTTX_GREYBUS_TSB_UNIPRO_H_
#define _NUTTX_GREYBUS_TSB_UNIPRO_H_

/*
 * TSB attributes
 */
#define TSB_T_REGACCCTRL_TESTONLY  0x007f
#define TSB_DME_DDBL2_A            0x6000
#define TSB_DME_DDBL2_B            0x6001
#define TSB_MAILBOX                0xa000
#define TSB_DME_LAYERENABLEREQ     0xd000
#define TSB_DME_LAYERENABLECNF     0xd000
#define TSB_DME_RESETREQ           0xd010
#define TSB_DME_RESETCNF           0xd010
#define TSB_DME_ENDPOINTRESETREQ   0xd011
#define TSB_DME_ENDPOINTRESETCNF   0xd011
#define TSB_DME_ENDPOINTRESETIND   0xd012
#define TSB_DME_LINKSTARTUPREQ     0xd020
#define TSB_DME_LINKSTARTUPCNF     0xd020
#define TSB_DME_LINKSTARTUPIND     0xd021
#define TSB_DME_LINKLOSTIND        0xd022
#define TSB_DME_HIBERNATEENTERREQ  0xd030
#define TSB_DME_HIBERNATEENTERCNF  0xd030
#define TSB_DME_HIBERNATEENTERIND  0xd031
#define TSB_DME_HIBERNATEEXITREQ   0xd032
#define TSB_DME_HIBERNATEEXITCNF   0xd032
#define TSB_DME_HIBERNATEEXITIND   0xd033
#define TSB_DME_POWERMODEIND       0xd040
#define TSB_DME_TESTMODEREQ        0xd050
#define TSB_DME_TESTMODECNF        0xd050
#define TSB_DME_TESTMODEIND        0xd051
#define TSB_DME_ERRORPHYIND        0xd060
#define TSB_DME_ERRORPAIND         0xd061
#define TSB_DME_ERRORDIND          0xd062
#define TSB_DME_ERRORNIND          0xd063
#define TSB_DME_ERRORTIND          0xd064
#define TSB_INTERRUPTENABLE        0xd080
#define TSB_INTERRUPTSTATUS        0xd081
#define TSB_L2STATUS               0xd082
#define TSB_POWERSTATE             0xd083
#define TSB_TXBURSTCLOSUREDELAY    0xd084
#define TSB_MPHYCFGUPDT            0xd085
#define TSB_ADJUSTTRAILINGCLOCKS   0xd086
#define TSB_SUPPRESSRREQ           0xd087
#define TSB_L2TIMEOUT              0xd088
#define TSB_MAXSEGMENTCONFIG       0xd089
#define TSB_TBD                    0xd090
#define TSB_RBD                    0xd091
#define TSB_DEBUGTXBYTECOUNT       0xd092
#define TSB_DEBUGRXBYTECOUNT       0xd093
#define TSB_DEBUGINVALIDBYTEENABLE 0xd094
#define TSB_DEBUGLINKSTARTUP       0xd095
#define TSB_DEBUGPWRCHANGE         0xd096
#define TSB_DEBUGSTATES            0xd097
#define TSB_DEBUGCOUNTER0          0xd098
#define TSB_DEBUGCOUNTER1          0xd099
#define TSB_DEBUGCOUNTER0MASK      0xd09a
#define TSB_DEBUGCOUNTER1MASK      0xd09b
#define TSB_DEBUGCOUNTERCONTROL    0xd09c
#define TSB_DEBUGCOUNTEROVERFLOW   0xd09d
#define TSB_DEBUGOMC               0xd09e
#define TSB_DEBUGCOUNTERBMASK      0xd09f
#define TSB_DEBUGSAVECONFIGTIME    0xd0a0
#define TSB_DEBUGCLOCKENABLE       0xd0a1
#define TSB_DEEPSTALLCFG           0xd0a2
#define TSB_DEEPSTALLSTATUS        0xd0a3

#endif
