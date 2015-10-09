/*
 * Copyright (c) 2014-2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

#ifndef __ARCH_ARM_INCLUDE_TSB_IRQ_H
#define __ARCH_ARM_INCLUDE_TSB_IRQ_H

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#include <arch/tsb/chip.h>

#define NR_IRQS             (16+ARMV7M_PERIPHERAL_INTERRUPTS)

#define TSB_IRQ_NMI         (2)
#define TSB_IRQ_HARDFAULT   (3)
#define TSB_IRQ_MEMFAULT    (4)
#define TSB_IRQ_BUSFAULT    (5)
#define TSB_IRQ_USAGEFAULT  (6)
#define TSB_IRQ_SVCALL      (11)
#define TSB_IRQ_DBGMONITOR  (12)
#define TSB_IRQ_PENDSV      (14)
#define TSB_IRQ_SYSTICK     (15)

#define TSB_IRQ_EXT_BASE    (16)

#define TSB_IRQ_HSIC                       (0 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_RESERVED0                  (1 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_RESERVED1                  (2 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_I2SOERR                    (3 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_I2SO                       (4 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_I2SIERR                    (5 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_I2SI                       (6 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GPIO                       (7 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_SPI                        (8 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UART                       (9 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_I2C                       (10 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_PWM                       (11 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_TMR0                      (12 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_TMR1                      (13 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_TMR2                      (14 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_TMR3                      (15 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_TMR4                      (16 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_RX_LNGPKT           (17 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_RX_SHTPKT           (18 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_RX_DSILPTX          (19 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_RX_DSIRXTRIG        (20 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_RX_TXERR            (21 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_RX_RXERR            (22 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_RX_LPRXSTATE        (23 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_TX_APPSIDEERR       (24 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_TX_CMD              (25 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_TX_DSIRXERR         (26 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_TX_DSIRXTRIG        (27 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_TX_DSIPRTO          (28 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_TX_DSIRXSTATE       (29 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_TX_INIT             (30 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_AL_RX_FIFO_OVF      (31 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI0_AL_TX               (32 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_RX_LNGPKT           (33 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_RX_SHTPKT           (34 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_RX_DSILPTX          (35 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_RX_DSIRXTRIG        (36 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_RX_TXERR            (37 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_RX_RXERR            (38 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_RX_LPRXSTATE        (39 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_TX_APPSIDEERR       (40 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_TX_CMD              (41 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_TX_DSIRXERR         (42 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_TX_DSIRXTRIG        (43 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_TX_DSIPRTO          (44 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_TX_DSIRXSTATE       (45 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_TX_INIT             (46 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_AL_RX_FIFO_OVF      (47 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_CDSI1_AL_TX               (48 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UHSWKUP                   (49 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UHS                       (50 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_MBOX00                    (51 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_MBOX01                    (52 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_MBOX02                    (53 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_MBOX03                    (54 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_MBOX04                    (55 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_MBOX05                    (56 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_MBOX06                    (57 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_MBOX07                    (58 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_MBOX_SNAPSHOT             (59 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMACABORT                (60 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC00                   (61 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC01                   (62 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC02                   (63 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC03                   (64 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC04                   (65 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC05                   (66 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC06                   (67 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC07                   (68 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC08                   (69 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC09                   (70 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC10                   (71 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC11                   (72 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC12                   (73 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC13                   (74 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC14                   (75 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC15                   (76 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC16                   (77 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC17                   (78 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC18                   (79 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC19                   (80 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC20                   (81 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC21                   (82 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC22                   (83 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC23                   (84 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC24                   (85 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC25                   (86 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC26                   (87 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC27                   (88 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC28                   (89 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC29                   (90 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC30                   (91 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_GDMAC31                   (92 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO                    (93 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM00           (94 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM01           (95 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM02           (96 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM03           (97 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM04           (98 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM05           (99 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM06           (100 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM07           (101 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM08           (102 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM09           (103 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM10           (104 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM11           (105 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM12           (106 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM13           (107 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM14           (108 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM15           (109 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM16           (110 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM17           (111 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM18           (112 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM19           (113 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM20           (114 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM21           (115 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM22           (116 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM23           (117 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM24           (118 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM25           (119 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM26           (120 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM27           (121 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM28           (122 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM29           (123 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM30           (124 + TSB_IRQ_EXT_BASE)
#define TSB_IRQ_UNIPRO_RX_EOM31           (125 + TSB_IRQ_EXT_BASE)

void tsb_dumpnvic(void);
void tsb_irq_clear_pending(int);

FAR const char* tsb_irq_name(int irq);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_TSB_IRQ_H */

