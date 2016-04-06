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

#ifndef __ARCH_ARM_TSB_CHIP_H
#define __ARCH_ARM_TSB_CHIP_H

#include <nuttx/config.h>
#include <arch/tsb/chip.h>

/* AP/GP Bridge peripherals memory map */

#define SYSCTL_BASE     0x40000000
#define SYSCTL_SIZE     0x1000

#define I2C_BASE        0x40001000
#define I2C_SIZE        0x100

#define TMR_BASE        0x40002000
#define TMR_SIZE        0x800

#define GPIO_BASE       0x40003000
#define GPIO_SIZE       0x40

#define PWMOD_BASE      0x40004000
#define PWMOD_SIZE      0x400

#define UART_BASE       0x40005000
#define UART_SIZE       0x100

#define MBOX_BASE       0x40006000
#define MBOX_SIZE       0x1000

#define I2SLP_SC_BASE   0x40007000
#define I2SLP_SC_SIZE   0x100

#define I2SLP_SO_BASE   0x40008000
#define I2SLP_SO_SIZE   0x200

#define I2SLP_SI_BASE   0x40009000
#define I2SLP_SI_SIZE   0x200

#define CDSI0_BASE      0x40010000
#define CDSI0_SIZE      0x2000

#define CDSI1_BASE      0x40012000
#define CDSI1_SIZE      0x2000

#define GDMAC_BASE      0x40014000
#define GDMAC_SIZE      0x1000

#define GDMACN_BASE     0x40015000
#define GDMACN_SIZE     0x1000

#define SPI_BASE        0x40018000
#define SPI_SIZE        0x800

#define UHSSD_BASE      0x40019000
#define UHSSD_SIZE      0x100

#define AIO_UNIPRO_BASE 0x40020000
#define AIO_UNIPRO_SIZE 0x2000

#define HSIC_BASE       0x40040000
#define HSIC_SIZE       0x1000

#define ISAA_BASE       0x40084000
#define ISAA_SIZE       0x1000

#define VIDCRYPT_BASE   0x40085000
#define VIDCRYPT_SIZE   0x100

#define CM3UP_BASE      0xE000E000
#define CM3UP_SIZE      0x1000


#endif /* __ARCH_ARM_TSB_CHIP_H */
