#ifndef __ARCH_ARM_TSB_CHIP_H
#define __ARCH_ARM_TSB_CHIP_H

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

#define CM3UP_BASE      0xE000E000
#define CM3UP_SIZE      0x1000


#endif /* __ARCH_ARM_TSB_CHIP_H */
