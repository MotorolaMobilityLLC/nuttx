#include <nuttx/config.h>
#include <stdint.h>
#include "chip.h"
#include "up_arch.h"

#define TSB_SYSCTL_SOFTRESET0           (SYSCTL_BASE + 0x0)
#define TSB_SYSCTL_SOFTRESETRELEASE0    (SYSCTL_BASE + 0x100)
#define TSB_SYSCTL_CLOCKGATING0         (SYSCTL_BASE + 0x200)
#define TSB_SYSCTL_CLOCKENABLE0         (SYSCTL_BASE + 0x300)
#define TSB_SYSCTL_PINSHARE             (SYSCTL_BASE + 0x800)

/* Clock bits for CLOCKGATING/CLOCKENABLE registers */
#define TSB_SYSCTL_CLOCK_SDIOSYSCLK (1 << 9)
#define TSB_SYSCTL_CLOCK_SDIOSDCLK  (1 << 10)
#define TSB_SYSCTL_CLOCK_MBOXCLK    (1 << 12)
#define TSB_SYSCTL_CLOCK_GDMACLK    (1 << 13)
#define TSB_SYSCTL_CLOCK_TMRCLK     (1 << 14)
#define TSB_SYSCTL_CLOCK_WDTCLK     (1 << 15)
#define TSB_SYSCTL_CLOCK_TIMERCLK   (1 << 16)
#define TSB_SYSCTL_CLOCK_PWMODPCLK  (1 << 17)
#define TSB_SYSCTL_CLOCK_PWMODSCLK  (1 << 18)
#define TSB_SYSCTL_CLOCK_I2CPCLK    (1 << 19)
#define TSB_SYSCTL_CLOCK_I2CSCLK    (1 << 20)
#define TSB_SYSCTL_CLOCK_UARTPCLK   (1 << 21)
#define TSB_SYSCTL_CLOCK_UARTSCLK   (1 << 22)
#define TSB_SYSCTL_CLOCK_SPIPCLK    (1 << 23)
#define TSB_SYSCTL_CLOCK_SPISCLK    (1 << 24)
#define TSB_SYSCTL_CLOCK_GPIOCLK    (1 << 25)

#define DEFAULT_PERIPHERAL_CLOCK_BITS \
    (TSB_SYSCTL_CLOCK_SDIOSYSCLK | \
     TSB_SYSCTL_CLOCK_SDIOSDCLK  | \
     TSB_SYSCTL_CLOCK_MBOXCLK    | \
     TSB_SYSCTL_CLOCK_GDMACLK    | \
     TSB_SYSCTL_CLOCK_TMRCLK     | \
     TSB_SYSCTL_CLOCK_WDTCLK     | \
     TSB_SYSCTL_CLOCK_TIMERCLK   | \
     TSB_SYSCTL_CLOCK_PWMODPCLK  | \
     TSB_SYSCTL_CLOCK_PWMODSCLK  | \
     TSB_SYSCTL_CLOCK_I2CPCLK    | \
     TSB_SYSCTL_CLOCK_I2CSCLK    | \
     TSB_SYSCTL_CLOCK_UARTPCLK   | \
     TSB_SYSCTL_CLOCK_UARTSCLK   | \
     TSB_SYSCTL_CLOCK_SPIPCLK    | \
     TSB_SYSCTL_CLOCK_SPISCLK    | \
     TSB_SYSCTL_CLOCK_GPIOCLK)

/* Reset_n bits for SOFTRESET/SOFTRELEASE */
#define TSB_SOFTRESET_SDIOSYS (1 << 6)
#define TSB_SOFTRESET_SDIOSD  (1 << 7)
#define TSB_SOFTRESET_MBOX    (1 << 9)
#define TSB_SOFTRESET_GDMA    (1 << 10)
#define TSB_SOFTRESET_TMR     (1 << 11)
#define TSB_SOFTRESET_WDT     (1 << 12)
#define TSB_SOFTRESET_TIMER   (1 << 13)
#define TSB_SOFTRESET_PWMODP  (1 << 14)
#define TSB_SOFTRESET_PWMODS  (1 << 15)
#define TSB_SOFTRESET_I2CP    (1 << 16)
#define TSB_SOFTRESET_I2CS    (1 << 17)
#define TSB_SOFTRESET_UARTP   (1 << 18)
#define TSB_SOFTRESET_UARTS   (1 << 19)
#define TSB_SOFTRESET_SPIP    (1 << 20)
#define TSB_SOFTRESET_SPIS    (1 << 21)
#define TSB_SOFTRESET_GPIO    (1 << 22)

#define UART_RBR_THR_DLL    (UART_BASE + 0x0)
#define UART_IER_DLH        (UART_BASE + 0x4)
#define UART_FCR_IIR        (UART_BASE + 0x8)
#define UART_LCR            (UART_BASE + 0xc)
#define UART_LSR            (UART_BASE + 0x14)

#define UART_115200_BPS     26
#define UART_DLL_115200     ((UART_115200_BPS >> 0) & 0xff)
#define UART_DLH_115200     ((UART_115200_BPS >> 8) & 0xff)
#define UART_LCR_DLAB  (0x1 << 7) // Divisor latch
#define UART_LCR_DLS_8 (0x3 << 0) // 8 bit

#define UART_FCR_IIR_IID0_FIFOE  (0x1 << 0) // FIFO Enable
#define UART_FCR_IIR_IID1_RFIFOR (0x2 << 0) // FIFO RX Reset
#define UART_FCR_IIR_IID1_XFIFOR (0x3 << 0) // FIFO RX Reset

#define UART_LSR_THRE (0x1 << 5)

void tsb_lowsetup(void) {
    int i;

    putreg32(DEFAULT_PERIPHERAL_CLOCK_BITS, TSB_SYSCTL_CLOCKGATING0);
    putreg32(getreg32(TSB_SYSCTL_PINSHARE) | 3, TSB_SYSCTL_PINSHARE);
    putreg32(TSB_SYSCTL_CLOCK_UARTPCLK | TSB_SYSCTL_CLOCK_UARTSCLK,
             TSB_SYSCTL_CLOCKENABLE0);
    putreg32(TSB_SOFTRESET_UARTP | TSB_SOFTRESET_UARTS, TSB_SYSCTL_SOFTRESET0);
    putreg32(TSB_SOFTRESET_UARTP | TSB_SOFTRESET_UARTS,
             TSB_SYSCTL_SOFTRESETRELEASE0);

    /*
     * The controller requires "several cycles" after reset to stabilize before
     * register writes will work. Try this a few times.
     *
     * And the LORD spake, saying, "First shalt thou take out the Holy Pin,
     * then shalt thou count to three, no more, no less. Three shall be the
     * number thou shalt count, and the number of the counting shall be three.
     * Four shalt thou not count, neither count thou two, excepting that thou
     * then proceed to three. Five is right out. Once the number three, being
     * the third number, be reached, then lobbest thou thy Holy Hand Grenade
     * of Antioch towards thy foe, who being naughty in My sight, shall snuff
     * it.
     *      -The Book of Armaments
     */
    for (i = 0; i < 3; i++) {
        putreg32(UART_LCR_DLAB | UART_LCR_DLS_8, UART_LCR);
        putreg32(UART_DLL_115200, UART_RBR_THR_DLL);
        putreg32(UART_DLH_115200, UART_IER_DLH);
        putreg32(UART_LCR_DLS_8, UART_LCR);
        putreg32(0x0, UART_IER_DLH);
        putreg32(UART_FCR_IIR_IID0_FIFOE | UART_FCR_IIR_IID1_RFIFOR |
                 UART_FCR_IIR_IID1_XFIFOR, UART_FCR_IIR);
    }
}

void up_lowputc(int c){
    while ((getreg32(UART_LSR) & UART_LSR_THRE) != UART_LSR_THRE)
        ;

    putreg32(c, UART_RBR_THR_DLL);
}


#ifndef CONFIG_16550_UART

int up_putc(int ch) {
    if (ch == '\n') {
        up_lowputc('\r');
    }

    up_lowputc(ch);
    return ch;
}

#endif
