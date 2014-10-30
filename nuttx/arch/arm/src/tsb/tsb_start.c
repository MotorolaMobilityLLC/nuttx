#include <nuttx/config.h>
#include <nuttx/init.h>
#include <arch/board/board.h>
#include "up_arch.h"
#include "up_internal.h"

#include "tsb_scm.h"
#include "tsb_lowputc.h"

#ifdef DEBUG_EARLY_BOOT
#define dbg(x) up_lowputc(x)
#else
#define dbg(x)
#endif

void __start(void) {
    uint32_t *dest;

    /* Configure clocks */
    tsb_clk_init();

    /* Configure the UART so we can get debug output as soon as possible */
    tsb_lowsetup();
    dbg('A');

    /* Zero bss section */
    for (dest = &_sbss; dest < &_ebss;) {
        *dest++ = 0;
    }
    dbg('B');

#if 0
    /* We're executing out of SRAM, no need to relocate */
    const uint32_t *src;
    for (src = &_eronly, dest = &_sdata; dest < &_edata;) {
        *dest++ = *src++;
    }
#endif
    dbg('C');

    /* early init the 16550 driver */
    up_earlyserialinit();
    dbg('D');

    tsb_boardinitialize();

    os_start();
}
