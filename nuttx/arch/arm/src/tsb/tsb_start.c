#include <nuttx/config.h>
#include <nuttx/init.h>
#include <arch/board/board.h>
#include "up_arch.h"
#include "up_internal.h"
#include "ram_vectors.h"
#include "nvic.h"

#include "tsb_scm.h"
#include "tsb_lowputc.h"

#ifdef DEBUG_EARLY_BOOT
#define dbg(x) up_lowputc(x)
#else
#define dbg(x)
#endif

void tsb_start(void);

extern uint32_t _stext_lma;
extern uint32_t _sdata_lma;

void __start(void) __attribute__((section(".bootstrap.loader")));

void __start(void) {
#ifdef CONFIG_BOOT_COPYTORAM
    extern void bootstrap(void);
    bootstrap();
#endif

    tsb_start();
}

void tsb_start(void) {
    uint32_t *dst;
    __attribute__((unused)) const uint32_t *src;

#ifdef CONFIG_BOOT_COPYTORAM
    for (src = &_sdata_lma, dst = &_sdata; dst < &_edata;) {
        *dst++ = *src++;
    }
#endif

    /* Zero .bss */
    for (dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0;
    }

    /* Relocate vector table (eg from bootrom) */
    extern uint32_t _vectors;
    putreg32((uint32_t)&_vectors, NVIC_VECTAB);
#ifdef CONFIG_ARCH_RAMVECTORS
    up_ramvec_initialize();
#endif

    /* Configure clocks */
    tsb_clk_init();

    /* Configure the UART so we can get debug output as soon as possible */
    tsb_lowsetup();
    dbg('A');

#ifdef CONFIG_TSB_PINSHARE_ETM
    tsb_set_pinshare(TSB_PIN_ETM);

#ifdef CONFIG_TSB_TRACE_DRIVESTRENGTH_MIN
    tsb_set_drivestrength(TSB_TRACE_DRIVESTRENGTH, tsb_ds_min);
#elif CONFIG_TSB_TRACE_DRIVESTRENGTH_DEFAULT
    tsb_set_drivestrength(TSB_TRACE_DRIVESTRENGTH, tsb_ds_default);
#elif CONFIG_TSB_TRACE_DRIVESTRENGTH_MAX
    tsb_set_drivestrength(TSB_TRACE_DRIVESTRENGTH, tsb_ds_max);
#endif
#endif

#ifdef CONFIG_16550_UART
    /* early init the 16550 driver */
    up_earlyserialinit();
#endif
    dbg('D');

    tsb_boardinitialize();

    os_start();
}
