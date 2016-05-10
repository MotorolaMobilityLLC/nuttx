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

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "nvic.h"
#include "up_arch.h"
#include "up_internal.h"

extern uint32_t _vectors;
volatile uint32_t *current_regs;

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Steps between supported priority values */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 |\
   NVIC_SYSH_PRIORITY_DEFAULT << 16 |\
   NVIC_SYSH_PRIORITY_DEFAULT << 8  |\
   NVIC_SYSH_PRIORITY_DEFAULT)

#define irqn_to_nvic(irqn) \
        ((irqn) - TSB_IRQ_EXT_BASE)

#define nvic_to_irqn(nvic) \
        ((nvic) + TSB_IRQ_EXT_BASE)

#define IRQ_NAME(irq)                \
    [irqn_to_nvic(irq)] = #irq

#if CONFIG_DEBUG_VERBOSE
const char *irq_names[] = {
    IRQ_NAME(TSB_IRQ_HSIC),
    IRQ_NAME(TSB_IRQ_RESERVED0),
    IRQ_NAME(TSB_IRQ_RESERVED1),
    IRQ_NAME(TSB_IRQ_I2SOERR),
    IRQ_NAME(TSB_IRQ_I2SO),
    IRQ_NAME(TSB_IRQ_I2SIERR),
    IRQ_NAME(TSB_IRQ_I2SI),
    IRQ_NAME(TSB_IRQ_GPIO),
    IRQ_NAME(TSB_IRQ_SPI),
    IRQ_NAME(TSB_IRQ_UART),
    IRQ_NAME(TSB_IRQ_I2C),
    IRQ_NAME(TSB_IRQ_PWM),
    IRQ_NAME(TSB_IRQ_TMR0),
    IRQ_NAME(TSB_IRQ_TMR1),
    IRQ_NAME(TSB_IRQ_TMR2),
    IRQ_NAME(TSB_IRQ_TMR3),
    IRQ_NAME(TSB_IRQ_TMR4),
    IRQ_NAME(TSB_IRQ_CDSI0_RX_LNGPKT),
    IRQ_NAME(TSB_IRQ_CDSI0_RX_SHTPKT),
    IRQ_NAME(TSB_IRQ_CDSI0_RX_DSILPTX),
    IRQ_NAME(TSB_IRQ_CDSI0_RX_DSIRXTRIG),
    IRQ_NAME(TSB_IRQ_CDSI0_RX_TXERR),
    IRQ_NAME(TSB_IRQ_CDSI0_RX_RXERR),
    IRQ_NAME(TSB_IRQ_CDSI0_RX_LPRXSTATE),
    IRQ_NAME(TSB_IRQ_CDSI0_TX_APPSIDEERR),
    IRQ_NAME(TSB_IRQ_CDSI0_TX_CMD),
    IRQ_NAME(TSB_IRQ_CDSI0_TX_DSIRXERR),
    IRQ_NAME(TSB_IRQ_CDSI0_TX_DSIRXTRIG),
    IRQ_NAME(TSB_IRQ_CDSI0_TX_DSIPRTO),
    IRQ_NAME(TSB_IRQ_CDSI0_TX_DSIRXSTATE),
    IRQ_NAME(TSB_IRQ_CDSI0_TX_INIT),
    IRQ_NAME(TSB_IRQ_CDSI0_AL_RX_FIFO_OVF),
    IRQ_NAME(TSB_IRQ_CDSI0_AL_TX),
    IRQ_NAME(TSB_IRQ_CDSI1_RX_LNGPKT),
    IRQ_NAME(TSB_IRQ_CDSI1_RX_SHTPKT),
    IRQ_NAME(TSB_IRQ_CDSI1_RX_DSILPTX),
    IRQ_NAME(TSB_IRQ_CDSI1_RX_DSIRXTRIG),
    IRQ_NAME(TSB_IRQ_CDSI1_RX_TXERR),
    IRQ_NAME(TSB_IRQ_CDSI1_RX_RXERR),
    IRQ_NAME(TSB_IRQ_CDSI1_RX_LPRXSTATE),
    IRQ_NAME(TSB_IRQ_CDSI1_TX_APPSIDEERR),
    IRQ_NAME(TSB_IRQ_CDSI1_TX_CMD),
    IRQ_NAME(TSB_IRQ_CDSI1_TX_DSIRXERR),
    IRQ_NAME(TSB_IRQ_CDSI1_TX_DSIRXTRIG),
    IRQ_NAME(TSB_IRQ_CDSI1_TX_DSIPRTO),
    IRQ_NAME(TSB_IRQ_CDSI1_TX_DSIRXSTATE),
    IRQ_NAME(TSB_IRQ_CDSI1_TX_INIT),
    IRQ_NAME(TSB_IRQ_CDSI1_AL_RX_FIFO_OVF),
    IRQ_NAME(TSB_IRQ_CDSI1_AL_TX),
    IRQ_NAME(TSB_IRQ_UHSWKUP),
    IRQ_NAME(TSB_IRQ_UHS),
    IRQ_NAME(TSB_IRQ_MBOX00),
    IRQ_NAME(TSB_IRQ_MBOX01),
    IRQ_NAME(TSB_IRQ_MBOX02),
    IRQ_NAME(TSB_IRQ_MBOX03),
    IRQ_NAME(TSB_IRQ_MBOX04),
    IRQ_NAME(TSB_IRQ_MBOX05),
    IRQ_NAME(TSB_IRQ_MBOX06),
    IRQ_NAME(TSB_IRQ_MBOX07),
    IRQ_NAME(TSB_IRQ_MBOX_SNAPSHOT),
    IRQ_NAME(TSB_IRQ_GDMACABORT),
    IRQ_NAME(TSB_IRQ_GDMAC00),
    IRQ_NAME(TSB_IRQ_GDMAC01),
    IRQ_NAME(TSB_IRQ_GDMAC02),
    IRQ_NAME(TSB_IRQ_GDMAC03),
    IRQ_NAME(TSB_IRQ_GDMAC04),
    IRQ_NAME(TSB_IRQ_GDMAC05),
    IRQ_NAME(TSB_IRQ_GDMAC06),
    IRQ_NAME(TSB_IRQ_GDMAC07),
    IRQ_NAME(TSB_IRQ_GDMAC08),
    IRQ_NAME(TSB_IRQ_GDMAC09),
    IRQ_NAME(TSB_IRQ_GDMAC10),
    IRQ_NAME(TSB_IRQ_GDMAC11),
    IRQ_NAME(TSB_IRQ_GDMAC12),
    IRQ_NAME(TSB_IRQ_GDMAC13),
    IRQ_NAME(TSB_IRQ_GDMAC14),
    IRQ_NAME(TSB_IRQ_GDMAC15),
    IRQ_NAME(TSB_IRQ_GDMAC16),
    IRQ_NAME(TSB_IRQ_GDMAC17),
    IRQ_NAME(TSB_IRQ_GDMAC18),
    IRQ_NAME(TSB_IRQ_GDMAC19),
    IRQ_NAME(TSB_IRQ_GDMAC20),
    IRQ_NAME(TSB_IRQ_GDMAC21),
    IRQ_NAME(TSB_IRQ_GDMAC22),
    IRQ_NAME(TSB_IRQ_GDMAC23),
    IRQ_NAME(TSB_IRQ_GDMAC24),
    IRQ_NAME(TSB_IRQ_GDMAC25),
    IRQ_NAME(TSB_IRQ_GDMAC26),
    IRQ_NAME(TSB_IRQ_GDMAC27),
    IRQ_NAME(TSB_IRQ_GDMAC28),
    IRQ_NAME(TSB_IRQ_GDMAC29),
    IRQ_NAME(TSB_IRQ_GDMAC30),
    IRQ_NAME(TSB_IRQ_GDMAC31),
    IRQ_NAME(TSB_IRQ_UNIPRO),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM00),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM01),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM02),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM03),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM04),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM05),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM06),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM07),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM08),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM09),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM10),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM11),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM12),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM13),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM14),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM15),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM16),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM17),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM18),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM19),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM20),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM21),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM22),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM23),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM24),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM25),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM26),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM27),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM28),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM29),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM30),
    IRQ_NAME(TSB_IRQ_UNIPRO_RX_EOM31),
};
#endif

void up_irqinitialize(void) {
    uint32_t reg;
    int num_priority_registers;

    /* Disable all interrupts */
    num_priority_registers = getreg32(NVIC_ICTR) + 1;
    reg = NVIC_IRQ0_31_ENABLE;
    while (num_priority_registers--) {
        putreg32(0, reg);
        reg += 4;
    }

    /* Set nvic interrupts and exceptions to midpoint priority */
    putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
    putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
    putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

    /* Now set all of the peripheral interrupt lines to the default priority */
    num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;
    reg = NVIC_IRQ0_3_PRIORITY;
    while (num_priority_registers--) {
        putreg32(DEFPRIORITY32, reg);
        reg += 4;
    }

    /* current_regs is non-NULL only while processing an interrupt */
    current_regs = NULL;

    /*
     * Attach the SVCall and Hard Fault exception handlers. The SVCall
     * exception is used for performing context switches; The Hard Fault
     * must also be caught because a SVCall may show up as a Hard Fault
     * under certain conditions.
     */
    irq_attach(TSB_IRQ_SVCALL, up_svcall);
    irq_attach(TSB_IRQ_HARDFAULT, up_hardfault);

    irqenable();
}

void up_disable_irq(int irqn) {
    int irq = irqn_to_nvic(irqn);
    uint32_t bit = 1 << (irq % 32);
    uint32_t reg = NVIC_IRQ_CLEAR(irq);

    putreg32(bit, (uint32_t*)reg);
}

void up_enable_irq(int irqn) {
    int irq = irqn_to_nvic(irqn);
    uint32_t bit = 1 << (irq % 32);
    uint32_t reg = NVIC_IRQ_ENABLE(irq);

    putreg32(bit, (uint32_t*)reg);
}

void up_ack_irq(int irq) {
    (void)irq;
}

#if CONFIG_DEBUG_VERBOSE
/*
 * Print human-readable strings for enabled bits in peripheral NVIC space
 */
static inline void dbg_irq_names(uint32_t *base, size_t lines_per_word) {
    uint32_t val;
    unsigned int irq;
    uint32_t *reg = base;

    for (irq = 0; irq < ARMV7M_PERIPHERAL_INTERRUPTS; irq++) {
        val = getreg32(&reg[irq / lines_per_word]);
        if (val & (1 << (irq % lines_per_word))) {
            lldbg("    [%03u:%s]\n", irq, irq_names[irq]);
        }
    }
}
#endif


/**
 * @brief Clear pending IRQ
 * @param irqn IRQ number to clear
 */
void tsb_irq_clear_pending(int irqn) {
    int irq = irqn_to_nvic(irqn);
    uint32_t bit = 1 << (irq % 32);
    uint32_t reg = NVIC_IRQ_CLRPEND(irq);

    putreg32(bit, (uint32_t*)reg);
}


#if CONFIG_DEBUG_VERBOSE
/**
 * @brief Print out a bunch of information about the NVIC and currently
 *        configured peripheral interrupts on the low-level debug console
 */
void tsb_dumpnvic(void) {
    uint32_t *reg;
    uint32_t nr_prio_regs = ARMV7M_PERIPHERAL_INTERRUPTS/4;
    uint32_t i;
    irqstate_t flags;

    flags = irqsave();
    lldbg("  INTCTRL:     %08x VECTAB: %08x\n",
          getreg32(NVIC_INTCTRL),
          getreg32(NVIC_VECTAB));

    lldbg("  SYSH_PRIO:   %08x %08x %08x\n",
          getreg32(NVIC_SYSH4_7_PRIORITY),
          getreg32(NVIC_SYSH8_11_PRIORITY),
          getreg32(NVIC_SYSH12_15_PRIORITY));

    reg = (uint32_t*)NVIC_IRQ0_3_PRIORITY;
    for (i = 0; i < nr_prio_regs; i += 4) {
        lldbg("  IRQ PRIO:    %08x %08x %08x %08x\n",
              getreg32(reg),
              getreg32(reg + 1),
              getreg32(reg + 2),
              getreg32(reg + 3));
        reg += 4;
    }

    lldbg("  IRQ ENABLE:  %08x %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_31_ENABLE),
          getreg32(NVIC_IRQ32_63_ENABLE),
          getreg32(NVIC_IRQ64_95_ENABLE),
          getreg32(NVIC_IRQ96_127_ENABLE),
          getreg32(NVIC_IRQ128_159_ENABLE));
    dbg_irq_names((uint32_t*)NVIC_IRQ_ENABLE(0), 32);

    lldbg("  IRQ PENDING: %08x %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_31_PEND),
          getreg32(NVIC_IRQ32_63_PEND),
          getreg32(NVIC_IRQ64_95_PEND),
          getreg32(NVIC_IRQ96_127_PEND),
          getreg32(NVIC_IRQ128_159_PEND));
    dbg_irq_names((uint32_t*)NVIC_IRQ_PEND(0), 32);

    irqrestore(flags);
}
#endif

#ifdef CONFIG_USEC_MEASURE_PERF

/************************************************************************
 * Name: tsb_irq_name
 *
 * Description:
 *   Return a string name per irq number.
 *
 * Parameters:
 *   irq - irq number
 *
 *   If invalid irq number is given string "IRQ_TOO_LARGE" is returned
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ************************************************************************/

const char *irq_base_names[] = {
	"IRQ_UNKNOWN",
	"IRQ_UNKNOWN",
	"IRQ_NMI",
	"IRQ_HARDFAULT",
	"IRQ_MEMFAULT",
	"IRQ_BUSFAULT",
	"IRQ_USAGEFAULT",
	"IRQ_UNKNOWN",
	"IRQ_UNKNOWN",
	"IRQ_UNKNOWN",
	"IRQ_UNKNOWN",
	"IRQ_UNKNOWN",
	"IRQ_SVCALL",
	"IRQ_DBGMONITOR",
	"IRQ_PENDSV",
	"IRQ_SYSTICK"
};

/* macro to allow compile time checks with sizeof operator */
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

FAR const char* tsb_irq_name(int irq)
{
	/* cause compile time assert if string array sizes are not aligned with size macros */
	BUILD_BUG_ON((sizeof(irq_names)/sizeof(irq_names[0]) != ARMV7M_PERIPHERAL_INTERRUPTS));
	BUILD_BUG_ON((sizeof(irq_base_names)/sizeof(irq_base_names[0]) != TSB_IRQ_EXT_BASE));
	BUILD_BUG_ON(NR_IRQS != (TSB_IRQ_EXT_BASE + ARMV7M_PERIPHERAL_INTERRUPTS));

    if(irq >= NR_IRQS)
    {
        return "IRQ_TOO_LARGE";
    }
    else if(irq < TSB_IRQ_EXT_BASE)
    {
        return irq_base_names[irq];
    }

    return irq_names[irq];
}

#endif
