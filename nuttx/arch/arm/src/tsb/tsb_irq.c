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

void up_irqinitialize(void) {
    uint32_t reg;
    int num_priority_registers;

    /* Disable all interrupts */
    putreg32(0, NVIC_IRQ0_31_ENABLE);
    putreg32(0, NVIC_IRQ32_63_ENABLE);

    /* Move vector table to RAM. */
    putreg32((uint32_t)&_vectors, NVIC_VECTAB);

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

void up_disable_irq(int irq) {
    uint32_t bit = 1 << (irq % 32);
    uint32_t reg = NVIC_IRQ_CLEAR(irq);

    putreg32(bit, reg);
}

void up_enable_irq(int irq) {
    uint32_t bit = 1 << (irq % 32);
    uint32_t reg = NVIC_IRQ_ENABLE(irq);

    putreg32(bit, reg);
}

void up_ack_irq(int irq) {
}
