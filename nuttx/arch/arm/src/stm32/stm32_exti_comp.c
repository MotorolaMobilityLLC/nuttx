/****************************************************************************
 * arch/arm/src/stm32/stm32_exti_comp.c
 *
 * Copyright (c) 2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_exti.h"
#include "up_arch.h"

#include <arch/irq.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to the COMP EXTI lines */
static xcpt_t stm32_exti_comp_handlers[STM32_COMP_NUM];

/* Comparator EXTI lines */
static const uint32_t stm32_exti_comp_lines[STM32_COMP_NUM] = {
#if defined(CONFIG_STM32_STM32L4X3) || defined (CONFIG_STM32_STM32L4X6)
    EXTI_COMP1,
    EXTI_COMP2
#else
#  error "Unrecognized STM32 chip"
#endif
};

 /****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_exti_comp_isr(int irq, void *context)
{
    int ret = 0;
    uint32_t pr, ln;
    int i;

    /* Examine the state of each comparator line and dispatch interrupts */
    pr = getreg32(STM32_EXTI_PR);
    for (i = 0; i < STM32_COMP_NUM; i++) {
        ln = stm32_exti_comp_lines[i];
        if (pr & ln) {
            /* Clear the pending interrupt */
            putreg32(ln, STM32_EXTI_PR);
            if (stm32_exti_comp_handlers[i]) {
                ret = stm32_exti_comp_handlers[i](irq, context);
            }
        }
    }

    return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_comp
 *
 * Description:
 *   Sets/clears comparator based events and interrupt triggers.
 *
 * Parameters:
 *  - cmp: comparator
 *  - rising/falling edge: enables interrupt on rising/falling edget
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *
 * Returns:
 *   The previous value of the interrupt handler function pointer.  This
 *   value may, for example, be used to restore the previous handler when
 *   multiple handlers are used.
 *
 ****************************************************************************/

xcpt_t stm32_exti_comp(stm32_comp_t cmp, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func)
{
    xcpt_t oldhandler;
    irqstate_t flags;
    uint32_t ln = stm32_exti_comp_lines[cmp];

   /*
    * Disable IRQs so that the handler gets installed correctly before the IRQ
    * is re-enabled.
    */
    flags = irqsave();

    /* Install external interrupt handlers */
    if (func) {
        irq_attach(STM32_IRQ_COMP, stm32_exti_comp_isr);
        up_enable_irq(STM32_IRQ_COMP);
    } else {
        up_disable_irq(STM32_IRQ_COMP);
    }


    /* Configure rising/falling edges */
    modifyreg32(STM32_EXTI_RTSR, risingedge  ? 0 : ln, risingedge  ? ln : 0);
    modifyreg32(STM32_EXTI_FTSR, fallingedge ? 0 : ln, fallingedge ? ln : 0);

    /* Enable Events and Interrupts */
    modifyreg32(STM32_EXTI_EMR, event ? 0 : ln, event ? ln : 0);
    modifyreg32(STM32_EXTI_IMR, func  ? 0 : ln, func  ? ln : 0);

    /* Get the previous IRQ handler and save the new IRQ handler. */
    oldhandler = stm32_exti_comp_handlers[cmp];
    stm32_exti_comp_handlers[cmp] = func;

    /* Re-enable IRQs */
    irqrestore(flags);

    /* Return the old IRQ handler */
    return oldhandler;
}
