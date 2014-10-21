#ifndef __ARCH_ARM_INCLUDE_TSB_IRQ_H
#define __ARCH_ARM_INCLUDE_TSB_IRQ_H

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* No peripheral interrupts at the moment */
#define NR_IRQS             (16)

#define TSB_IRQ_NMI         (2)
#define TSB_IRQ_HARDFAULT   (3)
#define TSB_IRQ_MEMFAULT    (4)
#define TSB_IRQ_BUSFAULT    (5)
#define TSB_IRQ_USAGEFAULT  (6)
#define TSB_IRQ_SVCALL      (11)
#define TSB_IRQ_DBGMONITOR  (12)
#define TSB_IRQ_PENDSV      (14)
#define TSB_IRQ_SYSTICK     (15)

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_TSB_IRQ_H */

