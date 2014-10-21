#ifndef __ARCH_ARM_SRC_TSB_TSB_LOWPUTC_H
#define __ARCH_ARM_SRC_TSB_TSB_LOWPUTC_H

#include <nuttx/config.h>
#include "chip.h"

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN void tsb_lowsetup(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TSB_TSB_LOWPUTC_H */

