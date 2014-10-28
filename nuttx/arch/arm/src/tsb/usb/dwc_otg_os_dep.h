#ifndef _DWC_OS_DEP_H_
#define _DWC_OS_DEP_H_

/**
 * @file
 *
 * This file contains OS dependent structures.
 *
 */
#include <unistd.h>
#include <stdint.h>

#include "dwc_os.h"
/** The OS page size */
#define DWC_OS_PAGE_SIZE	(sysconf(_SC_PAGE_SIZE))

#define LINUX_VERSION_CODE 1
#define KERNEL_VERSION(a,b,c) (0)

typedef struct os_dependent {
	/** Base address returned from ioremap() */
	void *base;

	/** Register offset for Diagnostic API */
	uint32_t reg_offset;
} os_dependent_t;

#ifdef __cplusplus
}
#endif

#endif /* _DWC_OS_DEP_H_ */
