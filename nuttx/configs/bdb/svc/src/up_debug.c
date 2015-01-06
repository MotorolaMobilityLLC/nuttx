/****************************************************************************
 * configs/bdb/svc/src/up_debug.h
 * SVC support for debug print messages:
 * - allows to control the dump of the data, per component and with
 *   a debug level,
 * - dumps data out to an externally exposed interface (e.g. UART).
 *
 * Copyright (C) 2014 Google, Inc.
 *
 ****************************************************************************/
#define DBG_COMP DBG_DBG    /* DBG_COMP macro of the component */
#include "up_debug.h"

/* Debug control internal data */
dbg_ctrl_t dbg_ctrl = { DBG_ALL, DBG_INFO };
/* Debug buffer, dynamically allocated/freed */
char *dbg_msg;
char dbg_rescue[DBG_RESCUE_SIZE];


/* Get the level and components to enable debug for */
void dbg_get_config(uint32_t *comp, uint32_t *level)
{
    *comp = dbg_ctrl.comp;
    *level = dbg_ctrl.lvl;

    dbg_info("%s(): debug comp=0x%x, level=%d\n", __func__,
           dbg_ctrl.comp, dbg_ctrl.lvl);
}

/* Configure the level and components to enable debug for */
int dbg_set_config(uint32_t comp, uint32_t level)
{
    /* DBG_MAX is always enabled */
    if (level > DBG_MAX)
        level = DBG_MAX;

    dbg_ctrl.comp = comp;
    dbg_ctrl.lvl = level;

    dbg_info("%s(): debug comp=0x%x, level=%d\n", __func__,
           dbg_ctrl.comp, dbg_ctrl.lvl);

    return 0;
}
