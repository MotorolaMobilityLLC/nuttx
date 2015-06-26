/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>

#include <arch/tsb/gpio.h>
#include <arch/tsb/unipro.h>
#include <arch/tsb/device_table.h>
#include <arch/tsb/driver.h>
#include <apps/greybus-utils/utils.h>
#include <apps/nsh.h>

#ifdef CONFIG_BOARD_HAVE_DISPLAY
#include <arch/board/dsi.h>
#endif

#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
#include <arch/board/csi.h>
#endif

int bridge_main(int argc, char *argv[])
{
    tsb_gpio_register(NULL);
#ifdef CONFIG_BOARD_HAVE_DISPLAY
    display_init();
#endif

#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
    camera_init();
#endif

    tsb_device_table_register();
    tsb_driver_register();

    enable_manifest("IID-1", NULL, 0);
    gb_unipro_init();
    enable_cports();

#ifdef CONFIG_EXAMPLES_NSH
    printf("Calling NSH\n");
    return nsh_main(argc, argv);
#else
    return 0;
#endif
}

