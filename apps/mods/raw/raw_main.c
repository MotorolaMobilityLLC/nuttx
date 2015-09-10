/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>

extern int mods_raw_send(uint32_t  len, uint8_t data[]);

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int raw_main(int argc, char *argv[])
#endif
{
  if (argc > 1)
    {
      uint32_t len = strlen(argv[1]) + 1;

      printf("%s\n", argv[1]);
      mods_raw_send(len, (uint8_t *)argv[1]);
    }
  return 0;
}
