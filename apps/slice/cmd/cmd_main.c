/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c.h>
#include <stdio.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define I2C_SLAVE_BUS  1
#define I2C_MASTER_BUS 2

#define I2C_ADDRESS_SELF 0x55

#define EXTRA_FMT "%s: "
#define EXTRA_ARG ,__FUNCTION__
#define logd(format, ...) \
  printf(EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int slice_cmd_main(int argc, char *argv[])
#endif
{
  FAR struct i2c_dev_s *dev1;

  dev1 = up_i2cinitialize(I2C_SLAVE_BUS);

#ifdef CONFIG_I2C_SLAVE
  if (I2C_SETOWNADDRESS(dev1, I2C_ADDRESS_SELF, 7) == OK) {
    I2C_REGISTERCALLBACK(dev1, NULL);
    logd("Slave setup complete!\n");
  } else {
    logd("Slave setup failed!\n");
  }
#endif

  // TODO: implement me
  //(void)up_i2cuninitialize(dev1);

  return 0;
}

