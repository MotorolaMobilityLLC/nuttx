/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#include <apps/builtin.h>
#include <nuttx/config.h>
#include <nuttx/i2c.h>
#include <stdio.h>

#define EXTRA_FMT "%s: "
#define EXTRA_ARG ,__FUNCTION__
#define logd(format, ...) \
  printf(EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

static int slice_cmd_read_cb(void *v)
{
  FAR struct i2c_dev_s *dev = (struct i2c_dev_s *)v;
  uint8_t val;
  I2C_SLAVE_READ(dev, &val, sizeof(val));
  return 0;
}

static int slice_cmd_write_cb(void *v)
{
  FAR struct i2c_dev_s *dev = (struct i2c_dev_s *)v;
  uint8_t val = 42;
  I2C_SLAVE_WRITE(dev, &val, sizeof(val));
  return 0;
}

static struct i2c_cb_ops_s cb_ops =
{
  .read = slice_cmd_read_cb,
  .write = slice_cmd_write_cb,
};

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int slice_cmd_main(int argc, char *argv[])
#endif
{
  FAR struct i2c_dev_s *dev1;

  dev1 = up_i2cinitialize(CONFIG_SLICE_CMD_I2C_SLAVE_BUS);

  if (I2C_SETOWNADDRESS(dev1, CONFIG_SLICE_CMD_I2C_SLAVE_ADDR, 7) == OK) {
    I2C_REGISTERCALLBACK(dev1, &cb_ops, dev1);
    logd("Slave setup complete!\n");
  } else {
    logd("Slave setup failed!\n");
  }

  // Attempt to start nsh shell if present
  FAR char *args[] = {"nsh", NULL};
  exec_builtin(args[0], args, NULL, 0);

  while (true) {
    // TODO: Implement me. Sleeping for 1 day in the meantime.
    sleep(86400);
  }

  (void)up_i2cuninitialize(dev1);

  return 0;
}

