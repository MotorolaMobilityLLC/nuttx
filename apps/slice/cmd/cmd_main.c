/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c.h>
#include <stdio.h>

#define EXTRA_FMT "%s: "
#define EXTRA_ARG ,__FUNCTION__
#define logd(format, ...) \
  printf(EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#ifdef CONFIG_EXAMPLES_NSH
extern int nsh_main(int argc, char *argv[]);
#endif

/* called when master is writing to slave */
static int slice_cmd_read_cb(void *v)
{
  FAR struct i2c_dev_s *dev = (struct i2c_dev_s *)v;
  uint8_t val;
  I2C_SLAVE_READ(dev, &val, sizeof(val));
  return 0;
}

/* called when master is reading from slave */
static int slice_cmd_write_cb(void *v)
{
  FAR struct i2c_dev_s *dev = (struct i2c_dev_s *)v;
  uint8_t val = 42;
  I2C_SLAVE_WRITE(dev, &val, sizeof(val));
  return 0;
}

/* called to end transaction */
static int slice_cmd_stop_cb(void *v)
{
  return 0;
}

static struct i2c_cb_ops_s cb_ops =
{
  .read = slice_cmd_read_cb,
  .write = slice_cmd_write_cb,
  .stop = slice_cmd_stop_cb,
};

int slice_cmd_main(int argc, char *argv[])
{
  FAR struct i2c_dev_s *dev1;

  dev1 = up_i2cinitialize(CONFIG_SLICE_CMD_I2C_SLAVE_BUS);

  if (I2C_SETOWNADDRESS(dev1, CONFIG_SLICE_CMD_I2C_SLAVE_ADDR, 7) == OK) {
    I2C_REGISTERCALLBACK(dev1, &cb_ops, dev1);
    logd("Slave setup complete!\n");
  } else {
    logd("Slave setup failed!\n");
  }

#ifdef CONFIG_EXAMPLES_NSH
  logd("Calling NSH\n");
  return nsh_main(argc, argv);
#else
  return 0;
#endif
}
