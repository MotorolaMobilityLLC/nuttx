/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <apps/greybus-utils/svc.h>

#define EXTRA_FMT "%s: "
#define EXTRA_ARG ,__FUNCTION__
#define logd(format, ...) \
  lowsyslog(EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#define SLICE_NUM_REGS 1

#define SLICE_REG_SVC     0     /* SVC message register */

#ifdef CONFIG_EXAMPLES_NSH
extern int nsh_main(int argc, char *argv[]);
#endif

struct slice_cmd_data
{
  FAR struct i2c_dev_s *i2c;
  int reg;
  unsigned char *reg_val;
  int reg_size;
  int reg_idx;
};

static struct slice_cmd_data slice_cmd_self =
{
    .i2c = NULL,
    .reg = -1,
    .reg_val = NULL,
    .reg_size = 0,
    .reg_idx = 0,
};

static int slice_cmd_recv_from_svc(void *buf, size_t length)
{
  logd("length=%d\n", length);

  // TODO: Add queue instead of just dropped msg
  if (!slice_cmd_self.reg_val) {
    slice_cmd_self.reg_val = malloc(length);
    if (slice_cmd_self.reg_val) {
      memcpy(slice_cmd_self.reg_val, buf, length);
      slice_cmd_self.reg_size = length;
      return 0;
    }
  }

  return -1;
}

/* called when master is writing to slave */
static int slice_cmd_read_cb(void *v)
{
  struct slice_cmd_data *slf = (struct slice_cmd_data *)v;
  FAR struct i2c_dev_s *dev = slf->i2c;
  uint8_t val;

  I2C_SLAVE_READ(dev, &val, sizeof(val));
  if (slf->reg < 0)
    {
      if (val < SLICE_NUM_REGS)
        {
          slf->reg = val;
        }
    }
  else 
    {
      /* Writes to registers not supported */
    }

  return 0;
}

/* called when master is reading from slave */
static int slice_cmd_write_cb(void *v)
{
  struct slice_cmd_data *slf = (struct slice_cmd_data *)v;
  FAR struct i2c_dev_s *dev = slf->i2c;
  uint8_t val = 0;

  if ((slf->reg == SLICE_REG_SVC) && (slf->reg_val != NULL)) {
    val = slf->reg_val[slf->reg_idx];
    slf->reg_idx = (slf->reg_idx + 1) % slf->reg_size;
    if (slf->reg_idx == 0) {
      /* host has read entire SVC message */
      free(slf->reg_val);
      slf->reg_val = NULL;
      slf->reg_size = 0;
    }
  }

  I2C_SLAVE_WRITE(dev, &val, sizeof(val));
  return 0;
}

/* called to end transaction */
static int slice_cmd_stop_cb(void *v)
{
  struct slice_cmd_data *slf = (struct slice_cmd_data *)v;

  slf->reg = -1;
  slf->reg_idx = 0;
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
  slice_cmd_self.i2c = dev1;

  if (I2C_SETOWNADDRESS(dev1, CONFIG_SLICE_CMD_I2C_SLAVE_ADDR, 7) == OK) {
    I2C_REGISTERCALLBACK(dev1, &cb_ops, &slice_cmd_self);
    logd("Slave setup complete!\n");
  } else {
    logd("Slave setup failed!\n");
  }

  svc_register(slice_cmd_recv_from_svc);
  send_svc_handshake();

#ifdef CONFIG_EXAMPLES_NSH
  logd("Calling NSH\n");
  return nsh_main(argc, argv);
#else
  return 0;
#endif
}
