/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdio.h>

#define SPI_SLAVE_BUS  1

#define EXTRA_FMT "%s: "
#define EXTRA_ARG ,__FUNCTION__
#define logd(format, ...) \
  printf(EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#define SLICE_NUM_REGS 8

struct slice_reg_data
{
  FAR struct spi_dev_s *dev;
  int addr;
  uint8_t regs[SLICE_NUM_REGS];
};

/* called when master is writing to slave */
static int slice_reg_read_cb(void *v)
{
  struct slice_reg_data *slf = (struct slice_reg_data *)v;
  FAR struct spi_dev_s *dev = slf->dev;
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  uint8_t val;

  SPI_SLAVE_READ(dev, &val);

  if (slf->addr < 0)
    {
      if (val < SLICE_NUM_REGS)
        {
          slf->addr = val;
        }
    }
  else 
    {
        slf->regs[slf->addr] = val;
        slf->addr = (slf->addr + 1) % SLICE_NUM_REGS;
    }
#endif
#ifdef CONFIG_STM32_SPI_DMA
  SPI_EXCHANGE(dev,slf->regs, slf->regs, sizeof(slf->regs));
#endif
  return 0;
}

/* called when master is reading from slave */
static int slice_reg_write_cb(void *v)
{
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  struct slice_reg_data *slf = (struct slice_reg_data *)v;
  FAR struct spi_dev_s *dev = slf->dev;
  uint8_t val = 0x55;

  SPI_SLAVE_WRITE(dev, &val);
#endif
  return 0;
}

/* called to end transaction */
static int slice_reg_txn_end_cb(void *v)
{
  struct slice_reg_data *slf = (struct slice_reg_data *)v;

  slf->addr = -1;

  return 0;
}

static struct spi_cb_ops_s cb_ops =
{
  .read = slice_reg_read_cb,
  .write = slice_reg_write_cb,
  .txn_end = slice_reg_txn_end_cb,
};


static struct slice_reg_data slice_self =
{
    .regs = { 0x00, 0x01, 0x02, 0x03,
              0x04, 0x05, 0x06, 0x07, },
    .addr = -1,
};

static void dump_regs(void)
{
    int i;

    logd("address    %d\n",  slice_self.addr);
    for (i = 0; i < SLICE_NUM_REGS; i++)
    {
        logd("%d 0x%02x\n", i, slice_self.regs[i]);
    }
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int slice_spi_reg_main(int argc, char *argv[])
#endif
{
  FAR struct spi_dev_s *dev1;

  dev1 = up_spiinitialize(SPI_SLAVE_BUS);
  slice_self.dev = dev1;

#ifdef CONFIG_SPI_SLAVE
  SPI_SLAVE_REGISTERCALLBACK(dev1, &cb_ops, &slice_self);
  logd("Slave setup complete!\n");
#endif

  dump_regs();

#ifdef CONFIG_STM32_SPI_DMA
  SPI_EXCHANGE(dev1,slice_self.regs, slice_self.regs, sizeof(slice_self.regs));
#endif
  return 0;
}

