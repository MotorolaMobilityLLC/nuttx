/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdio.h>

#define EXTRA_FMT "%s: "
#define EXTRA_ARG ,__FUNCTION__
#define logd(format, ...) \
  printf(EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#define MODS_NUM_REGS 8

FAR struct spi_dev_s *dev1;
int txn_status = -1;

struct mods_reg_data
{
  FAR struct spi_dev_s *dev;
  int addr;
  uint8_t regs[MODS_NUM_REGS];
  uint8_t crc[2];
};

/* called when master is writing to slave */
static int mods_reg_read_cb(void *v)
{
  struct mods_reg_data *slf = (struct mods_reg_data *)v;
  FAR struct spi_dev_s *dev = slf->dev;
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  uint8_t val;

  SPI_SLAVE_READ(dev, &val);

  if (slf->addr < 0)
    {
      if (val < MODS_NUM_REGS)
        {
          slf->addr = val;
        }
    }
  else
    {
        slf->regs[slf->addr] = val;
        slf->addr = (slf->addr + 1) % MODS_NUM_REGS;
    }
#endif
#ifdef CONFIG_STM32_SPI_DMA
  SPI_EXCHANGE(dev,slf->regs, slf->regs, sizeof(slf->regs) + 2);
#endif
  txn_status = 0;
  return 0;
}

/* called when master is reading from slave */
static int mods_reg_write_cb(void *v)
{
#ifdef CONFIG_STM32_SPI_INTERRUPTS
  struct mods_reg_data *slf = (struct mods_reg_data *)v;
  FAR struct spi_dev_s *dev = slf->dev;
  uint8_t val = 0x55;

  SPI_SLAVE_WRITE(dev, &val);
#endif
  return 0;
}

/* called to end transaction */
static int mods_reg_txn_end_cb(void *v)
{
  struct mods_reg_data *slf = (struct mods_reg_data *)v;

  slf->addr = -1;

  return 0;
}

/* called to end transaction */
static int mods_reg_txn_err_cb(void *v)
{
  struct mods_reg_data *slf = (struct mods_reg_data *)v;
  FAR struct spi_dev_s *dev = slf->dev;
  txn_status = -1;

#ifdef CONFIG_STM32_SPI_DMA
 SPI_EXCHANGE(dev,slf->regs, slf->regs, sizeof(slf->regs) + 2);
 logd("Slave DMA armed! from txn_err\n");
 txn_status = 0;
#endif
  return 0;
}

static struct spi_cb_ops_s cb_ops =
{
  .read = mods_reg_read_cb,
  .write = mods_reg_write_cb,
  .txn_end = mods_reg_txn_end_cb,
  .txn_err = mods_reg_txn_err_cb,
};


static struct mods_reg_data mods_self =
{
    .regs = { 0x00, 0x01, 0x02, 0x03,
              0x04, 0x05, 0x06, 0x07, },
    .crc = { 0x00, 0x00, },
    .addr = -1,
};

static void dump_regs(void)
{
    int i;

    logd("address    %d\n",  mods_self.addr);
    for (i = 0; i < MODS_NUM_REGS; i++)
    {
        logd("%d 0x%02x\n", i, mods_self.regs[i]);
    }
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int mods_spi_reg_main(int argc, char *argv[])
#endif
{
  if (dev1 == NULL)
    {
      dev1 = up_spiinitialize(CONFIG_MODS_SPI_REG_BUS);
      mods_self.dev = dev1;
      logd("Slave init complete!\n");
#ifdef CONFIG_SPI_SLAVE
      SPI_SLAVE_REGISTERCALLBACK(dev1, &cb_ops, &mods_self);
      logd("Slave setup complete!\n");
#endif
    }

  dump_regs();

#ifdef CONFIG_STM32_SPI_DMA
  if (txn_status == -1)
    {
      SPI_EXCHANGE(dev1,mods_self.regs, mods_self.regs, sizeof(mods_self.regs) + 2);
      logd("Slave DMA armed!\n");
    }
#endif
  return 0;
}
