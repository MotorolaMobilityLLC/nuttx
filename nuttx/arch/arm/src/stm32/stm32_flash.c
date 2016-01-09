/************************************************************************************
 * arch/arm/src/stm32/stm32_flash.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/* Provides standard flash access functions, to be used by the  flash mtd driver.
 * The interface is defined in the include/nuttx/progmem.h
 *
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <errno.h>
#include <string.h>

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include "stm32_flash.h"
#include "stm32_rcc.h"
#include "stm32_waste.h"

#include "up_arch.h"

#if defined(CONFIG_STM32_STM32F10XX)  || \
    defined (CONFIG_STM32_STM32F40XX) || \
    defined (CONFIG_STM32_STM32L4X6) || \
    defined (CONFIG_STM32_STM32L4X3)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

#if defined(CONFIG_STM32_STM32F10XX)
#define FLASH_CR_PAGE_ERASE              FLASH_CR_PER
#define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPRT_ERR
#elif defined(CONFIG_STM32_STM32F40XX)
#define FLASH_CR_PAGE_ERASE              FLASH_CR_SER
#define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPERR
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

void stm32_flash_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      up_waste();
    }

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      putreg32(FLASH_KEY1, STM32_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_KEYR);
    }
}

void stm32_flash_lock(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_LOCK);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#if defined CONFIG_STM32_STM32F10XX || defined(CONFIG_STM32_STM32L4X6) || \
    defined(CONFIG_STM32_STM32L4X3)

size_t up_progmem_pagesize(size_t page)
{
  return STM32_FLASH_PAGESIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if (addr >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  return addr / STM32_FLASH_PAGESIZE;
}

size_t up_progmem_getaddress(size_t page)
{
  if (page >= STM32_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  return page * STM32_FLASH_PAGESIZE + STM32_FLASH_BASE;
}

#endif /* def CONFIG_STM32_STM32F10XX */

#ifdef CONFIG_STM32_STM32F40XX

size_t up_progmem_pagesize(size_t page)
{
  static const size_t page_sizes[STM32_FLASH_NPAGES] =
    {
      16 * 1024,
      16 * 1024,
      16 * 1024,
      16 * 1024,
      64 * 1024,
      128 * 1024,
      128 * 1024,
      128 * 1024,
    };

  if (page >= sizeof(page_sizes) / sizeof(*page_sizes))
    {
      return 0;
    }
  else
    {
      return page_sizes[page];
    }
}

ssize_t up_progmem_getpage(size_t addr)
{
  size_t page_end = 0;
  size_t i;

  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if (addr >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  for (i = 0; i < STM32_FLASH_NPAGES; ++i)
    {
      page_end += up_progmem_pagesize(i);
      if (page_end > addr)
        {
          return i;
        }
    }

  return -EFAULT;
}

size_t up_progmem_getaddress(size_t page)
{
  size_t base_address = STM32_FLASH_BASE;
  size_t i;

  if (page >= STM32_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  for (i = 0; i < page; ++i)
    {
      base_address += up_progmem_pagesize(i);
    }

  return base_address;
}

#endif /* def CONFIG_STM32_STM32F40XX */

size_t up_progmem_npages(void)
{
  return STM32_FLASH_NPAGES;
}

bool up_progmem_isuniform(void)
{
#ifdef STM32_FLASH_PAGESIZE
  return true;
#else
  return false;
#endif /* def STM32_FLASH_PAGESIZE */
}

#if defined(CONFIG_STM32_STM32L4X6) || defined(CONFIG_STM32_STM32L4X3)

ssize_t _up_progmem_erase_page(size_t page)
{
  uint32_t flash_status;
  uint32_t page_offset = (page & STM32_FLASH_PAGE_BANK_MASK);

  /* 1. Check that no Flash memory operation is ongoing by checking the BSY */
  /*    bit in the Flash status register (FLASH_SR).                        */

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  /* 2. Check and clear all error programming flags due to a previous       */
  /*    programming.                                                        */

  flash_status = getreg32(STM32_FLASH_SR);
  if (flash_status)
    {
      modifyreg32(STM32_FLASH_SR, 0, flash_status);
    }

  /* 3. Set the PER bit and select the page you wish to erase (PNB) with the */
  /*    associated bank (BKER) in the Flash control register (FLASH_CR).     */

  if (page < STM32_FLASH_PAGE_PER_BANK)
    {
      modifyreg32(STM32_FLASH_CR, FLASH_CR_BKER, 0);
    }
  else
    {
      modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_BKER);
    }
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PER);
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_SNB(page_offset));

  /* 4. Set the STRT bit in the FLASH_CR register.                         */

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_STRT);

  /* 5. Wait for the BSY bit to be cleared in the FLASH_SR register and    */
  /*    clear previous settings.                                           */

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  modifyreg32(STM32_FLASH_CR,
      FLASH_CR_BKER | FLASH_CR_PER | FLASH_CR_SNB(page_offset), 0);

  return 0;
}

#else

ssize_t _up_progmem_erase_page(size_t page)
{
#ifdef CONFIG_STM32_STM32F10XX
  size_t page_address;
#endif /* def CONFIG_STM32_STM32F10XX */

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PAGE_ERASE);

#if defined(CONFIG_STM32_STM32F10XX)
  /* must be valid - page index checked above */
  page_address = up_progmem_getaddress(page);
  putreg32(page_address, STM32_FLASH_AR);
#elif defined(CONFIG_STM32_STM32F40XX)
  modifyreg32(STM32_FLASH_CR, FLASH_CR_SNB_MASK, FLASH_CR_SNB(page));
#endif

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_STRT);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PAGE_ERASE, 0);

  return 0;
}

#endif

ssize_t up_progmem_erasepage(size_t page)
{
  ssize_t err = 0;

  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Get flash ready and begin erasing single page */

  if (!(getreg32(STM32_RCC_CR) & RCC_CR_HSION))
    {
      return -EPERM;
    }

  stm32_flash_unlock();

  err = _up_progmem_erase_page(page);
  if (err)
    {
       return err;
    }

  stm32_flash_lock();

  /* Verify */
  if (up_progmem_ispageerased(page) == 0)
    {
      return up_progmem_pagesize(page); /* success */
    }
  else
    {
      return -EIO; /* failure */
    }
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != 0xff)
        {
          bwritten++;
        }
    }

  return bwritten;
}

#if defined (CONFIG_STM32_STM32L4X6) || defined (CONFIG_STM32_STM32L4X3)

static inline void write_single_dword(uint32_t *a, const uint32_t dword[])
{
  putreg32(dword[0], a);
  putreg32(dword[1], a + 1);
}

static ssize_t up_progmem_write_dword(size_t addr, const uint32_t buf[], size_t bytes)
{
  uint32_t flash_status;
  size_t w;   /* word counter */
  uint8_t pad[8];     /* one double word worth for any trailing data */
  size_t double_words = bytes / 8;
  size_t pad_bytes = bytes % 8;
  uint32_t *a;

  /* Align to double word */

  if (addr & 0x7)
    {
      dbg("!!!Error not aligned @ 0x%08x [%lu]\n", addr, __LINE__);
      return -EINVAL;
    }

  if ((addr < STM32_FLASH_BASE) ||
      ((addr + bytes) > (STM32_FLASH_BASE + STM32_FLASH_SIZE)))
    {
      dbg("!!!Error  0x%08x..0x%08x out of range 0x%08x..0x%08x\n",
          addr, addr + bytes,
          STM32_FLASH_BASE, (STM32_FLASH_BASE + STM32_FLASH_SIZE));
      return -EFAULT;
    }

  stm32_flash_unlock();

  /* 1. Check that no Flash main memory operation is ongoing by checking    */
  /*    the BSY bit in the Flash status register (FLASH_SR).                */

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  /* 2. Check and clear all error programming flags due to a previous       */
  /*    programming. If not, PGSERR is set.                                 */

  flash_status = getreg32(STM32_FLASH_SR);
  if (flash_status)
    {
      modifyreg32(STM32_FLASH_SR, 0, flash_status);
    }

  /* 3. Set the PG bit in the Flash control register (FLASH_CR).            */

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);

  /* 4. Perform the data write operation at the desired memory address,     */
  /*    inside main memory block. Only double word can be programmed.       */
  /*    – Write a first word in an address aligned with double word         */
  /*    – Write the second word                                             */

  for (a = (uint32_t *)addr, w = 0; w < double_words * 2; a += 2, w += 2)
    {
      write_single_dword(a, &buf[w]);
    }

  /* see if we have any trailing bytes and write them */
  if (pad_bytes)
    {
      memset(pad, 0xff, sizeof(pad));
      memcpy(pad, (const void *)&buf[w], pad_bytes);
      write_single_dword(a, (uint32_t *)pad);
    }

  /* 5. Wait until the BSY bit is cleared in the FLASH_SR register.         */

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  /* 6. Since we are not explicitly using the EOP interrupt, check the      */
  /*    status for any errors.  Ignore the EOP bit since it would not       */
  /*    indicate an error if it were set.  Clear whatever is there          */
  /*    either way.                                                         */

  flash_status = getreg32(STM32_FLASH_SR);
  if (flash_status & ~FLASH_SR_EOP)
    {
      dbg("!!!Error flash status =0x%08x\n", addr, flash_status);
      bytes = -EIO;
    }
  modifyreg32(STM32_FLASH_SR, 0, flash_status);

  /* 7. Clear the PG bit in the FLASH_SR register                           */

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);

  stm32_flash_lock();

  return bytes;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
   return up_progmem_write_dword(addr, (const uint32_t *)buf, count);
}

#else

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  uint16_t *hword = (uint16_t *)buf;
  size_t written = count;

  /* STM32 requires half-word access */

  if (count & 1)
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if ((addr+count) >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  /* Get flash ready and begin flashing */

  if (!(getreg32(STM32_RCC_CR) & RCC_CR_HSION))
    {
      return -EPERM;
    }

  stm32_flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);

#if defined(CONFIG_STM32_STM32F40XX)
  /* TODO: implement up_progmem_write() to support other sizes than 16-bits */
  modifyreg32(STM32_FLASH_CR, FLASH_CR_PSIZE_MASK, FLASH_CR_PSIZE_X16);
#endif

  for (addr += STM32_FLASH_BASE; count; count-=2, hword++, addr+=2)
    {
      /* Write half-word and wait to complete */

      putreg16(*hword, addr);

      while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

      /* Verify */

      if (getreg32(STM32_FLASH_SR) & FLASH_SR_WRITE_PROTECTION_ERROR)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          return -EROFS;
        }

      if (getreg16(addr) != *hword)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          return -EIO;
        }
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
  return written;
}
#endif

#endif /* defined(CONFIG_STM32_STM32F10XX) || defined (CONFIG_STM32_STM32F40XX) */
