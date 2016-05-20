/****************************************************************************
 * drivers/syslog/ramlog.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
     Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/syslog/ramlog.h>

#include <arch/irq.h>

#ifdef CONFIG_RAMLOG

#ifdef CONFIG_RAMLOG_LAST_DMESG
#define __ramlog_dev       __attribute__((section(".ramlog_dev")))
#define __ramlog           __attribute__((section(".ramlog")))
#define RAMLOG_VALIDITY    (0x0F1E2D3C)
#else
#define __ramlog
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_LAST_DMESG
struct ramlog_ldev_s
{
  int               rl_validity;     /* Allows check if ramlog is valid */
  uint16_t          rl_head;         /* The head index (where data is added) */
  uint16_t          rl_tail;         /* The tail index (where data is removed) */
  size_t            rl_bufsize;      /* Size of the RAM buffer */
};
#endif

struct ramlog_dev_s
{
#ifdef CONFIG_RAMLOG_LAST_DMESG
  int               rl_validity;     /* Allows check if ramlog is valid */
#endif
#ifndef CONFIG_RAMLOG_NONBLOCKING
  volatile uint8_t  rl_nwaiters;     /* Number of threads waiting for data */
#endif
  volatile uint16_t rl_head;         /* The head index (where data is added) */
  volatile uint16_t rl_tail;         /* The tail index (where data is removed) */
  sem_t             rl_exclsem;      /* Enforces mutually exclusive access */
#ifndef CONFIG_RAMLOG_NONBLOCKING
  sem_t             rl_waitsem;      /* Used to wait for data */
#endif
  size_t            rl_bufsize;      /* Size of the RAM buffer */
  FAR char         *rl_buffer;       /* Circular RAM buffer */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *rl_fds[CONFIG_RAMLOG_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Helper functions */

#ifndef CONFIG_DISABLE_POLL
static void ramlog_pollnotify(FAR struct ramlog_dev_s *priv,
                              pollevent_t eventset);
#endif
static ssize_t ramlog_addchar(FAR struct ramlog_dev_s *priv, char ch);

/* Character driver methods */

static ssize_t ramlog_read(FAR struct file *, FAR char *, size_t);
static ssize_t ramlog_write(FAR struct file *, FAR const char *, size_t);
#ifndef CONFIG_DISABLE_POLL
static int     ramlog_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);
#endif
#ifdef CONFIG_RAMLOG_LAST_DMESG
static ssize_t ramlog_open_last(FAR struct file *);
static ssize_t ramlog_read_last(FAR struct file *, FAR char *, size_t);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ramlogfops =
{
  0,             /* open */
  0,             /* close */
  ramlog_read,   /* read */
  ramlog_write,  /* write */
  0,             /* seek */
  0              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , ramlog_poll  /* poll */
#endif
};

#ifdef CONFIG_RAMLOG_LAST_DMESG
static const struct file_operations g_lastramlogfops =
{
  /* Only the open and read operations are required */
  .open = ramlog_open_last,
  .read = ramlog_read_last,
};
#endif

/* This is the pre-allocated buffer used for the console RAM log and/or
 * for the syslogging function.
 */

#if defined(CONFIG_RAMLOG_CONSOLE) || defined(CONFIG_RAMLOG_SYSLOG)
static char g_sysbuffer[CONFIG_RAMLOG_BUFSIZE] __ramlog;

#ifdef CONFIG_RAMLOG_LAST_DMESG
static char g_syslastbuffer[CONFIG_RAMLOG_BUFSIZE] __ramlog;
static struct ramlog_ldev_s g_syslastdev __ramlog;
static struct ramlog_dev_s g_sysdev __ramlog_dev;
#else

/* This is the device structure for the console or syslogging function.  It
 * must be statically initialized because the RAMLOG syslog_putc function
 * could be called before the driver initialization logic executes.
 */

static struct ramlog_dev_s g_sysdev =
{
#ifndef CONFIG_RAMLOG_NONBLOCKING
  0,                             /* rl_nwaiters */
#endif
  0,                             /* rl_head */
  0,                             /* rl_tail */
  SEM_INITIALIZER(1),            /* rl_exclsem */
#ifndef CONFIG_RAMLOG_NONBLOCKING
  SEM_INITIALIZER(0),            /* rl_waitsem */
#endif
  CONFIG_RAMLOG_BUFSIZE,         /* rl_bufsize */
  g_sysbuffer                    /* rl_buffer */
};
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramlog_pollnotify
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static void ramlog_pollnotify(FAR struct ramlog_dev_s *priv,
            pollevent_t eventset)
{
  FAR struct pollfd *fds;
  irqstate_t flags;
  int i;

  /* This function may be called from an interrupt handler */

  for (i = 0; i < CONFIG_RAMLOG_NPOLLWAITERS; i++)
    {
      flags = irqsave();
      fds = priv->rl_fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & eventset);
          if (fds->revents != 0)
            {
              sem_post(fds->sem);
            }
        }
      irqrestore(flags);
    }
}
#else
#  define ramlog_pollnotify(priv,event)
#endif

/****************************************************************************
 * Name: ramlog_addchar
 ****************************************************************************/

static int ramlog_addchar(FAR struct ramlog_dev_s *priv, char ch)
{
  irqstate_t flags;
  int nexthead;

  /* Disable interrupts (in case we are NOT called from interrupt handler) */

  flags = irqsave();

  /* Calculate the write index AFTER the next byte is written */

  nexthead = priv->rl_head + 1;
  if (nexthead >= priv->rl_bufsize)
    {
      nexthead = 0;
    }

  /* Would the next write overflow the circular buffer? */

  if (nexthead == priv->rl_tail)
    {
#ifdef CONFIG_RAMLOG_TRULY_CIRCULAR
      /* Yes, so move tail to drop oldest byte from buffer. The semaphore
       * cannot be used here because we could be called from an interrupt
       * handler. However, this should be okay since interrupts are disabled
       * (in case we are NOT called from interrupt handler).
       */

      if (++priv->rl_tail >= priv->rl_bufsize)
        {
          priv->rl_tail = 0;
        }
#else
      /* Yes... Return an indication that nothing was saved in the buffer. */

      irqrestore(flags);
      return -EBUSY;
#endif
    }

  /* No... copy the byte and re-enable interrupts */

  priv->rl_buffer[priv->rl_head] = ch;
  priv->rl_head = nexthead;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: ramlog_read
 ****************************************************************************/

static ssize_t ramlog_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  struct inode *inode  = filep->f_inode;
  struct ramlog_dev_s *priv;
  ssize_t nread;
  char ch;
  int ret;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* If the circular buffer is empty, then wait for something to be written
   * to it.  This function may NOT be called from an interrupt handler.
   */

  DEBUGASSERT(!up_interrupt_context());

  /* Get exclusive access to the rl_tail index */

  ret = sem_wait(&priv->rl_exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Loop until something is read */

  for (nread = 0; nread < len; )
    {
      /* Get the next byte from the buffer */

      if (priv->rl_head == priv->rl_tail)
        {
          /* The circular buffer is empty. */

#ifdef CONFIG_RAMLOG_NONBLOCKING
          /* Return what we have (with zero mean the end-of-file) */

          break;
#else
          /* Did we read anything? */

          if (nread > 0)
            {
              /* Yes.. re-enable interrupts and the break out to return what
               * we have.
               */

              break;
            }

          /* If the driver was opened with O_NONBLOCK option, then don't wait.
           * Re-enable interrupts and return EGAIN.
           */

          if (filep->f_oflags & O_NONBLOCK)
            {
              nread = -EAGAIN;
              break;
            }

          /* Otherwise, wait for something to be written to the circular
           * buffer. Increment the number of waiters so that the ramlog_write()
           * will not that it needs to post the semaphore to wake us up.
           */

          sched_lock();
          priv->rl_nwaiters++;
          sem_post(&priv->rl_exclsem);

          /* We may now be pre-empted!  But that should be okay because we
           * have already incremented nwaiters.  Pre-emptions is disabled
           * but will be re-enabled while we are waiting.
           */

          ret = sem_wait(&priv->rl_waitsem);

          /* Interrupts will be disabled when we return.  So the decrementing
           * rl_nwaiters here is safe.
           */

          priv->rl_nwaiters--;
          sched_unlock();

          /* Did we successfully get the rl_waitsem? */

          if (ret >= 0)
            {
              /* Yes... then retake the mutual exclusion semaphore */

              ret = sem_wait(&priv->rl_exclsem);
            }

          /* Was the semaphore wait successful? Did we successful re-take the
           * mutual exclusion semaphore?
           */

          if (ret < 0)
            {
              /* No.. One of the two sem_wait's failed. */

              int errval = errno;

              /* Were we awakened by a signal?  Did we read anything before
               * we received the signal?
               */

              if (errval != EINTR || nread >= 0)
                {
                  /* Yes.. return the error. */

                  nread = -errval;
                }

              /* Break out to return what we have.  Note, we can't exactly
               * "break" out because whichever error occurred, we do not hold
               * the exclusion semaphore.
               */

              goto errout_without_sem;
            }
#endif /* CONFIG_RAMLOG_NONBLOCKING */
        }
      else
        {
          /* The circular buffer is not empty, get the next byte from the
           * tail index.
           */

          ch = priv->rl_buffer[priv->rl_tail];

          /* Increment the tail index and re-enable interrupts */

          if (++priv->rl_tail >= priv->rl_bufsize)
            {
              priv->rl_tail = 0;
            }

          /* Add the character to the user buffer */

          buffer[nread] = ch;
          nread++;
        }
    }

  /* Relinquish the mutual exclusion semaphore */

  sem_post(&priv->rl_exclsem);

  /* Notify all poll/select waiters that they can write to the FIFO */

#ifndef CONFIG_RAMLOG_NONBLOCKING
errout_without_sem:
#endif

#ifndef CONFIG_DISABLE_POLL
  if (nread > 0)
    {
      ramlog_pollnotify(priv, POLLOUT);
    }
#endif

  /* Return the number of characters actually read */

  return nread;
}

/****************************************************************************
 * Name: ramlog_write
 ****************************************************************************/

static ssize_t ramlog_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  struct inode *inode = filep->f_inode;
  struct ramlog_dev_s *priv;
  ssize_t nwritten;
  char ch;
  int ret;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

 /* Loop until all of the bytes have been written.  This function may be
  * called from an interrupt handler!  Semaphores cannot be used!
  *
  * The write logic only needs to modify the rl_head index.  Therefore,
  * there is a difference in the way that rl_head and rl_tail are protected:
  * rl_tail is protected with a semaphore; rl_tail is protected by disabling
  * interrupts.
  */

  for (nwritten = 0; nwritten < len; nwritten++)
    {
      /* Get the next character to output */

      ch = buffer[nwritten];

      /* Ignore carriage returns */

#ifdef CONFIG_RAMLOG_CRLF
      if (ch == '\r')
        {
          continue;
        }

      /* Pre-pend a carriage before a linefeed */

      if (ch == '\n')
        {
          ret = ramlog_addchar(priv, '\r');
          if (ret < 0)
            {
              /* The buffer is full and nothing was saved. Break out of the
               * loop to return the number of bytes written up to this point.
               * The data to be written is dropped on the floor.
               */

              break;
            }
        }
#endif

      /* Then output the character */

      ret = ramlog_addchar(priv,ch);
      if (ret < 0)
        {
          /* The buffer is full and nothing was saved. Break out of the
           * loop to return the number of bytes written up to this point.
           * The data to be written is dropped on the floor.
           */

          break;
        }
    }

  /* Was anything written? */

#if !defined(CONFIG_RAMLOG_NONBLOCKING) || !defined(CONFIG_DISABLE_POLL)
  if (nwritten > 0)
    {
      irqstate_t flags;
#ifndef CONFIG_RAMLOG_NONBLOCKING
      int i;
#endif

      /* Are there threads waiting for read data? */

      flags = irqsave();
#ifndef CONFIG_RAMLOG_NONBLOCKING
      for (i = 0; i < priv->rl_nwaiters; i++)
        {
          /* Yes.. Notify all of the waiting readers that more data is available */

          sem_post(&priv->rl_waitsem);
        }
#endif

      /* Notify all poll/select waiters that they can write to the FIFO */

      ramlog_pollnotify(priv, POLLIN);
      irqrestore(flags);
    }
#endif

  /* We always have to return the number of bytes requested and NOT the
   * number of bytes that were actually written.  Otherwise, callers
   * will think that this is a short write and probably retry (causing
   */

  return len;
}

/****************************************************************************
 * Name: ramlog_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
int ramlog_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv;
  pollevent_t eventset;
  int ndx;
  int ret;
  int i;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Get exclusive access to the poll structures */

  ret = sem_wait(&priv->rl_exclsem);
  if (ret < 0)
    {
      int errval = errno;
      return -errval;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_RAMLOG_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->rl_fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->rl_fds[i] = fds;
              fds->priv       = &priv->rl_fds[i];
              break;
            }
        }

      if (i >= CONFIG_RAMLOG_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should immediately notify on any of the requested events?
       * First, check if the xmit buffer is full.
       */

      eventset = 0;

      ndx = priv->rl_head + 1;
      if (ndx >= priv->rl_bufsize)
        {
          ndx = 0;
        }

      if (ndx != priv->rl_tail)
       {
         eventset |= POLLOUT;
       }

      /* Check if the receive buffer is empty */

      if (priv->rl_head != priv->rl_tail)
       {
         eventset |= POLLIN;
       }

      if (eventset)
        {
          ramlog_pollnotify(priv, eventset);
        }

    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG
      if (!slot)
        {
          ret = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot      = NULL;
      fds->priv  = NULL;
    }

errout:
  sem_post(&priv->rl_exclsem);
  return ret;
}
#endif

/****************************************************************************
 * Name: ramlog_open_last
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_LAST_DMESG
static int ramlog_open_last(FAR struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct ramlog_ldev_s *priv;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  filep->f_pos = priv->rl_tail;
  return OK;
}
#endif

/****************************************************************************
 * Name: ramlog_read_last
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_LAST_DMESG
static ssize_t ramlog_read_last(FAR struct file *filep, FAR char *buffer,
                size_t len)
{
  struct inode *inode = filep->f_inode;
  struct ramlog_ldev_s *priv;
  ssize_t nread;
  char ch;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Loop until nothing more to read */

  for (nread = 0; nread < len; )
    {
      /* Get the next byte from the buffer */

      if (priv->rl_head == filep->f_pos)
        {
          /* The circular buffer is empty. */

          break;
        }
      else
        {
          /* The circular buffer is not empty, get the next byte from the
           * position index.
           */

          ch = g_syslastbuffer[filep->f_pos];

          /* Increment the position index */

          if (++filep->f_pos >= priv->rl_bufsize)
            {
              filep->f_pos = 0;
            }

          /* Add the character to the user buffer */

          buffer[nread++] = ch;
        }
    }

  /* Return the number of characters actually read */

  return nread;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramlog_register
 *
 * Description:
 *   Create the RAM logging device and register it at the specified path.
 *
 ****************************************************************************/

#if !defined(CONFIG_RAMLOG_CONSOLE) && !defined(CONFIG_RAMLOG_SYSLOG)
int ramlog_register(FAR const char *devpath, FAR char *buffer, size_t buflen)
{
  FAR struct ramlog_dev_s *priv;
  int ret = -ENOMEM;

  /* Sanity checking */

  DEBUGASSERT(devpath && buffer && buflen > 1);

  /* Allocate a RAM logging device structure */

  priv = (struct ramlog_dev_s *)kmm_zalloc(sizeof(struct ramlog_dev_s));
  if (priv)
    {
      /* Initialize the non-zero values in the RAM logging device structure */

      sem_init(&priv->rl_exclsem, 0, 1);
#ifndef CONFIG_RAMLOG_NONBLOCKING
      sem_init(&priv->rl_waitsem, 0, 0);
#endif
      priv->rl_bufsize = buflen;
      priv->rl_buffer  = buffer;

      /* Register the character driver */

      ret = register_driver(devpath, &g_ramlogfops, 0666, priv);
      if (ret < 0)
        {
          kmm_free(priv);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: ramlog_consoleinit
 *
 * Description:
 *   Use a pre-allocated RAM logging device and register it at /dev/console
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_CONSOLE
int ramlog_consoleinit(void)
{
  FAR struct ramlog_dev_s *priv = &g_sysdev;

  /* Register the console character driver */

  return register_driver("/dev/console", &g_ramlogfops, 0666, priv);
}
#endif

/****************************************************************************
 * Name: ramlog_sysloginit
 *
 * Description:
 *   Use a pre-allocated RAM logging device and register it at the path
 *   specified by CONFIG_RAMLOG_SYSLOG
 *
 *   If CONFIG_RAMLOG_CONSOLE is also defined, then this functionality is
 *   performed when ramlog_consoleinit() is called.
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
int ramlog_sysloginit(void)
{
#ifdef CONFIG_RAMLOG_LAST_DMESG
  /* If last ramlog is valid, register the last syslog character driver */

  if (g_syslastdev.rl_validity == RAMLOG_VALIDITY)
    {
      register_driver(CONFIG_SYSLOG_LAST_DEVPATH, &g_lastramlogfops, 0666,
                      &g_syslastdev);
    }
#endif

  /* Register the syslog character driver */

  return register_driver(CONFIG_SYSLOG_DEVPATH, &g_ramlogfops, 0666, &g_sysdev);
}
#endif

/****************************************************************************
 * Name: ramlog_preserve_last
 *
 * Description:
 *   Called very early on boot (before data section is initialized and bss
 *   section is cleared). If the last ramlog buffer is still intact in RAM,
 *   it will be saved so it can be read later.
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_LAST_DMESG
void ramlog_preserve_last(void)
{
  FAR struct ramlog_dev_s *priv = &g_sysdev;

  /* Clear out the last ramlog buffer and structure*/
  memset(g_syslastbuffer, 0, CONFIG_RAMLOG_BUFSIZE);
  memset(&g_syslastdev, 0, sizeof(struct ramlog_ldev_s));

  /* Check if ramlog from previous boot is still in RAM */
  if (priv->rl_validity == RAMLOG_VALIDITY)
    {
      /* Copy previous ramlog */

      g_syslastdev.rl_validity = priv->rl_validity;
      g_syslastdev.rl_head = priv->rl_head;
      g_syslastdev.rl_tail = priv->rl_tail;
      g_syslastdev.rl_bufsize = priv->rl_bufsize;
      memcpy(g_syslastbuffer, priv->rl_buffer, priv->rl_bufsize);
    }

  /* Must initialize g_sysdev here since it is not statically initialized */
  priv->rl_validity = RAMLOG_VALIDITY;
  priv->rl_head = 0;
  priv->rl_tail = 0;
  sem_init(&priv->rl_exclsem, 0, 1);
  priv->rl_bufsize = CONFIG_RAMLOG_BUFSIZE;
  priv->rl_buffer = g_sysbuffer;
#ifndef CONFIG_RAMLOG_NONBLOCKING
  priv->rl_nwaiters = 0;
  sem_init(&priv->rl_waitsem, 0, 0);
#endif
}
#endif

/****************************************************************************
 * Name: syslog_putc
 *
 * Description:
 *   This is the low-level system logging interface.  The debugging/syslogging
 *   interfaces are syslog() and lowsyslog().  The difference is that
 *   the syslog() internface writes to fd=1 (stdout) whereas lowsyslog() uses
 *   a lower level interface that works from interrupt handlers.  This
 *   function is a a low-level interface used to implement lowsyslog()
 *   when CONFIG_RAMLOG_SYSLOG=y and CONFIG_SYSLOG=y
 *
 ****************************************************************************/

#if defined(CONFIG_RAMLOG_CONSOLE) || defined(CONFIG_RAMLOG_SYSLOG)
int syslog_putc(int ch)
{
  FAR struct ramlog_dev_s *priv = &g_sysdev;
  int ret;

#ifdef CONFIG_RAMLOG_CRLF
  /* Ignore carriage returns.  But return success. */

  if (ch == '\r')
    {
      return ch;
    }

  /* Pre-pend a newline with a carriage return */

  if (ch == '\n')
    {
      ret = ramlog_addchar(priv, '\r');
      if (ret < 0)
        {
          /* The buffer is full and nothing was saved. */

          goto errout;
        }
    }
#endif

  /* Add the character to the RAMLOG */

  ret = ramlog_addchar(priv, ch);
  if (ret >= 0)
    {
      /* Return the character added on success */

      return ch;
    }

  /* On a failure, we need to return EOF and set the errno so that
   * work like all other putc-like functions.
   */

errout:
  set_errno(-ret);
  return EOF;
}
#endif

#endif /* CONFIG_RAMLOG */
