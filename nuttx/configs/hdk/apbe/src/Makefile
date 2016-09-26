-include $(TOPDIR)/Make.defs

CFLAGS		+= -I$(TOPDIR)/sched

ASRCS		=
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= up_boot.c

ifeq ($(CONFIG_NSH_ARCHINIT),y)
endif

ifeq ($(CONFIG_INPUT),y)
endif

ifeq ($(CONFIG_USBMSC),y)
endif

ifeq ($(CONFIG_WATCHDOG),y)
	CSRCS		+= up_watchdog.c
endif

ifeq ($(CONFIG_MODS_USB_HCD_ROUTER),y)
	CSRCS		+= dummy_hsic.c
endif

COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

ARCH_SRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(WINTOOL),y)
	CFLAGS	+= -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}" \
		   -I "${shell cygpath -w $(ARCH_SRCDIR)/common}" \
		   -I "${shell cygpath -w $(ARCH_SRCDIR)/armv7-m}"
	else
	CFLAGS	+= -I$(ARCH_SRCDIR)/chip -I$(ARCH_SRCDIR)/common -I$(ARCH_SRCDIR)/armv7-m
endif

all: libboard$(LIBEXT)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libboard$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, libboard$(LIBEXT))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
