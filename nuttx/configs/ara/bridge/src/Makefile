#
# Copyright (c) 2015 Google Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

-include $(TOPDIR)/Make.defs
include $(TOPDIR)/configs/ara/common/Make.defs

CFLAGS		+= -I$(TOPDIR)/sched -I$(ARACOMMON_INCLUDE)

ASRCS		=
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= up_boot.c
CSRCS		+= board-ara.c

ifeq ($(CONFIG_ARA_BRIDGE_BOARD_ARA_DEVBOARD),y)
CSRCS		+= board-ara-devboard.c
endif

ifeq ($(CONFIG_APB_AP),y)
CSRCS   += csi_tx_init.c
CSRCS   += dsi_rx_init.c
else ifeq ($(CONFIG_APB_CAMERA),y)
CSRCS   += csi_rx_init.c ov5645.c
else ifeq ($(CONFIG_APB_DISPLAY),y)
CSRCS   += lg4892.c
CSRCS   += dsi_tx_init.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS		+= up_pwm.c
endif


ifeq ($(CONFIG_ARA_BRIDGE_HAVE_USB4624),y)
CSRCS		+= usb4624.c
endif

ifeq ($(CONFIG_ARA_BRIDGE_HAVE_USB3813),y)
CSRCS		+= usb3813.c
endif

ifeq ($(CONFIG_ARA_BRIDGE_HAVE_HID_TOUCH),y)
CSRCS		+= hid_dummy_touch.c
endif

ifeq ($(CONFIG_ARA_BRIDGE_HAVE_LIGHTS),y)
CSRCS		+= lights_dummy.c
endif

ifeq ($(CONFIG_APBRIDGEA),y)
CSRCS += common_gadget.c
CSRCS += apbridgea_gadget.c
endif

ifeq ($(CONFIG_ARA_BRIDGE_HAVE_BATTERY),y)
CSRCS += battery_dummy.c
endif

ifeq ($(CONFIG_ARCH_CHIP_DEVICE_SDIO),y)
CSRCS += sdio_board.c
endif

ifeq ($(CONFIG_ARA_BRIDGE_BOARD_OOT),y)
CSRCS += $(OOT_BOARD)
endif


COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

ARCH_SRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
CFLAGS          += -I$(ARCH_SRCDIR)/chip -I$(ARCH_SRCDIR)/common -I$(ARCH_SRCDIR)/armv7-m

all: $(ARACOMMON_LIB) libboard$(LIBEXT)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libboard$(LIBEXT): $(OBJS) $(ARACOMMON_LIB)
	$(call ARACOMMON_LINK, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, libboard$(LIBEXT))
	$(call CLEAN)
	$(MAKE) -C $(ARACOMMON_SRC) clean

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(MAKE) -C $(ARACOMMON_SRC) distclean

-include Make.dep
