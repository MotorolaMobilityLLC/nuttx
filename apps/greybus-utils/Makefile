############################################################################
# apps/nshlib/Makefile
#
#   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#   Copyright (c) 2014, 2015 Google Inc.
#   All rights reserved.
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
############################################################################

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# NSH Library

ifeq ($(CONFIG_SVC_MSG),y)
CSRCS += svc_msg.c
endif

CSRCS += manifest.c

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ifeq ($(CONFIG_MANIFEST_GPIO), y)
 MANIFEST = simple-gpio-module
endif

ifeq ($(CONFIG_MANIFEST_I2C), y)
 MANIFEST = simple-i2c-module
endif

ifeq ($(CONFIG_MANIFEST_GPIO_I2C), y)
 MANIFEST = simple-gpio-i2c-module
endif

ifeq ($(CONFIG_MANIFEST_BATTERY_GPIO_I2C), y)
 MANIFEST = simple-battery-gpio-i2c-module
endif

ifeq ($(CONFIG_MANIFEST_LOOPBACK), y)
 MANIFEST = simple-loopback-module
endif

ifeq ($(CONFIG_MANIFEST_CONSOLE), y)
 MANIFEST = simple-uart-module
endif

ifeq ($(CONFIG_CUSTOM_MANIFEST), y)
 MANIFEST = $(subst ",,$(CONFIG_MANIFEST))
endif

comma:= ,
empty:=
space:= $(empty) $(empty)

MANIFEST_TMP = $(subst -,_,$(MANIFEST))
MANIFEST_TMP2 = $(addsuffix _mnfb, $(MANIFEST_TMP))
MANIFEST_DEFINE = $(addprefix manifests_, $(MANIFEST_TMP2))
MANIFEST_LIST = $(subst $(comma),$(space),$(MANIFEST))
MANIFESTS = $(addprefix manifests/, $(MANIFEST_LIST))
MNFS = $(addsuffix .mnfs, $(MANIFESTS))
MNFB = $(addsuffix .mnfb, $(MANIFESTS))
MNFC = $(addsuffix .c, $(MANIFESTS))
MNFC_INCLUDES = $(addprefix -include , $(MNFC))
$(warning $(MANIFEST))
$(warning $(MNFS))

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN		= ..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN		= ..\\libapps$(LIBEXT)
else
  BIN		= ../libapps$(LIBEXT)
endif
endif

ROOTDEPPATH = --dep-path .
VPATH =

# Build targets

all: .built
.PHONY: context .depend depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(MNFB): $(MNFS)
	$(shell manifesto $<)

$(MNFC): $(MNFB)
	$(shell xxd -i $< > $@)

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

install:

context:

manifest.o: $(MNFC) manifest.c
	$(call COMPILE, -DMANIFEST="$(MANIFEST_DEFINE)" $(MNFC_INCLUDES) manifest.c, manifest.o)

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

