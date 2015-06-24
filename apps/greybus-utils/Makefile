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

ifeq ($(CONFIG_GREYBUS_UTILS),y)

ifeq ($(CONFIG_SVC_MSG),y)
CSRCS += svc_msg.c
endif

CSRCS += manifest.c

endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ifeq ($(CONFIG_MANIFEST_ALL), y)
 MANIFEST = all-modules
endif

ifeq ($(CONFIG_DST_DEVID), )
CONFIG_DST_DEVID := 1
endif

ifeq ($(CONFIG_CUSTOM_MANIFEST), y)
 MANIFEST = $(subst ",,$(CONFIG_MANIFEST))
else
 MANIFEST_COPY := $(MANIFEST)
 MANIFEST = $(CONFIG_DST_DEVID),$(MANIFEST_COPY)
endif

ifdef MANIFEST_REQUIRED
 ifndef MANIFEST
  $(error MANIFEST unspecified or does not exist!)
 endif
endif

comma:= ,
empty:=
space:= $(empty) $(empty)
semicolon:= ;

DEVID_LIST:= 1 2 3 4 5 8

MANIFEST_TMP = $(subst -,_,$(MANIFEST))
MANIFEST_TMP2 = $(subst $(semicolon),$(space),$(MANIFEST_TMP))
MANIFEST_TMP3 = $(subst $(comma),$(comma)manifests_, $(MANIFEST_TMP2))
MANIFEST_TMP4 = $(addsuffix _mnfb}$(comma), $(MANIFEST_TMP3))
MANIFEST_DEFINE = $(addprefix {, $(MANIFEST_TMP4))
MANIFEST_DEVID_LIST = $(subst $(semicolon),$(space),$(MANIFEST))
MANIFEST_DEVID_LIST2 = $(subst $(comma),$(space),$(MANIFEST_DEVID_LIST))
MANIFEST_LIST = $(filter-out $(DEVID_LIST), $(MANIFEST_DEVID_LIST2))
MANIFESTS = $(addprefix manifests/, $(MANIFEST_LIST))
MNFS = $(addsuffix .mnfs, $(MANIFESTS))
MNFB = $(addsuffix .mnfb, $(MANIFESTS))
MNFC = $(addsuffix .c, $(MANIFESTS))
MNFC_INCLUDES = $(addprefix -include , $(MNFC))
ALL_MNFS = $(wildcard manifests/*.mnfs)
ALL_MNFB = $(subst .mnfs,.mnfb,$(ALL_MNFS))
ALL_MNFC = $(subst .mnfs,.c,$(ALL_MNFS))

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
	@true

.PHONY: context .depend depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

%.mnfb: %.mnfs
	manifesto $<

%.c: %.mnfb
	xxd -i $< > $@

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

install:

context:
	@true

manifest.o: $(MNFC) manifest.c
	$(call COMPILE, -DMANIFEST="$(MANIFEST_DEFINE)" $(MNFC_INCLUDES) manifest.c, manifest.o)

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend
	@true

clean:
	$(call DELFILE, .built)
	$(call CLEAN)
	$(call DELFILE, $(ALL_MNFC) $(ALL_MNFB))

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

