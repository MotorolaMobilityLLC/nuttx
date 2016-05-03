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

ifeq ($(CONFIG_GREYBUS_UTILS),y)
CSRCS += manifest.c
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

ifeq ($(CONFIG_MANIFEST_ALL), y)
 MANIFEST = all-modules
endif

ifeq ($(CONFIG_CUSTOM_MANIFEST), y)
 MANIFEST = $(subst ",,$(CONFIG_MANIFEST))
endif

ifeq ($(CONFIG_OOT_MANIFEST), y)
 ifndef OOT_MANIFEST
  $(error "out-of-tree manifest selected in Kconfig but global OOT_MANIFEST variable is undefined")
 endif

 MANIFEST = $(OOT_MANIFEST)
 MANIFESTS = $(subst .mnfs,,$(OOT_MANIFEST))
endif

ifdef EXT_MANIFEST_INC
 ifneq ("$(wildcard $(EXT_MANIFEST_INC))","")
  $(info "using externally signed manifest " $(EXT_MANIFEST_INC))
  USE_EXT_MANIFEST_INC = y
 else
  $(error "EXT_MANIFEST_INC path invalid " $(EXT_MANIFEST_INC))
 endif
else
  USE_EXT_MANIFEST_INC = n
endif

MANIFESTS ?= $(addprefix manifests/, $(MANIFEST))
MNFS = $(addsuffix .mnfs, $(MANIFESTS))
MNFB = $(addsuffix .mnfb, $(MANIFESTS))
MNSG = $(addsuffix .mnsg, $(MANIFESTS))

ALL_MNFS = $(wildcard manifests/*.mnfs) $(OOT_MANIFEST)
ALL_MNFB = $(subst .mnfs,.mnfb,$(ALL_MNFS))
ALL_MNSG = $(subst .mnfs,.mnsg,$(ALL_MNFS))

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

ifeq ($(USE_EXT_MANIFEST_INC), y)
manifest.inc: $(EXT_MANIFEST_INC)
		cp $(EXT_MANIFEST_INC) $@
else
%.mnfb: %.mnfs
	manifesto $< $(CONFIG_ARCH_BOARDID_VID) $(CONFIG_ARCH_BOARDID_PID)

%.mnsg: %.mnfb
	openssl dgst -sha256 -sign testkey_priv.key -out $@ $<

ifneq ($(MANIFESTS),)
manifest.inc: $(MNFB) $(MNSG)
	@echo manifest.inc : $(MNFB) $(MANIFESTS)
	xxd -i < $(MNFB) > $@
	echo "," >> $@
	xxd -i < $(MNSG) >> $@
else
manifest.inc: ;
endif

endif

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

install:

context:
	@true

.depend: Makefile $(SRCS) manifest.inc
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend
	@true

clean:
	$(call DELFILE, .built)
	$(call CLEAN)
	$(call DELFILE, manifest.inc $(ALL_MNFB))
	$(call DELFILE, $(ALL_MNSG))

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

