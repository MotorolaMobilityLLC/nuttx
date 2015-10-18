############################################################################
# apps/examples/Makefile
#
#   Copyright (C) 2011-2014 Gregory Nutt. All rights reserved.
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
#
############################################################################

-include $(TOPDIR)/.config # Current configuration

# Sub-directories

# Keep this list sorted alphabetically, adding new apps in the right place.
SUBDIRS  = apbridge
SUBDIRS += battery
SUBDIRS += arapm
SUBDIRS += bringup_entry
SUBDIRS += debug
SUBDIRS += dev_info
SUBDIRS += etm
SUBDIRS += gb_loopback
SUBDIRS += gb_tape
SUBDIRS += gpbridge
SUBDIRS += gpio
SUBDIRS += i2s
SUBDIRS += pwm
SUBDIRS += pwm_unit_test
SUBDIRS += sdio_unit_test
SUBDIRS += service_mgr
SUBDIRS += spi
SUBDIRS += springpm
SUBDIRS += svc
SUBDIRS += svc_power
SUBDIRS += time
SUBDIRS += unipro
SUBDIRS += usb-host

# Sub-directories that might need context setup.  Directories may need
# context setup for a variety of reasons, but the most common is because
# the example may be built as an NSH built-in function.

ifeq ($(CONFIG_NSH_BUILTIN_APPS),y)
# Keep this list sorted alphabetically, adding new apps in the right place.
CNTXTDIRS += battery
CNTXTDIRS += arapm
CNTXTDIRS += bringup_entry
CNTXTDIRS += debug
CNTXTDIRS += dev_info
CNTXTDIRS += etm
CNTXTDIRS += gb_loopback
CNTXTDIRS += gb-tape
CNTXTDIRS += gpio
CNTXTDIRS += i2s
CNTXTDIRS += pwm
CNTXTDIRS += pwm_unit_test
CNTXTDIRS += sdio_unit_test
CNTXTDIRS += service_mgr
CNTXTDIRS += spi
CNTXTDIRS += springpm
CNTXTDIRS += svc
CNTXTDIRS += svc_power
CNTXTDIRS += time
CNTXTDIRS += unipro
CNTXTDIRS += usb-host
endif

all: nothing

.PHONY: nothing context depend clean distclean

define SDIR_template
$(1)_$(2):
	$(Q) $(MAKE) -C $(1) $(2) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)"
endef

$(foreach SDIR, $(CNTXTDIRS), $(eval $(call SDIR_template,$(SDIR),context)))
$(foreach SDIR, $(SUBDIRS), $(eval $(call SDIR_template,$(SDIR),depend)))
$(foreach SDIR, $(SUBDIRS), $(eval $(call SDIR_template,$(SDIR),clean)))
$(foreach SDIR, $(SUBDIRS), $(eval $(call SDIR_template,$(SDIR),distclean)))

nothing:

install:

context: $(foreach SDIR, $(CNTXTDIRS), $(SDIR)_context)

depend: $(foreach SDIR, $(SUBDIRS), $(SDIR)_depend)

clean: $(foreach SDIR, $(SUBDIRS), $(SDIR)_clean)

distclean: $(foreach SDIR, $(SUBDIRS), $(SDIR)_distclean)

-include Make.dep
