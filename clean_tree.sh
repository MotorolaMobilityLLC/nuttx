#!/bin/bash
# clean_tree.sh
#
# Copyright (c) 2015 Google, Inc.
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

# remove symlinks and leftovers

echo "Cleaning up"

if [ "$#" -ne 1 ]; then
   # determine NuttX top level folder absolute path
   TOPDIR="`dirname \"$0\"`"  # relative
   TOPDIR="`( cd \"$TOPDIR/nuttx\" && pwd )`"  # absolutized and normalized
else
   TOPDIR=$1
fi
rm -f ${TOPDIR}/arch/arm/src/board >/dev/null 2>&1
rm -f ${TOPDIR}/arch/arm/src/chip >/dev/null 2>&1
rm -f ${TOPDIR}/arch/arm/include/board >/dev/null 2>&1
rm -f ${TOPDIR}/arch/arm/include/chip >/dev/null 2>&1
rm -f ${TOPDIR}/include/apps >/dev/null 2>&1
rm -f ${TOPDIR}/include/arch >/dev/null 2>&1
rm -f ${TOPDIR}/include/nuttx/config.h >/dev/null 2>&1
rm -f ${TOPDIR}/include/nuttx/version.h >/dev/null 2>&1
rm -f ${TOPDIR}/tools/mkconfig >/dev/null 2>&1
rm -f ${TOPDIR}/tools/mkdeps >/dev/null 2>&1
rm -f ${TOPDIR}/tools/mkversion >/dev/null 2>&1
rm -f ${TOPDIR}/.config  >/dev/null 2>&1
rm -f ${TOPDIR}/Make.defs >/dev/null 2>&1
rm -f ${TOPDIR}/setenv.h >/dev/null 2>&1
rm -f ${TOPDIR}/Make.defs >/dev/null 2>&1
rm -f ${TOPDIR}/setenv.sh >/dev/null 2>&1
rm -f ${TOPDIR}/setenv.bat >/dev/null 2>&1
rm -f ${TOPDIR}/.config >/dev/null 2>&1
rm -f ${TOPDIR}/.config.old >/dev/null 2>&1
rm -f ${TOPDIR}/.version >/dev/null 2>&1
rm -f ${TOPDIR}/nuttx >/dev/null 2>&1
rm -f ${TOPDIR}/nuttx.bin >/dev/null 2>&1
rm -f ${TOPDIR}/System.map >/dev/null 2>&1
rm -f ${TOPDIR}/../apps/platform/board 2>&1
rm -f ${TOPDIR}/../apps/builtin/builtin_list.h 2>&1
rm -f ${TOPDIR}/../apps/builtin/builtin_proto.h 2>&1
find ./ -iname "*.o" -type f -delete >/dev/null 2>&1
find ./ -iname "*.a" -type f -delete >/dev/null 2>&1
find ./ -iname ".depend" -type f -delete >/dev/null 2>&1
find ./ -iname ".built" -type f -delete >/dev/null 2>&1
find ./ -iname ".updated" -type f -delete >/dev/null 2>&1
find ./ -iname "Make.dep" -type f -delete >/dev/null 2>&1
find ./ -iname "Make*.dep" -type f -delete >/dev/null 2>&1
