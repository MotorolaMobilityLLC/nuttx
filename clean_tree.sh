#!/bin/bash
# clean_tree.sh

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
