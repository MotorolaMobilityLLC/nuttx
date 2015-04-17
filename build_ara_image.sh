#!/bin/bash
# build_ara_image.sh
#
# ---------------------------------------------------------------
# 2015-03-25 - darryln - adapted from nuttx/tools/configure.sh
# ---------------------------------------------------------------

# define exit error codes
ARA_BUILD_CONFIG_ERR_BAD_PARAMS=1
ARA_BUILD_CONFIG_ERR_NO_NUTTX_TOPDIR=2
ARA_BUILD_CONFIG_ERR_CONFIG_NOT_FOUND=3
ARA_BUILD_CONFIG_ERR_CANT_CREATE_OUTPUT=4
ARA_BUILD_CONFIG_ERR_CONFIG_COPY_FAILED=5

echo "Project Ara firmware image builder"

USAGE="

USAGE: ${0} <board-name> <config-name>

Where:
  <board-name> is the name of the board in the configs directory
  <config-name> is the name of the board configuration sub-directory

"

# validate parameters for board & image are present
if [ "$#" -ne 2 ] ; then
  echo "Required parameters not specified."
  echo "$USAGE"
  exit $ARA_BUILD_CONFIG_ERR_BAD_PARAMS
fi

board=$1
image=$2

ARA_MAKE_BUILD_NAME_UNIQUE=0
ARA_USE_SPINNER=0

# determine NuttX top level folder absolute path
TOPDIR="`dirname \"$0\"`"  # relative
TOPDIR="`( cd \"$TOPDIR/nuttx\" && pwd )`"  # absolutized and normalized
if [ -z "$TOPDIR" ] ; then
  # error; for some reason, the path is not accessible
  # to the script (e.g. permissions)
  echo "Can't determine NuttX top level folder, fatal."
  exit $ARA_BUILD_CONFIG_ERR_NO_NUTTX_TOPDIR
fi

# set path to image config
configpath=${TOPDIR}/configs/${board}/${image}
if [ ! -d "${configpath}" ]; then
  echo "Build config '${configpath}' does not exist"
  exit $ARA_BUILD_CONFIG_ERR_CONFIG_NOT_FOUND
fi

# set build output path
buildbase="`( cd \"$TOPDIR/..\" && pwd )`/build"

# create build name from config
# substitute "-" for "/"
board=$(echo "$board" | sed -e "s#/#-#")
image=$(echo "$image" | sed -e "s#/#-#")

buildname=${board}-${image}
# uniquify build name with time stamp
if [ $ARA_MAKE_BUILD_NAME_UNIQUE == 1 ]; then
  buildname=${buildname}-`date +"%Y%m%d-%H%M%S"`
fi

# full path to defconfig file
defconfigFile="${configpath}/defconfig"

echo "Build config file   : $defconfigFile"
echo "Build name          : '$buildname'"

# define paths used during build process
ARA_BUILD_CONFIG_PATH="$buildbase/$buildname/config"
ARA_BUILD_IMAGE_PATH="$buildbase/$buildname/image"
ARA_BUILD_TOPDIR="$buildbase/$buildname"

echo "Build output folder : $ARA_BUILD_TOPDIR"
echo "Image output folder : $ARA_BUILD_IMAGE_PATH"

# delete build tree if it already exists
if [ -d $ARA_BUILD_TOPDIR ] ; then
   rm -rf $ARA_BUILD_TOPDIR
fi

# create folder structure in build output tree
mkdir -p "$ARA_BUILD_CONFIG_PATH"
mkdir -p "$ARA_BUILD_IMAGE_PATH"
mkdir -p "$ARA_BUILD_TOPDIR"

# Copy nuttx tree to build tree
cp -r ./nuttx $ARA_BUILD_TOPDIR/nuttx
cp -r ./apps $ARA_BUILD_TOPDIR/apps
cp -r ./misc $ARA_BUILD_TOPDIR/misc
cp -r ./NxWidgets $ARA_BUILD_TOPDIR/NxWidgets

# copy Make.defs to build output tree
if ! install -m 644 -p ${configpath}/Make.defs ${ARA_BUILD_TOPDIR}/nuttx/Make.defs ; then
    echo "Failed to copy Make.defs"
#    exit $ARA_BUILD_CONFIG_ERR_CONFIG_COPY_FAILED
fi

# copy setenv.sh to build output tree
if ! install -p ${configpath}/setenv.sh ${ARA_BUILD_TOPDIR}/nuttx/setenv.sh ; then
  echo "Failed to copy setenv.sh"
#  exit $ARA_BUILD_CONFIG_ERR_CONFIG_COPY_FAILED
else
 chmod 755 "${ARA_BUILD_TOPDIR}/nuttx/setenv.sh"
fi

# copy defconfig to build output tree
if ! install -m 644 -p ${defconfigFile} ${ARA_BUILD_TOPDIR}/nuttx/.config ; then
    echo "Failed to copy defconfig"
#    exit $ARA_BUILD_CONFIG_ERR_CONFIG_COPY_FAILED
fi

# save config files
cp ${ARA_BUILD_TOPDIR}/nuttx/.config   ${ARA_BUILD_CONFIG_PATH}/.config
cp ${ARA_BUILD_TOPDIR}/nuttx/Make.defs ${ARA_BUILD_CONFIG_PATH}/Make.defs
cp ${ARA_BUILD_TOPDIR}/nuttx/setenv.sh  ${ARA_BUILD_CONFIG_PATH}/setenv.sh

#echo "Build configured"
MAKE_RESULT=1

build_image() {
  echo -n "Building '$buildname'" ...
  local MAKELOG="$ARA_BUILD_TOPDIR/build.log"
  #loca lMAKECMD=" --always-make -f Makefile.unix"
  #local MAKECMD=" V=2 --always-make --debug=v -p  -f Makefile.unix"
  local MAKECMD=" --always-make V=2 --debug -r -f Makefile.unix"

  pushd $ARA_BUILD_TOPDIR/nuttx > /dev/null
  make $MAKECMD > $MAKELOG 2>&1 &
  local pid=$! # get pid of make

  if [ $ARA_USE_SPINNER -eq 1 ] ; then
    #### spinner progress indicator
    local spinstr='|/-\\'
    tput civis; # hide cursor
    #while [ "$(ps a | awk '{print $1}' | grep $pid)" ]; do
    while [ -d /proc/$pid ] ; do
      local temp=${spinstr#?}
      printf " %c  " "$spinstr"
      spinstr=$temp${spinstr%"$temp"}
      sleep 0.1
      printf "\b\b\b\b"
    done
    printf "\b\b\b     "
    echo
    tput cnorm; # show cursor
    #### end of spinner progress indicator
  fi

  wait $pid
  if [ $? -eq 0 ] ; then
    MAKE_RESULT=1
  else
    MAKE_RESULT=0
  fi
  popd > /dev/null

}

clean_build() {
   # for each file found in the source tree, delete the one in the build tree
  echo "Cleaning build tree"
  outpath=$ARA_BUILD_TOPDIR
  srcpath=$(readlink -f $(dirname "$0"))
  (cd $srcpath; find -P . -type f) | sort > src_files
  (cd $outpath; find -P . -type f) | sort > out_files
  comm -1 -2 src_files out_files | sed "s#\\.#$outpath#" > files_to_remove
  xargs -a files_to_remove -d\\n rm
  rm src_files out_files files_to_remove
}

copy_image_files() {
  echo "Copying image files"
  imgfiles="nuttx nuttx.bin System.map"
  for fn in $imgfiles; do
    cp $ARA_BUILD_TOPDIR/nuttx/$fn $ARA_BUILD_TOPDIR/image/$fn  >/dev/null 2>&1
    rm -f $ARA_BUILD_TOPDIR/nuttx/$fn >/dev/null 2>&1
  done
}

build_image

if [ $MAKE_RESULT -eq 0 ] ; then
  echo "Build failed"
  exit 1
fi

echo "Build succeeded"
copy_image_files
clean_build
echo "Build complete"
exit 0
