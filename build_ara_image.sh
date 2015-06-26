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
ARA_BUILD_CONFIG_ERR_CONFIG_COPY_FAILED=4

# Other build configuration.
ARA_BUILD_PARALLEL=1            # controls make's -j flag

echo "Project Ara firmware image builder"

USAGE="

USAGE:
    (1) rebuild specific image config
        ${0} [-j N] <board-name> <config-name>
    (2) rebuild all image configs under configs/ara
        ${0} [-j N] all

Options:
  -j N: do a parallel build with N processes

Arguments:
  <board-name> is the name of the board in the configs directory
  <config-name> is the name of the board configuration sub-directory

"

buildall=0

while getopts "j:" opt; do
    case $opt in
        j)
            ARA_BUILD_PARALLEL=${OPTARG}
            ;;
        \?)
            echo "Unknown option: -$OPTARG." >&2
            echo $USAGE
            exit $ARA_BUILD_CONFIG_ERR_BAD_PARAMS
            ;;
        :)
            echo "Missing required argument for -$OPTARG." >&2
            echo $USAGE
            exit $ARA_BUILD_CONFIG_ERR_BAD_PARAMS
            ;;
    esac
done
shift $((OPTIND-1))

# check for "all" parameter
if [ "$1" = "all" ] ; then
  echo "Building all configurations"
  buildall=1
# validate parameters for board & image are present
elif  [ "$#" -ne 2 ] ; then
  echo "Required parameters not specified."
  echo "$USAGE"
  exit $ARA_BUILD_CONFIG_ERR_BAD_PARAMS
#capture parameters for board  & image
else
  board=$1
  image=$2
fi

# determine NuttX top level folder absolute path
TOPDIR="`dirname \"$0\"`"  # relative
TOPDIR="`( cd \"$TOPDIR/nuttx\" && pwd )`"  # absolutized and normalized
if [ -z "$TOPDIR" ] ; then
  # error; for some reason, the path is not accessible
  # to the script (e.g. permissions)
  echo "Can't determine NuttX top level folder, fatal."
  exit $ARA_BUILD_CONFIG_ERR_NO_NUTTX_TOPDIR
fi

build_image_from_defconfig() {
  # configpath, defconfigFile, buildname, buildbase
  # must be defined on entry

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
  cp -r $TOPDIR/../nuttx $ARA_BUILD_TOPDIR/nuttx
  cp -r $TOPDIR/../apps $ARA_BUILD_TOPDIR/apps
  cp -r $TOPDIR/../misc $ARA_BUILD_TOPDIR/misc
  cp -r $TOPDIR/../NxWidgets $ARA_BUILD_TOPDIR/NxWidgets

  # copy Make.defs to build output tree
  if ! install -m 644 -p ${configpath}/Make.defs ${ARA_BUILD_TOPDIR}/nuttx/Make.defs  >/dev/null 2>&1; then
      echo "Warning: Failed to copy Make.defs"
  fi

  # copy setenv.sh to build output tree
  if  install -p ${configpath}/setenv.sh ${ARA_BUILD_TOPDIR}/nuttx/setenv.sh >/dev/null 2>&1; then
  chmod 755 "${ARA_BUILD_TOPDIR}/nuttx/setenv.sh"
  fi

  # copy defconfig to build output tree
  if ! install -m 644 -p ${defconfigFile} ${ARA_BUILD_TOPDIR}/nuttx/.config ; then
      echo "ERROR: Failed to copy defconfig"
      exit $ARA_BUILD_CONFIG_ERR_CONFIG_COPY_FAILED
  fi

  # save config files
  cp ${ARA_BUILD_TOPDIR}/nuttx/.config   ${ARA_BUILD_CONFIG_PATH}/.config > /dev/null 2>&1
  cp ${ARA_BUILD_TOPDIR}/nuttx/Make.defs ${ARA_BUILD_CONFIG_PATH}/Make.defs > /dev/null 2>&1
  cp ${ARA_BUILD_TOPDIR}/nuttx/setenv.sh  ${ARA_BUILD_CONFIG_PATH}/setenv.sh > /dev/null 2>&1

  echo -n "Building '$buildname'" ...
  export ARA_BUILD_NAME=$buildname
  pushd $ARA_BUILD_TOPDIR/nuttx > /dev/null
  make  -j ${ARA_BUILD_PARALLEL} --always-make -r -f Makefile.unix  2>&1 | tee $ARA_BUILD_TOPDIR/build.log

  MAKE_RESULT=${PIPESTATUS[0]}

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
  # if bridge image (i.e. *not* an svc image)
  # expand image to 2M using truncate utility
  # for more info, run "truncate --help"
  if [ -z $(echo $buildname | grep "svc")  ] ; then
    truncate -s 2M $ARA_BUILD_TOPDIR/image/nuttx.bin
  fi
}

# set build output path
buildbase="`( cd \"$TOPDIR/..\" && pwd )`/build"

if [ $buildall -eq 1 ] ; then
  # build list of defconfigs
  defconfig_list=$(find $TOPDIR/configs/ara  -iname defconfig)
  # process list of defconfigs
  for cfg in $defconfig_list; do
    # save full path to defconfig
    defconfigFile=${cfg}
    # get abs path to defconfig
    configpath=$(readlink -f $(dirname "$cfg"))
    #create build name
    buildname=$(readlink -f $(dirname "$cfg"))
    #strip abs path
    buildname=$(echo "$buildname" | sed -e "s:^$TOPDIR/configs/::")
    # repl slash with dash
    buildname=$(echo "$buildname" | sed -e "s#/#-#g")
    # build the image
    build_image_from_defconfig
    # check build result
    if [ $MAKE_RESULT -ne 0 ] ; then
      echo "Build '$buildname' failed"
      exit 1
    fi
    echo "Build '$buildname' succeeded"
    copy_image_files
    clean_build
  done
  echo "Build all configurations succeeded"
  exit 0
fi

# build from board+image params

# set path to image config
configpath=${TOPDIR}/configs/${board}/${image}
if [ ! -d "${configpath}" ]; then
  echo "Build config '${configpath}' does not exist"
  exit $ARA_BUILD_CONFIG_ERR_CONFIG_NOT_FOUND
fi
# create build name from config
# substitute "-" for "/"
board=$(echo "$board" | sed -e "s#/#-#")
image=$(echo "$image" | sed -e "s#/#-#")
buildname=${board}-${image}
# full path to defconfig file
defconfigFile="${configpath}/defconfig"
build_image_from_defconfig
if [ $MAKE_RESULT -ne 0 ] ; then
  echo "Build '$buildname' failed"
  exit 1
fi

echo "Build '$buildname' succeeded"
copy_image_files
clean_build
echo "Build complete"
exit 0
