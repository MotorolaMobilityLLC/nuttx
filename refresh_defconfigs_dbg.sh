#!/bin/bash
# refresh_defconfigs.sh

# define exit error codes
ARA_BUILD_CONFIG_ERR_BAD_PARAMS=1
ARA_BUILD_CONFIG_ERR_NO_NUTTX_TOPDIR=2
ARA_BUILD_CONFIG_ERR_CONFIG_NOT_FOUND=3
ARA_BUILD_CONFIG_ERR_CONFIG_COPY_FAILED=4

# Other build configuration.
ARA_BUILD_PARALLEL=1            # controls make's -j flag

echo "Project Ara firmware defconfig refresh"
echo "--------------------------"
printenv
echo "--------------------------"

ARA_BUILD_PARALLEL="-j 6"

# determine NuttX top level folder absolute path
TOPDIR="`dirname \"$0\"`"  # relative
TOPDIR="`( cd \"$TOPDIR/nuttx\" && pwd )`"  # absolutized and normalized
if [ -z "$TOPDIR" ] ; then
  # error; for some reason, the path is not accessible
  # to the script (e.g. permissions)
  echo "Can't determine NuttX top level folder, fatal."
  exit $ARA_BUILD_CONFIG_ERR_NO_NUTTX_TOPDIR
fi

rebuild_defconfig() {
  # configpath, defconfigFile, buildname, buildbase
  # must be defined on entry
  echo "rebuild_defconfig() start"
  echo "buildname='$buildname'"

  # define paths used during build process
  ARA_BUILD_CONFIG_PATH="$buildbase/$buildname/config"
  ARA_BUILD_TOPDIR="$buildbase/$buildname"

  # delete build tree if it already exists
  if [ -d $ARA_BUILD_TOPDIR ] ; then
    rm -rf $ARA_BUILD_TOPDIR
  fi

  # create folder structure in build output tree
  mkdir -p "$ARA_BUILD_CONFIG_PATH"
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

  echo "Copying defconfig for '$buildname' to build output tree"
  # copy defconfig to build output tree
  if ! install -m 644 -p ${defconfigFile} ${ARA_BUILD_TOPDIR}/nuttx/.config ; then
      echo "ERROR: Failed to copy defconfig"
      exit $ARA_BUILD_CONFIG_ERR_CONFIG_COPY_FAILED
  fi

  echo "Saving config files for '$buildname'"
  # save config files
  cp ${ARA_BUILD_TOPDIR}/nuttx/.config   ${ARA_BUILD_CONFIG_PATH}/.config > /dev/null 2>&1
  cp ${ARA_BUILD_TOPDIR}/nuttx/Make.defs ${ARA_BUILD_CONFIG_PATH}/Make.defs > /dev/null 2>&1
  cp ${ARA_BUILD_TOPDIR}/nuttx/setenv.sh  ${ARA_BUILD_CONFIG_PATH}/setenv.sh > /dev/null 2>&1

  echo "Building config for '$buildname'"
  export ARA_BUILD_NAME=$buildname
  pushd $ARA_BUILD_TOPDIR/nuttx > /dev/null

#  make -j ${ARA_BUILD_PARALLEL} --always-make -r -f Makefile.unix  olddefconfig > /dev/null 2>&1
  make -j ${ARA_BUILD_PARALLEL} -d --always-make -r -f Makefile.unix  olddefconfig

  MAKE_RESULT=${PIPESTATUS[0]}

  popd > /dev/null
  echo "rebuild_defconfig() end"
}

copy_defconfig_file() {
    echo "copy_defconfig_file()"
    # uses configpath
    if  ! diff -q $ARA_BUILD_TOPDIR/nuttx/.config $configpath/defconfig > /dev/null  ; then
        echo "Updating defconfig"
        cp $ARA_BUILD_TOPDIR/nuttx/.config $configpath/defconfig
    fi
}

remove_build() {
  echo "remove_build()"
  # uses buildbase and buildname
  rm -rf "$buildbase/$buildname"
}

canonicalize() {
    TARGET_FILE=$1

    cd `dirname $TARGET_FILE`
    TARGET_FILE=`basename $TARGET_FILE`

    # Iterate down a (possible) chain of symlinks
    while [ -L "$TARGET_FILE" ]
    do
        TARGET_FILE=`readlink $TARGET_FILE`
        cd `dirname $TARGET_FILE`
        TARGET_FILE=`basename $TARGET_FILE`
    done

    # Compute the canonicalized name by finding the physical path
    # for the directory we're in and appending the target file.
    PHYS_DIR=`pwd -P`
    RESULT=$PHYS_DIR/$TARGET_FILE
    echo $RESULT
}

# set build output path
buildbase="`( cd \"$TOPDIR/..\" && pwd )`/build"
echo "buildbase='$buildbase'"
# build list of defconfigs
defconfig_list=$(find $TOPDIR/configs/ara  -iname defconfig)

# process list of defconfigs
for cfg in $defconfig_list; do
  echo "Checking '$cfg'"
  # save full path to defconfig
  defconfigFile=${cfg}
  # get abs path to defconfig
  configpath=$(canonicalize  $(dirname "$cfg"))
  #create build name
  buildname=$(canonicalize $(dirname "$cfg"))
  #strip abs path
  buildname=$(echo "$buildname" | sed -e "s:^$TOPDIR/configs/::")
  # repl slash with dash
  buildname=$(echo "$buildname" | sed -e "s#/#-#g")
  # rebuild the defconfig
  rebuild_defconfig
  # check build result
  if [ $MAKE_RESULT -ne 0 ] ; then
    echo "Rebuild defconfig for '$buildname' failed"
    exit 1
  fi
  copy_defconfig_file
  remove_build
done
echo "Refresh defconfigs completed"
exit 0


