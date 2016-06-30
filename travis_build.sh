#!/bin/bash

defconfig_list=$(find nuttx/configs/hdk -iname defconfig)

for cfg in $defconfig_list; do
  configpath=$(dirname "$cfg")
  mod=$(echo "$configpath" | sed -e "s:^nuttx/configs/::")

  echo "============================================"
  echo "== " $mod
  echo "============================================"

  pushd nuttx
    make distclean
    pushd tools
      if ! bash ./configure.sh ${mod}; then
          printf '%s failed!' "configure ${mod}" >&2
          exit 1
      fi
    popd

    make
    if test $? -ne 0; then
        printf '%s failed!' "make ${mod}" >&2
        exit 1
    fi
  popd

done
