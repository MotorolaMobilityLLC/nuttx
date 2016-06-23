#!/bin/bash

targets=""
targets+=" hdk/muc/base_powered"
targets+=" hdk/muc/base_unpowered"
targets+=" hdk/muc/battery"
targets+=" hdk/muc/blinky"
targets+=" hdk/muc/compute"
targets+=" hdk/muc/display"
targets+=" hdk/muc/factory"
targets+=" hdk/muc/speaker"
targets+=" hdk/muc/temperature"

targets+=" hdk/apbe/base"

echo targets: ${targets}

for mod in $targets; do
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
