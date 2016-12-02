#
# Copyright (c) 2016 Motorola Mobility, LLC.
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
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
source .version
source .config

MAJOR=$(grep CONFIG_VERSION_MAJOR .version | cut -f2 -d"=" )
MINOR=$(grep CONFIG_VERSION_MINOR .version | cut -f2 -d"=" )
printf -v VERSION  '%08x' $(($(($(grep CONFIG_VERSION_MAJOR .version | cut -f2 -d"=" )<<16)) + $(grep CONFIG_VERSION_MINOR .version | cut -f2 -d"=" )))

function tftf_apbe()
{
    CHP_MFG=${CONFIG_WRAP_CHIP_MFG_ID##0x}
    CHP_PID=${CONFIG_WRAP_CHIP_PRODUCT_ID##0x}
    BRD_VID=${CONFIG_WRAP_ARCH_BOARDID_VID##0x}
    BRD_PID=${CONFIG_WRAP_ARCH_BOARDID_PID##0x}
    TFTF_FILENAME="upd-00000126-00001001-fed70128-ffff0002-02.tftf"
    WRAP_FILENAME="upd-${CHP_MFG}-${CHP_PID}-${BRD_VID}-${BRD_PID}-02.tftf"
    start_address="$(grep Reset_Handler System.map | cut -d\  -f1)"

    echo Create base ${TFTF_FILENAME}
    create-tftf -v --elf ${1} \
          --out ${TFTF_FILENAME} \
          --start "0x${start_address}" \
          --unipro-mfg 0x00000126 \
          --unipro-pid 0x00001001 \
          --ara-vid 0xfed70128 \
          --ara-pid 0x00000000 \
          --ara-stage "02" \
          --ara-reserved-tftf 0x${VERSION}

    create-tftf -v \
        --data ${TFTF_FILENAME} \
        --out ${WRAP_FILENAME} \
        --start "0x10000000" \
        --unipro-mfg ${CONFIG_WRAP_CHIP_MFG_ID} \
        --unipro-pid ${CONFIG_WRAP_CHIP_PRODUCT_ID} \
        --ara-vid ${CONFIG_WRAP_ARCH_BOARDID_VID} \
        --ara-pid ${CONFIG_WRAP_ARCH_BOARDID_PID} \
        --ara-stage "02" \
        --ara-reserved-tftf 0x${VERSION}
}

function tftf_apba()
{
    TFTF_FILENAME="upd-00000126-00001001-fed70128-0xffff0001-02.tftf"
    start_address="$(grep Reset_Handler System.map | cut -d\  -f1)"

    create-tftf -v --elf ${1} \
          --out ${TFTF_FILENAME} \
          --start "0x${start_address}" \
          --unipro-mfg 0x00000126 \
          --unipro-pid 0x00001001 \
          --ara-vid 0xfed70128 \
          --ara-pid 0xffff0001 \
          --ara-stage "02" \
          --ara-reserved-tftf 0x${VERSION}
}

function tftf_muc()
{
    CHP_MFG=${CONFIG_CHIP_MFG_ID##0x}
    CHP_PID=${CONFIG_CHIP_PRODUCT_ID##0x}
    BRD_VID=${CONFIG_ARCH_BOARDID_VID##0x}
    BRD_PID=${CONFIG_ARCH_BOARDID_PID##0x}
    TFTF_FILENAME="nuttx.tftf"
    load_address="$(grep _vectors System.map | cut -d\  -f1)"
    start_address="$(grep __start System.map  | cut -d\  -f1)"

    create-tftf -v --code ${1} \
          --out ${TFTF_FILENAME} \
          --start "0x${start_address}" \
          --load "0x${load_address}" \
          --unipro-mfg ${CONFIG_CHIP_MFG_ID} \
          --unipro-pid ${CONFIG_CHIP_PRODUCT_ID} \
          --ara-vid ${CONFIG_ARCH_BOARDID_VID}\
          --ara-pid ${CONFIG_ARCH_BOARDID_PID} \
          --ara-stage ${CONFIG_TFTF_STAGE} \
          --ara-reserved-tftf 0x${VERSION}
}

BIN=$1
ELF=$2

function tftf_muc_hdr()
{
    CHP_MFG=${CONFIG_CHIP_MFG_ID##0x}
    CHP_PID=${CONFIG_CHIP_PRODUCT_ID##0x}
    BRD_VID=${CONFIG_ARCH_BOARDID_VID##0x}
    BRD_PID=${CONFIG_ARCH_BOARDID_PID##0x}
    TFTF_FILENAME="nuttx.tftf"

    dd if=${TFTF_FILENAME} of=tftf.hdr bs=512 count=1
}

echo $MFG_ID
if [ "${CONFIG_UNIPRO_P2P_APBE}"x != "x" ]; then
    echo "APBE"
    tftf_apbe ${ELF}
elif [ "${CONFIG_UNIPRO_P2P_APBA}"x != "x" ]; then
    echo "APBA"
    tftf_apba ${ELF}
else
    echo "MuC"
    tftf_muc ${BIN}
    tftf_muc_hdr
fi
