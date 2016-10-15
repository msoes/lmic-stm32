#!/bin/bash

THISDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

TOPDIR=$THISDIR/../..


if [ ! -f "$1" ]; then
    echo "Configuration not found!"
    exit 1
fi
CFG_PATH=$(realpath $1)

if [ ! -f "$2" ]; then
    echo "Firmware not found!"
    exit 1
fi
FW_PATH=$(realpath $2)

openocd -f $CFG_PATH  -f ./flash.cfg -c "flash_binary $FW_PATH"


