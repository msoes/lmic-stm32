#!/bin/bash

THISDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

TOPDIR=$THISDIR/../..

FW_PATH=$(realpath $1)

openocd -f ./imst.cfg  -f ./flash.cfg -c "flash_binary $FW_PATH"


