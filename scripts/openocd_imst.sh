#!/bin/bash

THISDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo $THISDIR

CFG_NAME=$1
if [ -z "$CFG_NAME" ]; then
    echo "No imst config supplied!"
fi


while true; do openocd -s /usr/local/share/openocd/  -f $THISDIR/$CFG_NAME  ; sleep 1;done
