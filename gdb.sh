#!/bin/bash

TARGET=${1:-debug}

set -e

echo "Please run 'st-util' in another terminal window (you might need sudo)"
echo ""

XARGO_ARGS=

if [ "${TARGET}" == "release" ]
then
    XARGO_ARGS="--release"
fi

xargo build ${XARGO_ARGS}

arm-none-eabi-gdb -iex 'add-auto-load-safe-path .' -ex "tar ext :4242" -ex "load-reset" "target/stm32f7/${TARGET}/roaster"
