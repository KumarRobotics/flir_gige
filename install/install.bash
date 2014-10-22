#!/usr/bin/env bash

CURRENT_DIR=$(pwd)
REPO_NAME=ebus_sdk
CLONE_DIR=/tmp

sudo apt-get install libudev-dev
if [[ ! -L libudev.so.0 ]]; then
    sudo ln -s /usr/lib/x86_64-linux-gnu/libudev.so /usr/lib/libudev.so.0
    sudo ln -s /usr/lib/libudev.so.0 /usr/lib/libudev.so.1
fi

cd ${CLONE_DIR}
git clone https://github.com/versatran01/${REPO_NAME}
cd ${REPO_NAME}
PLATFORM=$(uname -i)
if [[ ${PLATFORM} == "x86_64" ]] ; then
    echo "Installing 64-bit version"
    sudo ./eBUS_SDK_4.0.6.3228_Ubuntu-12.04-x86_64.run
else
    echo "Platform ${PLATFORM} not supported"
fi

echo "Cleaning up..."
cd ${CLONE_DIR}
rm -rf ${REPO_NAME}
cd ${CURRENT_DIR}
echo "Done."

