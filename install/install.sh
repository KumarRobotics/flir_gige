#!/bin/bash

# Stop if a command ends in an error
set -e

# Print out ASCII-Art Logo.
clear;
echo "======================================================================"
echo " _  __                          ____       _           _   _          "
echo "| |/ /   _ _ __ ___   __ _ _ __|  _ \ ___ | |__   ___ | |_(_) ___ ___ "
echo "| ' / | | | '_ \` _ \ / _\` | '__| |_) / _ \| '_ \ / _ \| __| |/ __/ __|"
echo "| . \ |_| | | | | | | (_| | |  |  _ < (_) | |_) | (_) | |_| | (__\__ \\"
echo "|_|\_\__,_|_| |_| |_|\__,_|_|  |_| \_\___/|_.__/ \___/ \__|_|\___|___/"
echo
echo "======================================================================"
sleep 1

# version=$(lsb_release -r | awk '{print $2}')
# echo -e "${version}"
# need to link libudev and liblog4cxx

# Check to see if we are running with root privileges
if [[ $(id -u) -ne 0 ]] ; then
    echo "Please run this script as root. (eg. using sudo)"
    exit 1
fi
