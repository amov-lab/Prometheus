#!/bin/bash
export SUDO_PASS="amov"
echo $SUDO_PASS | sudo -S killall -9 MediaServer
