#!/bin/sh
cat apt_rpi.txt | xargs -n 1 sudo apt -y install
