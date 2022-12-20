#!/bin/sh
Xvfb :10 -screen 0 1920x1280x24 -ac &
export DISPLAY=:10