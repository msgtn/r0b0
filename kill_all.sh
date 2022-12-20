#!/bin/sh
ps aux | grep "detect" | awk '{print $2}' | xargs kill -9
ps aux | grep "start.py" | awk '{print $2}' | xargs kill -9
ps aux | grep "chrome" | awk '{print $2}' | xargs kill -9
lsof -i tcp:7000 | grep node | awk '{print $2}' | xargs kill -9