#!/bin/sh
cd /home/cappy/blossom/
bash kill_all.sh
# bash enable_x.sh
cd /home/cappy/blossom/blossom-app
bash start.sh &
sleep 2
cd /home/cappy/blossom
bash gradcap.sh &
sleep 2
cd /home/cappy/blossom/pycoral
bash detect_face.sh