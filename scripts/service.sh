#!/bin/sh
sudo cp service/blsm-servo.service /etc/systemd/system/blsm-servo.service
sudo systemctl daemon-reload
sudo systemctl enable blsm-servo
sudo systemctl start blsm-servo
