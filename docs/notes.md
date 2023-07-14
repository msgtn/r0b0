
## Troubleshooting

### Motor settings
Setting motor info e.g. IDs needs torque to be disabled.
For example, to set the ID of motor 1 to 7 in using `r0b0.scripts.motor_calib.py`:
```
set_param('torque_enable',{1:False})
set_param('id',{1:7})
```

### 230712
- testing xl320
- ~~does not work with r0b0.scripts.motor_calib~~
- works under pypot, with params:
  - port: /dev/tty.usbmodem212401
  - baudrate: 1000000 (1e6)
- actually works, but dynamixel_python does not get the expected response, so pinging the motor fails.
- the issue is in the expected return packet
- Even using the U2D2 instead of the USB2AX (as old blsm did), still fails on expected

### 230713
- on raspberry pi, set up environment with `venv` instead of conda
- issues with rtmidi - look recent according to github issues
- running r0b0.scripts.motor_calib, needed to update the USB port
  - switched USB_PORT = '/dev/tty.usbserial-FT1SF1UM' -> '/dev/ttyUSB0'
  - Found portname by running `ls /dev/tty*` when U2D2 motor controller is both plugged and unplugged, and pinpointing what goes away

- in using free ngrok account
  - had to update both `host.py` and `controller.js` with the ngrok-generated url (.e.g https://16ac-32-221-140-83.ngrok-free.app)