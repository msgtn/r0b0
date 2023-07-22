## Setup
I use the Raspberry Pi as a USB gadget.
I used the [TechCraft images](https://github.com/techcraftco/rpi-usb-gadget/releases), specifically `raspios-desktop-arm64-2022-04-04-arm64.img.zip`, for the Raspberry Pi SD.
I use the [official Raspberry Pi imager](https://www.raspberrypi.com/software/) to flash the SD card.
Conveniently, the imager also enables settings (under 'Advanced options') to configure the hostname, username/password, and wireless LAN prior to booting the Pi.

I use VS Code's remote explorer to have IDE access to the Pi.
Make sure that the `Remote Explorer` and `Remote - SSH` extensions are installed in VS Code.
In the Remote Explorer window, add a new host with whatever credentials you use to `ssh` into the Pi.
The image defaults the accessible hostname to `10.55.0.1`, for any Raspberry Pi, so if another Pi/SD card has been ssh'd into, delete the prior configs for this hostname in `~/.ssh/known_hosts` .
For example, `ssh raspberrypi@10.55.0.1`.

## On the Pi

Then, set up git:
```
git config --global user.name "your name"
git config --global user.email "your@email.com"
```
Clone the repo:
```
git clone https://github.com/psychomugs/r0b0
cd r0b0
git submodule update --init --recursive
```
Set up a virtual environment over the default Python3.9:
```
sudo apt install python3-venv
python3 -m venv venv
pip3 install -r requirements/requirements.txt
pip3 install -r requirements/robots.txt
```
Generate ssl keys for the interface:
```
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout r0b0/key.pem -out r0b0/csr.pem
```

### Motor controller
We need to figure out the USB port that the motor controller (e.g. U2D2, USB2AX) is connected to.
Run `ls /dev/tty*` twice, once with the motor controller connected and again with it disconnected, and take note of the port that disappeared, e.g. `/dev/ttyUSB0`.
Modify `r0b0/config/gadgets/blsm_dxl.yaml` with this `usb_port`:
```
type: DynamixelRobot
usb_port: /dev/ttyUSB0
```

### ngrok (for interface)
Sign up for `ngrok`, then download:
```
wget https://bin.equinox.io/c/bNyj1mQVY4c/ngrok-v3-stable-linux-arm64.tgz
sudo tar xvzf ./ngrok-v3-stable-linux-arm64.tgz -C /usr/local/bin
```
Go to your online account settings, and retrieve your `NGOKR_AUTHTOKEN` to input here:
```
ngrok authtoken NGROK_AUTHTOKEN
```
Start a tunnel:
```
ngrok http https://localhost:8080
```
This will start tunneling from some randomly generated address, e.g.
`https://104e-32-221-140-83.ngrok-free.app`.

Modify this into the top lines of `r0b0/rigs/static/controller.js` :
```
const socketAddr = "https://104e-32-221-140-83.ngrok-free.app"
```
and `r0b0/rigs/static/host.py`:
```
SOCKET_ADDR = "https://104e-32-221-140-83.ngrok-free.app"
```

### Motor calibration
Next, we will calibrate the motors.
Open `r0b0/scripts/motor_calib.py` and select / modify the parameters (motor model, USB port, baud rate) towards the top for your configuration (XL330 for the new version of the robot, XL320 for the old version):
```
# for XL330 motors
MOTOR_MODEL,USB_PORT,BAUD_RATE = 'xl330-m288','/dev/tty.usbserial-FT1SF1UM',57600
# for XL320 motors
MOTOR_MODEL,USB_PORT,BAUD_RATE = 'xl320','/dev/tty.usbmodem212401',1e6
```
With **one motor connected at a time**, run this calibration script:
```
python3 -m r0b0.scripts.motor_calib
```
This will scan for connected motors, and should find the connected motor, usually with ID 1 if it has not yet been set. 
The script will pause at `(Pdb)` — this means that the script has started successfully and is now in a debugging loop.
To set the ID, for example from 1 to 2:
```
m1 = dxl_mgr.dxl_dict['1']
m1.set_torque_enable(False)
m1.set_id(2)
m2 = dxl_mgr.dxl_dict['2']
m2.set_torque_enable(True)
```
To test if the ID was changed successfully, we can toggle the LED.
```
m2.set_led(True)
m2.set_led(False)
```
To stop the script, type `Ctrl+D`.
Repeat this for motor IDs 3, 4, and 5.


### Start the robot
With the motor controller plugged in, start the robot (*Note: replace `blsm` with `blsm_320` if you are using the [older version of Blossom that uses XL320 motors](https://github.com/hrc2/blossom-public), replace `blsm` with `blsm_320`.*):
```
python3 start.py --config blsm
```

Wait until `(Pdb)` shows up in the terminal — this means that the script has started successfully and is now in a debugging loop.
Navigate to `https://104e-32-221-140-83.ngrok-free.app` in a mobile browser (ideally Safari on an iPhone), accept the warning, and turn on the `HEAD` control to motion control the robot.
To stop the script, type `Ctrl+D`.