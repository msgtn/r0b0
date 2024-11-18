# Blossom

![A layout of the robot's parts and the completed model with cover](https://github.com/msgtn/r0b0/blob/main/docs/assets/maker-faire.jpg)

*~~2023-07-14~~2024-11-01 This page is *still* a work-in-progress.*
This page covers the build and use of a Blossom robot, specifically:
- Building the robot hardware
- Setting up the software
- Controlling the robot using the mobile interface

## Build

![View of the robot's inner skeleton and a layout of its constituent components](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm-teaser.jpg)

The total component cost for the base Dynamixel-powered configuration is less than $200.
The platform also supports a cheaper configuration using micro servos and an Arduino for approximately $50.


### Parts

![](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm-instructions.png)
The parts to print in `*.stl` format are [available here](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/).
Each directory contains the whole runner (e.g. [blsm_A](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_A/blsm_A.stl) ) or individual parts (e.g. [A1](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_A/A1.stl)).
I printed the parts on an entry-level [Creality Ender 3](https://www.creality.com/products/ender-3-3d-printer) and sliced the files in Cura using [this profile](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm-020.curaprofile).
The instructions are [available here](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm.pdf).

| Part |  Description | Quantity | Method | Approximate total cost |
| ---- | ----- | -------- | ------ | --- |
| blsm_A | Part runner | 1 | [Print](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_A/) | NA |
| blsm_B | Part runner | 2 | [Print](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_B/) | NA |
| blsm_C | Part runner | 2 | [Print](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_C/) | NA |
| blsm_D | Part runner | 1 | [Print](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_D/) | NA |
| blsm_E | Part runner | 3 | [Print](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_E/) | NA |
| blsm_F | Part runner | 3 | [Print](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_F/) | NA |
| blsm_M | Part runner | 4 | [Print](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_M/) | NA |
| blsm_SR | *S*lip *R*ing for rotating the upper body | 1 | [Print the static non-rotating 'dummy' model](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_SR.stl), or use [the actual 12-wire slip ring to enable continuous rotation beyond 360 degrees](https://www.sparkfun.com/products/13065) | NA or $22.00 |
| String | String for actuating the head | 1 | Purchase (e.g. [fishing line](https://www.powerpro.com/content/powerpro/northamerica/us/en/homepage/PDP.P-POWERPRO.html), [twine](https://www.amazon.com/White-Cotton-Butchers-Twine-String/dp/B09TQXBFYD/)) | $5-$20 |
| Rubber bands, 4mm diameter | Rubber bands for hanging the head platform | 6 | Purchase e.g. [black rubber bands](https://www.amazon.com/Rubber-200pcs-Elastic-Sturdy-School/dp/B0924HDQXQ/) | $8 |
| TOTAL | | |  | $13-$50 |

### Motors and electronics
The standard full-featured configuration uses Dynamixel 'smart' servos.
Dynamixels have a lot of nice features built in, such as different operating modes (position- or velocity-control modes), velocity and acceleration profiles, and PID tweaking.

| Part |  Description | Quantity | Method | Approximate total cost |
| ---- | ----- | -------- | ------ | --- |
| Dynamixel XL330-M288 | Motor | 4 | [Purchase](https://www.robotis.us/dynamixel-xl330-m288-t) | $100 ($25/ea) |
| Dynamixel X3P Cable | Cables (included with XL330 motors) | 6 | [Purchase](https://www.robotis.us/robot-cable-x3p-180mm-10pcs) | NA (included with XL330 motors — only buy if need spares) |
| Dynamixel U2D2 | Motor controller | 1 | [Purchase](https://www.robotis.us/u2d2/) | $32.00 |
| TOTAL | | | | $132 |

The Arduino-powered configuration is much cheaper (the total cost equivalent to just one Dynamixel) and more hackable, but the movement is noisier and less smooth.
The left-right yaw rotation is also limited to ±90°.
This version is more readily hackable with whatever can interface with an Arduino.

| Part |  Description | Quantity | Method | Approximate total cost |
| ---- | ----- | -------- | ------ | --- |
| Arduino (or clone) | Microcontroller and cable | 1 | [Purchase](https://www.amazon.com/ELEGOO-Board-ATmega328P-ATMEGA16U2-Compliant/dp/B01EWOE0UU) | $15 |
| Micro servo | Small servos with basic position control | 4 | [Purchase](https://www.amazon.com/Dorhea-Arduino-Helicopter-Airplane-Walking/dp/B07Q6JGWNV/) | $10 |
| TOTAL | | | | $25 |

### Wiring
For the Dynamixel-powered configuration, [refer to the wiring documentation to set up the U2D2 controller with an external power supply](https://github.com/msgtn/r0b0/blob/main/docs/wiring.md).
For the Arduino-powered design, refer to [this Fritzing diagram](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_ard.png).
Either configuration will require USB breakouts for power and some cables.

| Part |  Description | Quantity | Method | Approximate total cost |
| ---- | ------------ | -------- | ------ | ---------------------- |
| USB breakout | Breaks out power connections to ease supplying power | 1 | [Purchase](https://www.amazon.com/Treedix-Type-C-Breakout-Connector-Converter/dp/B096M2HQLK) | $6 |
| Male–Female and Male–Male wires | Connects components | Several | [Purchase](amazon.com/Elegoo-EL-CP-004-Multicolored-Breadboard-arduino/dp/B01EV70C78) or use spare cables and breadboards | $7 |
| TOTAL | | | | $13 |

### Hardware
The minimal M2 hardware required is all available in [this set](https://www.amazon.com/gp/product/B082XR52P1/) for $10.

| Part | Quantity | Notes |
| ---- | -------- | ----- |
| M2x8mm | 14 | |
| M2x10mm | 12 | |
| M2 nuts | 12 | Only necessary if using SG90 servos |

### Tools and miscellaneous

| Part | Description | Quantity | Notes |
| ---- | ----------- | -------- | --- | 
| Wire cutters | Cutting parts from the runners, cutting wires | 1 | |
| Electrical wire | Connecting motors | A couple meters worth | |
| USB wall adapter | Powering the motors | 1 | Ideally 15W (5V/3A) or greater to supply sufficient power to the motors |


### Wiring
Follow the [wiring instructions](https://github.com/msgtn/r0b0/blob/main/docs/wiring.md).

## Software

### Environment setup
Set up [conda](https://conda.io), then set up a conda environment and install some other dependencies with `pip` (because of issues with [mouse](https://github.com/boppreh/mouse/issues/75)). Docker maybe coming soon (maybe).
```
conda env create -f env.yaml
conda activate r0b0
pip3 install -r req.txt 
```

To enable `https` for the control page, generate some keys with `openssl`.
Since this is self-signing(*?*), you can safely hit 'Enter' to accept the defaults for all fields.
```
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout r0b0/key.pem -out r0b0/csr.pem
```

### ngrok setup
*This has only been tested on iOS.*
Phone-based motion control is enabled through `ngrok`.
`ngrok` opens a tunnel to a local port (e.g. `localhost:8080`) through a URL.
Tunneling enables sending data transmission even from non-local networks — this enables telepresence by sending phone orientation data and WebRTC handshaking through the tunnel.
If you're not interested in motion control, you can skip this section.

Sign up for [ngrok](https://ngrok.com).
Continue on with the guides until you can run `ngrok` as a terminal command - this will probably require some `sudo apt`ing (Linux) or `brew`ing (macOS) and some `authtoken`ing.
Start a tunnel to `https://{hostname}:{port}` that the `blsm` rig is running on, e.g. with the defaults of `hostname=localhost` and `port=8080`:
```
ngrok http https://localhost:8080
```

The terminal will show you the forwarding URL, e.g.:
```
...
Forwarding http://someRandomLettersAndNumbers.ngrok.app -> https://localhost:8080
Forwarding https://someRandomLettersAndNumbers.ngrok.app -> https://localhost:8080
...
```

*This next part is a kludge.* We need to update this address in three files: `r0b0/rigs/static/controller.js`, `r0b0/rigs/static/player.js`, and `r0b0/rigs/host.py`.
This address is stored as `socketAddr` and `SOCKET_ADDR` towards the top of each file — modify these to `https://someRandomLettersAndNumbers.ngrok.app`:
In `controller.js` and `player.js`:
```
const socketAddr = "https://someRandomLettersAndNumbers.ngrok.app"
```
In `host.py`:
```
SOCKET_ADDR = "https://someRandomLettersAndNumbers.ngrok.app"
```

Note that `ngrok` must be running in a separate terminal — start it, then open another terminal to continue the instructions.

If you have a paid `ngrok` subscription, you can add a `--subdomain` argument to the tunnel command to maintain a consistent forwarding URL.
For example, to set the forwarding URL to `https://mysubdomain.ngrok.io`:
```
ngrok http https://localhost:8080 --subdomain=mysubdomain
```

### Motor calibration (Dynamixel models only)
Next, we will calibrate the motors.
This is only necessary for Dynamixel motors
First, we need to figure out the USB port that the motor controller (e.g. U2D2, USB2AX) is connected to.
Run `ls /dev/tty*` twice, once with the motor controller connected and again with it disconnected, and take note of the port that disappeared, e.g. `/dev/tty.usbserial-FT1SF1UM`.
Open `r0b0/scripts/motor_calib.py` and modify the parameters (motor model, USB port, baud rate) towards the top for your robot's configuration (XL330 for the new version of the robot, XL320 for the old version):
```
# an example for XL330 motors
MOTOR_MODEL,USB_PORT,BAUD_RATE = 'xl330-m288','/dev/tty.usbserial-FT1SF1UM',57600
# an example for XL320 motors
MOTOR_MODEL,USB_PORT,BAUD_RATE = 'xl320','/dev/tty.usbmodem212401',1e6
```
With one motor connected at a time, run this calibration script:
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
To set the motor to the default position:
```
# for XL330
m2.set_goal_position(1000) # for the towers:1000; for the base: 2000
# for XL320
m2.set_goal_position(700) # for the towers:700 ; for the base: 500
```
To stop the script, type `Ctrl+D`.
Repeat this for motor IDs 3 and 4.

## Starting the `blsm` rig

### Dynamixel
Start the `blsm` rig configuration, which contains the `blsm_dxl` robot as a `DynamixelRobot` and the `bslm_phone` browser-based interface as a `Page`.
The rig uses the `motion2motor` cable to translate `device_motion` events from the page (when accessed from a mobile browser) into `position` events for the motor.
<!-- *Note: if using the older Blossom -->

In `/config/gadgets/blsm_dxl.yaml` ([here](https://github.com/msgtn/r0b0/blob/main/config/gadgets/blsm_ard.yaml)), modify `usb_port` with the port we found during the motor calibration step:
```
type: DynamixelRobot
usb_port: /dev/tty.usbserial-FT1SF1UM   # modify this
```

In a separate terminal window from the `ngrok` tunnel script, 
```
python3 start.py --config blsm
```

### Arduino

We must first flash the Arduino with the pyFirmata firmware, which enables the Arduino to be controlled from Python through the [Arduino gadget class](../r0b0/gadgets/arduino.py).
Connect the Arduino to the computer.
Open [r0b0/gadgets/Standardfirmata.ino](../r0b0/gadgets/StandardFirmata/StandardFirmata.ino) in the [Arduino IDE](https://www.arduino.cc/en/software).
To find the port that the Arduino is connected to, use the Arduino IDE (`Tools` > `Port`).
Upload the firmware to the board (`Sketch` > `Upload`).

Next, we need to modify the configuration at `/config/gadgets/blsm_ard.yaml` ([here](https://github.com/msgtn/r0b0/blob/main/config/gadgets/blsm_ard.yaml)) with the `usb_port` and motor `id`s.
For the motor IDs, refer to the [Fritzing diagram](https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_ard.png) and modify according to your specific build:
```
type: ArduinoRobot
usb_port: /dev/cu.usbserial-ADAQDbKpQ # modify this to the port that the Arduino is connected to
baud_rate: 57600
timeout: 2
motors:
- name: base
  id: 9         # modify this to the pin that the BASE motor is connected to
- name: tower_1
  id: 10        # modify this to the pin that the FRONT head motor is connected to
- name: tower_2
  id: 6         # modify this to the pin that the LEFT head motor is connected to
- name: tower_3
  id: 5         # modify this to the pin that the RIGHT head motor is connected to
```

## Telepresence

### Video (optional)

Connect a USB webcam to your computer.
With the prior scripts running (`start.py` and the `ngrok` tunnel), on the desktop/laptop computer controlling the robot, navigate to `https://localhost:8080/broadcaster`.
This page contains the controls for WebRTC media sources.
Select the connected webcam in the dropdown, which should begin a video feed on the page.
<!-- The video should also appear on the mobile device connected to the `ngrok` page, though you may need to refresh or press a 'play' icon (▶️) if it appears. -->

### Control

In a mobile browser (e.g. Safari), navigate to the forwarding URL (`https://someRandomLettersAndNumbers.ngrok.app` in the above example). 
*Note: since the ssl certificates were self signed, you will probably run into a privacy warning on your browser. [Here's a guide on how to bypass this, which should be safe since this is being developed locally anyways.](https://www.vultr.com/docs/how-to-bypass-the-https-warning-for-self-signed-ssl-tls-certificates/)*

You should see video feed from the webcam selected in `https://localhost:8080/broadcaster`.
Hold the phone straight out, as if you were taking a picture of something directly in front of you. 
Toggle the 'head' switch to turn on control and begin transmitting the phone orientation to the robot.
The motor controller should start blinking blue to indicate that it is sending motor commands.
The robot's head should be moving in response to the phone motion.

### Recording movements
To begin recording a movement, ensure that the control switch is on and click the large red recording button in the center.
Move the phone to control the robot, then click the recording button again to stop.
This will save the motion as a `Tape` in the `/tapes` directory (more documentation [here](https://github.com/msgtn/r0b0/blob/main/r0b0/gadgets/README.md)).

### Player
In either the desktop or mobile browser, navigate to the Player page at `https://someRandomLettersAndNumbers.ngrok.app/player`.
Click 'Update' to populate the dropdown with the tape files in `tapes`.
Select a tape and click 'Play' to begin playback.
If you create new movement recordings using the controller interface, you can repopulate the dropdown by clicking 'Update' without having to refresh the page.
Note that tapes are only loaded once in the backend, so if you manually rename files, you must restart the whole `start.py` script to override the cached tape.

You can also call this function from the command line. 
For example, to play `tapes/demo_tape.json`:
```
rig.play('demo_tape')
```

## Troubleshooting

### Motor settings
Setting motor info e.g. IDs needs torque to be disabled.
For example, to set the ID of motor 1 to 7 in using `r0b0.scripts.motor_calib.py`:
```
set_param('torque_enable',{1:False})
set_param('id',{1:7})
```
### Interface issues
On the mobile interface, turning on the control switch should first prompt a request for access to the device orientation.
If this is not popping up, ensure that `socketAddr`/`SOCKET_ADDR` are defined appropriately in `r0b0/rigs/static/controller.js`, `r0b0/rigs/static/player.js`, and `r0b0/rigs/host.py`.
They should be set to the `ngrok` address tunnelling to `https://localhost:8080`, e.g. `https://104e-32-221-140-83.ngrok-free.app`.

### Slow control
There is a bit of lag between the phone control and the robot control, which is to be expected considering the data passing through the network.
Try the following if the lag is too large for your application.

#### Networking
Ensure that the phone controller is connected to the same network as the robot's computer.

#### Motor parameters 
The robot configuration at `/config/gadgets/blsm_dxl.yaml` ([here](https://github.com/msgtn/r0b0/blob/main/config/gadgets/blsm_dxl.yaml)) contains parameters for the motor movement, such as the goal/profile velocity/acceleration.
On startup, `/r0b0/gadgets/dxl_robot.py` ([here](https://github.com/msgtn/r0b0/blob/main/src/r0b0/gadgets/dxl_robot.py)) configures these parameters during startup. 
You can tune these values, and [refer to the motor documentation](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/) for available parameters.

To set motor parameters, add values as entries in the configuration file.
Any writable parameter can be set in the configuration file — just add the entry as lower cased and underscored (e.g. 'Profile Velocity' -> `profile_velocity`)
For example, in `/config/gadgets/blsm_dxl.yaml` ([here](https://github.com/msgtn/r0b0/blob/main/config/gadgets/blsm_dxl.yaml)), to set `tower_1`'s [Profile Velocity](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#profile-velocity) and [Profile Acceleration](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#profile-acceleration) to 300 and 100, respectively:
```
- name: tower_1
  model: xl330-m288
  id: 1
  operating_mode: 3
  profile_velocity: 300       
  profile_acceleration: 100
```
Setting `operating_mode: 3` sets the motors to position control mode, per the [documentation](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#operating-mode).
Faster velocity and acceleration will yield snappier movements at the risk of jerkiness.
