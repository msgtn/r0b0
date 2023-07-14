# Blossom

*2023-07-14 This page is a work-in-progress.*
This page goes over how to build and boot a Blossom robot.

## Build

Build the robot.
Files and instructions coming soon.

The total component cost is less than $200 for the full Dynamixel-powered configuration.
The design also supports a cheaper configuration using micro servos and an Arduino.
The Arduino-powered functionality is similar, though the movement is noisier and less smooth.

### Parts
The parts to print in `*.stl` format are [available here](./assets/blsm/).
The instructions are [available here](./assets/blsm/blsm.pdf).
| Part |  Description | Quantity | Method | Approximate total cost |
| ---- | ----- | -------- | ------ | --- |
| blsm_A | Part runner | 1 | [Print](./assets/blsm/blsm_A.stl) | NA |
| blsm_B | Part runner | 2 | [Print](./assets/blsm/blsm_B.stl) | NA |
| blsm_C | Part runner | 2 | [Print](./assets/blsm/blsm_C.stl) | NA |
| blsm_D | Part runner | 1 | [Print](./assets/blsm/blsm_D.stl) | NA |
| blsm_E | Part runner | 3 | [Print](./assets/blsm/blsm_E.stl) | NA |
| blsm_F | Part runner | 3 | [Print](./assets/blsm/blsm_F.stl) | NA |
| blsm_M | Part runner | 4 | [Print](./assets/blsm/blsm_M.stl) | NA |
| blsm_SR | Slip ring for rotating the upper body | 1 | [Print](./assets/blsm/blsm_SR.stl), or use a [12-wire slip ring to enable continuous rotation beyond 360 degrees](https://www.sparkfun.com/products/13065) | NA or $22.00 |
| String | String for actuating the head | 1 | Purchase (e.g. [fishing line](https://www.powerpro.com/content/powerpro/northamerica/us/en/homepage/PDP.P-POWERPRO.html), [twine](https://www.amazon.com/White-Cotton-Butchers-Twine-String/dp/B09TQXBFYD/)) | $5-$20 |
| Rubber bands, 4mm diameter | Rubber bands for hanging the head platform | 6 | Purchase (e.g. [black rubber bands](https://www.amazon.com/Rubber-200pcs-Elastic-Sturdy-School/dp/B0924HDQXQ/)) | $8 |

### Motors
The standard full-featured configuration uses Dynamixel 'smart' servos.
Dynamixels have a lot of nice features built in, such as different operating modes (position- or velocity-control modes), velocity and acceleration profiles, and PID tweaking.
| Part |  Description | Quantity | Method | Approximate total cost |
| ---- | ----- | -------- | ------ | --- |
| Dynamixel XL330-M288 | Motor | 4 | [Purchase](https://www.robotis.us/dynamixel-xl330-m288-t/) | $100 ($25/ea) |
| Dynamixel X3P Cable | Cables (included with XL330 motors) | 6 | [Purchase](https://www.robotis.us/robot-cable-x3p-180mm-10pcs/) | NA (included with XL330 motors — only buy if need spares) |
| Dynamixel U2D2 | Motor controller | 1 | [Purchase](https://www.robotis.us/u2d2/) | $32.00 |

Arduino configuration:
| Part |  Description | Quantity | Method | Approximate total cost |
| ---- | ----- | -------- | ------ | --- |
| Arduino (or clone) | Microcontroller and cable | 1 | [Purchase](https://www.amazon.com/ELEGOO-Board-ATmega328P-ATMEGA16U2-Compliant/dp/B01EWOE0UU) | $15 |
| Micro servo | Small servos with basic position control | 4 | [Purchase](https://www.amazon.com/Dorhea-Arduino-Helicopter-Airplane-Walking/dp/B07Q6JGWNV/) | $10 |

### Hardware
Below is the minimal M2 hardware required.
They are all available in [this set](https://www.amazon.com/gp/product/B082XR52P1/) for $10.
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
For the Dynamixel-powered configuration, [refer to the wiring documentation to set up the U2D2 controller with an external power supply](./wiring.md).
For the Arduino-powered design, 

## Software

### Environment setup
Set up [conda](https://conda.io), then set up a conda environment and install some other dependencies with `pip` (because of issues with [`mouse`](https://github.com/boppreh/mouse/issues/75)). Docker maybe coming soon (maybe).
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

*This next part is a kludge.* We need to update this address in two files: `r0b0/rigs/static/controller.js`, and `r0b0/rigs/host.py`.
This address is stored as `socketAddr` and `SOCKET_ADDR` towards the top of each file — modify these to `https://someRandomLettersAndNumbers.ngrok.app`:
In `controller.js`:
```
const socketAddr = "https://someRandomLettersAndNumbers.ngrok.app"
```
In `host.py`:
```
SOCKET_ADDR = "https://someRandomLettersAndNumbers.ngrok.app"
```


### `blsm` rig
Start the `blsm` rig, which contains the `blsm_dxl` robot as a `DynamixelRobot` and the `bslm_phone` browser-based interface as a `Page`. The `motion2motor` cable translates `device_motion` events from the page (when accessed from a mobile browser) into `position` events for the motor.
```
python3 start.py --config blsm
```

## Telepresence

### Video

Connect a USB webcam to your computer.

With the prior scripts running (`start.py` and the `ngrok` tunnel), on the desktop/laptop computer controlling the robot, navigate to `https://localhost:8080/broadcaster`.

This page contains the controls for WebRTC media sources.
Select the connected webcam in the dropdown, which should begin a video feed on the page.
<!-- The video should also appear on the mobile device connected to the `ngrok` page, though you may need to refresh or press a 'play' icon (▶️) if it appears. -->

### Control

In a mobile browser (e.g. Safari), navigate to the forwarding URL (`https://someRandomLettersAndNumbers.ngrok.app` in the above example). 
*Note: since the ssl certificates were self signed, you will probably run into a privacy warning on your browser. [Here's a guide on how to bypass this, since this is being developed locally anyways.](https://www.vultr.com/docs/how-to-bypass-the-https-warning-for-self-signed-ssl-tls-certificates/)*

You should see video feed from the webcam selected in `https://localhost:8080/broadcaster`.
Hold the phone straight in front of you, as if you were taking a picture of something directly in front of you. 
Toggle the 'head' switch to turn on control and begin transmitting the phone orientation to the robot.
The U2D2 motor controller should start blinking blue: this indicates that it is sending motor commands.


## Appendix

### Telepresence implementation
WebRTC is a confusing crissing-crossing async shouting match not unlike a political debate on an uncle's Facebook status.
Here's a table of the signaling that I *think* is going on between the desktop page `blsm_broadcaster.js` and the mobile page `blsm_controller.js`.
The robot is on the broadcaster's end; the controller is the remote user moving their phone to remotely control the robot.
Time is going downwards.
| broadcaster | controller | what's going on |
|:------------|-----------:|:-|
| connect | connect | Both devices connect to the same socket |
| emits `broadcaster` | | The broadcaster signals that it has media to broadcast |
| | emits `watcher`| the watcher signals that can watch |
| handles `watcher` | | The broadcaster adds the controller as a new `RTCPeerConnection`, gets its own media devices (i.e. the camer and microphone connected to the robot's computer), and sends this information to the controller |
| emits `candidate` | | The broadcaster sends information of the shared ICE server that clients on the same socket will use |
| | handles `candidate` | The controller processes the ICE server information |
| creates and emits `offer` | | The broadcaster offers details of its streaming capabilities |
| | handles `offer` | The controller reads the broadcaster's stream and updates its video stream with the robot's camera feed | 
| | creates and emits `answer` | The controller shares details of its own streaming capabilities (i.e. for the remote controller's audio/video to come through on the broadcaster's robot) |
| handles `answer` | | The broadcaster updates its video stream with the controller's camera feed (probably the phone's front-facing camera) |


### Design goals
Blossom serves as a critical design that questions three facets of robotics.
The first is aesthetics.
Most robots are white and LED-illuminated; others including myself have spilled many LaTeX templates over the downsides of this aesthetic conformity.
The second is utilitarianism.
No, Blossom won't fold your clothes or clean your room or wash your dishes, but then again, no robot short of unobtainable research prototypes can.
The third is consumption of robots.
Consumer robots are advertised and sold as 
Apart from the inherent ills of advertisement which needs no further bludgeoning, the marketing of robots performing physical or mental feats way above their actual capabilities in overproduced promotional videos is actively hurting robot development.

## Troubleshooting

### Motor settings
Setting motor info e.g. IDs needs torque to be disabled.
For example, to set the ID of motor 1 to 7 in using `r0b0.scripts.motor_calib.py`:
```
set_param('torque_enable',{1:False})
set_param('id',{1:7})
```