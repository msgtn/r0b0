# Blossom

This page goes over how to build and boot a Blossom robot.

## Build

Build the robot.
Files and instructions coming soon.

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

### `blsm` rig
Start the `blsm` rig, which contains the `blsm_dxl` robot as a `DynamixelRobot` and the `bslm_phone` browser-based interface as a `Page`. The `motion2motor` cable translates `device_motion` events from the page (when accessed from a mobile browser) into `position` events for the motor.
```
python3 start.py blsm_dxl
```

### Control

*This has only been tested on iOS.*

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

In a mobile browser (e.g. Safari), navigate to the forwarding URL (`https://someRandomLettersAndNumbers.ngrok.app` in the above example). 

### Telepresence with video

Connect a USB webcam to your computer.

With the prior scripts running (`start.py` and the `ngrok` tunnel), on the desktop/laptop computer controlling the robot, navigate to `https://localhost:8080/broadcaster`.

This page contains the controls for WebRTC media sources.
Select the connected webcam in the dropdown, which should begin a video feed on the page.
The video should also appear on the mobile device connected to the `ngrok` page, though you may need to refresh or press the 'play' icon (▶️).

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