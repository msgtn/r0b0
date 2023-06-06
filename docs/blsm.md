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
```
sudo openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout r0b0/key.pem -out r0b0/csr.pem
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