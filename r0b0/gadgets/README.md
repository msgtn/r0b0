# Gadgets

Gadgets, like Rigs, are defined through `*.yaml` files.
All Gadgets need two fields: `name` and `type`.
Actually, only `type` should be necessary; `r0b0.utils.loaders.load_config` should pull the config file name (e.g. `gadgetName` from `gadgetName.yaml`).
`type` defines the class of Gadget to instantiate from `r0b0.gadgets`.

```
name: testGadget
type: Gadget
```

Each Gadget requires more fields that act as the `kwargs` when instantiating their respective objects.
The following sections define these through example.

## DynamixelRobot

A `DynamixelRobot` is a robot constructed of Dynamixel motors.
The Gadget fields include the USB port (where the [U2D2 communication converter](https://emanual.robotis.com/docs/en/parts/interface/u2d2/) is plugged into) and baud rate, and the motors.
```
usb_port: /dev/tty.usbserial-FT1SF1UM
baud_rate: 57600
motors: # use these same names as kwargs to constructors
- name: tower_1
  model: xl330-m288
  id: 1
  position:
    reset: 0
    min: -40
    max: 140
- name: tower_2
  model: xl330-m288
  id: 2
  position:
    reset: 0
    min: -40
    max: 140
- name: tower_3
  model: xl330-m288
  id: 3
  position:
    reset: 0
    min: -40
    max: 140
- name: base
  model: xl330-m288
  id: 4
  position:
    reset: 0
    min: -140
    max: 140
```

## MIDI controller

## ArduinoBot

## Phone

`Phone` is a special type of `Gadget` that serves webpages.
Compared to other Gadgets, `Phone` hooks more closely to the `Host`, the medium/server for the whole `Rig`.
Phone defines how to route URLs and event emits.
```
route_urls:
  /: index.html
  /broadcast: broadcast.html
event_kwargs:
  device_motion:
    namespace: /phone
```
TODO - maybe this should be renamed to Browser or Page?
TODO - can namespace be folded into 


## Blossom App - Web Browser Version

Minimal functionality version of the app that runs in a browser at `blossombot.com` (redirects to `https://blossomapp.ngrok.io`)

## Local Development

Requires Node.js and npm (tested using Node.js 8.x and npm 5.x).

Start `ngrok`, get the control address
```bash
$ ./ngrok.sh 
$ 
$ npm i
$ npm start
```


Start `ngrok`:
```bash
$ ./ngrok.sh
```

In a new tab, get the control address:
```bash
$ ./get_ctrl.sh
```

In the `blossom` directory (separate repo) sure the robot is running on `localhost` on port `4000`:
```bash
$ python3 start.py -n woody -b --host localhost --port 4000
```
If not connected to a robot (for testing purposes), replace `woody` with `test`.

Start the server:
```bash
$ npm start
```

By default, the application will serve on port `8000` and is thus accessible from `https://blossomapp.ngrok.io`. To start and configure the video stream, go to `https://localhost:8000/broadcast.html`.

Access `https://blossomapp.ngrok.io` from a mobile device, enter the password (probably `test`), and take `Control` of the robot.

## Troubleshooting

Running on `https` is important; `DeviceMotion` doesn't work without it, and permissions must be given if on iOS.