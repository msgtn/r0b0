# `r0b0`
`r0b0` is a modular system for interfacing between various human interface devices.

## Structure
The network of devices is held together through [WebSockets](https://socket.io).
### Gadgets
A `Gadget` is a modular component that represents devices: robots (e.g. [Dynamixel motors](https://www.robotis.us/dynamixel-xl330-m288-t/), (soon) Arduino-controlled servos), MIDI controllers (e.g. [Akai MPK Mini](https://www.akaipro.com/mpk-mini), [Teenage Engineering OP-Z](https://teenage.engineering/products/op-z)), phones (through browsers).
`Gadget`s subclass `socketio.Client` and connect to the `Host`, through which they pass their respective `Messages` to other `Gadget`s.
Gadget configurations are stored as `*.yaml` files within `config/gadgets`.
### Host
The `Host` subclasses [`flask_socketio.SocketIO`](https://flask-socketio.readthedocs.io/en/latest/getting_started.html#initialization) and acts as the central "medium" through which the devices communicate.
The [Flask-SocketIO](https://flask-socketio.readthedocs.io/en/latest/getting_started.html#initialization) and [Flask](https://flask.palletsprojects.com) docs have more information on how to route URLs and handle events.

### Rigs
Rigs are combinations of `Gadget`s connected to a `Host`, with messaging functions defined throughout the network.
Similar to `Gadget`s, `Rig` configurations are stored as `*.yaml` files within `config/rigs`.

e.g. defining the cc2motor comm
```
def cc2motor(midi_cc_msg):
	# do stuff using msg
	return motor_msg
```

## Scripts
All from the top-level directory.

Start just a server:
```
python3 -m src.rigs.server
```

Start an example rig with `blossom` and `opz`:
```
python3 -m start dec
```

Test
```
python3 -m unittest discover -s src.gadgets -t .
```

## Notes
20230104
controller.js
what is it
what's in it
- DeviceOrientation emitter
  - Needs to get permission


## TODO
### Software
- [ ] event:callback is 1:1 - need to enable 1:many 
- [ ] Arduino
  - [ ] Need to upload StandardFirmata to arduino first
### Docs
- [ ] Add diagram
- [ ] 