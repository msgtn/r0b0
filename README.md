# Blossom

## Structure
- [ ] Add diagram
### Host
The `Host` subclasses `socketio.Server` and acts as the central "medium" through which Gadgets communicate.
### Gadgets
A `Gadget` is a modular component that represents a devices: robots, MIDI controllers, phones (through browsers).
`Gadget`s subclass `socketio.Client` and connect to the `Host`, passing respective `Messages` throughout the network.

### Configurations
Gadget configurations are stored as `*.yaml` files.

### Rigs
Rigs are combinations of `Gadget`s connected to a `Host`, with messaging functions defined throughout the network.
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
python3 -m src.gadgets.server
```

Start an example rig with `blossom` and `opz`:
```
python3 -m src.rigs.dec
```

Test
```
python3 -m unittest -s src.gadget -t .
```

## Notes
20230104
controller.js
what is it
what's in it
- DeviceOrientation emitter
  - Needs to get permission