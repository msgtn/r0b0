# `r0b0`
`r0b0` is a modular system for interfacing between various human interface devices.

## Structure
The network of devices is held together through [WebSockets](https://socket.io).
### Gadgets
A `Gadget` is a modular component that represents devices: robots (e.g. [Dynamixel motors](https://www.robotis.us/dynamixel-xl330-m288-t/), (soon) Arduino-controlled servos), MIDI controllers (e.g. [Akai MPK Mini](https://www.akaipro.com/mpk-mini), [Teenage Engineering OP-Z](https://teenage.engineering/products/op-z)), phones (through browsers).
`Gadget`s subclass `socketio.Client` and connect to the `Host`, through which they pass their respective `Messages` to other `Gadget`s.
Gadget configurations are stored as `*.yaml` files within `config/gadgets`.

Available gadgets and subgadgets:
- DynamixelRobot
- Arduino
  - ArduinoRobot
- MIDI controller
- PiGadgets
  - PiButton (gpiozero)
  - PiCamera (picamera2)
- PyGameGadgets
  - PyGameJoystick (physical joysticks e.g. LogiTech Extreme 3D Pro, Nintendo Switch Joy-cons)
  - PyGameKeys (physical keyboard, can emulate keyboard events)

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

### Message functions ('Bands?')
Message functions (`src.messages.msg_funcs.py`) are the input-output translators for events.
The basic skeleton for a message function is:

```
def input2output(data=None):
	# define the input event,
	# i.e. the event that triggers this function
    if data is None:
		return {'event':'input_event'}
	# perform 'translation'
	# output the data dictionary
	return {
		'event':'output',
		# add more arguments as needed
	}
```

To use a message function, define the message function and input/transmitting (`tx`) and output/receiving (`rx`) gadgets in the rig config:
```
name: joyblossom
gadgets:
- inputGadget
- outputGadget
messages:
- msg_func: input2output
  tx_gadget: inputGadget
  rx_gadget: outputGadget
```

In this example, the rig will connect `inputGadget` to `outputGadget` through the `input2output` function.
The rig will listen to `input_event`s emitted by `inputGadget`, perform the 'translation' through `input2output`, then emit `output_event`s to `outputGadget`.

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


国際的訳すること

## TODO
### Software
- [ ] event:callback is 1:1 - need to enable 1:many 
- [ ] Arduino
  - [ ] Need to upload StandardFirmata to arduino first
- [ ] Check if underscores in namespaces are fine
- [ ] Loopback events - events to self?
  - [ ] Maybe best to restrict to one gadget per name - make it explicit 
  - [ ] Just use new gadget configs as necessary

#### cleanup
- [ ] consolidate gadgets
- [ ] Arduino: ArduinoServos, ArduinoButtons
### Docs
- [ ] Add diagram
- [ ] 