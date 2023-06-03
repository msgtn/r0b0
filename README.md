# `r0b0`
`r0b0` is a framework for interfacing between various human interface devices (HID).
Though this started out as a refactor of the codebase for [the Blossom robot that I developed during my PhD](http://psychomugs.github.io/research), I've expanded it into a generalizable and modular communication framework between input-output devices.

I've used this framework to power new iterations of Blossom and, more recently and orthogonally to robots, my [Leica MPi film-to-digital camera conversion](https://psychomugs.github.io/mpi).

Some design goals:
- Portability. Like Blossom's codebase, this framework should run virtually identically on a macOS laptop (how Blossom was wired most of the time), Linux computer, or Raspberry Pi. For the Leica MPi, the code runs on a Raspberry Pi Zero W, 
- Parity with Blossom's codebase. By the end of my PhD, Blossom could be remotely controlled through an `ngrok`-tunneled mobile webpage for the final 'phase' of my PhD: [variable-perspective telepresence evaluations](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=LzEyxcsAAAAJ&citation_for_view=LzEyxcsAAAAJ:0EnyYjriUFMC). 
- Extensibility. I only add new gadgets as they are available to me, e.g. MIDI controllers I own, joysticks, my Nintendo Switch controllers, etc. Adding new gadgets should be as simple as defining new classes in `r0b0.gadgets` and new configs in `config.gadgets`.
- Modularity through ignorance. Each Gadget neither knows nor cares about the other Gadgets in the network. Communication is facilitated by user-defined message functions.

## Structure
The metaphor I held in mind comes from connecting musical instruments into performance setups (which I often found more fun than actually playing the instruments).
HID devices ('`Gadget`s') connect to form `Rig`s, [socket](https://socket.io)-connected networks of devices that emit and handle events.
Gadgets are unaware of each other.
The interesting bits happen in message functions: small functions that translate information from input events into instructions for output events.
By example, a `cc2motor` message function would translate a `midi_cc` event from the turning of a MIDI controller's knob into a `motor_position` event to actuate a robot's motor.

The following subsections provide overviews of each component; more detail and technical information are available within each module's respective README (e.g. `r0b0/gadgets/README.md`)

### Gadgets
A `Gadget` is a modular component that represents devices: robots (e.g. [Dynamixel motors](https://www.robotis.us/dynamixel-xl330-m288-t/), Arduino-controlled systems through PyFirmata), MIDI controllers (e.g. [Akai MPK Mini](https://www.akaipro.com/mpk-mini), [Teenage Engineering OP-Z](https://teenage.engineering/products/op-z)), phones (through browsers).
`Gadget`s subclass `socketio.Client` and connect to the `Host`, through which they pass their respective `Messages` to other `Gadget`s.
Gadget configurations are stored as `*.yaml` files within `config/gadgets`.

Available gadgets and subgadgets:
- DynamixelRobot
- Arduino through [pyFirmata](https://github.com/tino/pyFirmata)
  - ArduinoRobot
- MIDIController through [mido](https://mido.readthedocs.io/en/latest/)
- PiGadgets
  - PiButton (gpiozero)
  - PiCamera (picamera2)
- [PyGameGadgets](https://www.pygame.org/docs/)
  - PyGameJoystick (physical joysticks e.g. LogiTech Extreme 3D Pro, Nintendo Switch Joy-cons)
  - PyGameKeys (physical keyboard, can emulate keyboard events)
- [Mouse](https://github.com/boppreh/mouse): performing cursor movements

### Host
The `Host` is the server through which Gadgets connect and communicate.
As the 'medium' of the Rig, the Host translates events through message functions.
Host subclasses [`flask_socketio.SocketIO`](https://flask-socketio.readthedocs.io/en/latest/getting_started.html#initialization) and acts as the central server/"medium" through which the devices communicate.
The [Flask-SocketIO](https://flask-socketio.readthedocs.io/en/latest/getting_started.html#initialization) and [Flask](https://flask.palletsprojects.com) docs have more information on how to route URLs and handle events.
The Host should be

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
Message functions (`r0b0.messages.msg_funcs.py`) are the input-output translators for events.
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
python3 -m r0b0.rigs.server
```

Start an example rig with `blossom` and `opz`:
```
python3 -m start dec
```

Test
```
python3 -m unittest discover -s r0b0.gadgets -t .
```


## TODO
### Software
- [ ] event:callback is 1:1 - need to enable 1:many 
- [ ] Arduino
  - [ ] Need to upload StandardFirmata to arduino first
- [ ] Check if underscores in namespaces are fine
- [ ] Loopback events - events to self?
  - [ ] Maybe best to restrict to one gadget per name - make it explicit 
  - [ ] Just use new gadget configs as necessary
- [ ] DynamixelRobot - motor movement events
  - [ ] Implement motor events when moved
  - [ ] clean redundant functions
- [ ] Message functions
  - [ ] Rebrand to `rubber bands`?
  - [ ] Split into separate files, in `msg_funcs.__init__.py` import all of the functions in every file in that folder
- [ ] Docker Image


### Document
- Gadgets
  - Arduino - pyFirmata setup
  - DynamixelRobot - how to define motor config.yaml
  - Page - routing and events, integration with Host/Server
- ngrok
  - setting up an account
  - starting tunnel

#### cleanup
- [ ] consolidate gadgets
- [ ] Arduino: ArduinoServos, ArduinoButtons
### Docs
- [ ] Add diagram


# Notes

### 20230104
controller.js
what is it
what's in it
- DeviceOrientation emitter
  - Needs to get permission

### 230530
- Instructions
	- Base
	- Spine
		- 3x
		- connect
	- Head

Steps for setting up r0b0 on an rpi
- Download repo
- Define configs
	- Gadgets
	- Rigs
- message functions

## Cite / Ideas
- https://www.theverge.com/23539916/sony-ps5-accessibility-controller-leonardo-ces-2023
- https://www.ifixit.com/News/51614/framework-laptop-teardown-10-10-but-is-it-perfect
- Robots as bicycles, cars, things that communities can grow from, are utilitarian, but also usable for pure Play
  - the bicycle of the _______

