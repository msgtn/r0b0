# `r0b0`
`r0b0` is a framework for interfacing between various human interface devices (HID).
Though this started out as a refactor of the codebase for [the Blossom robot that I developed during my PhD](http://psychomugs.github.io/research), I've expanded it into a generalizable and modular communication framework between input-output devices.

I've used this framework to power new iterations of Blossom and, more recently and orthogonally to robots, my [Leica MPi film-to-digital camera conversion](https://psychomugs.github.io/mpi).

Some design goals:
- Portability. Like Blossom's codebase, this framework should run virtually identically on a macOS laptop (how Blossom was wired most of the time), Linux computer, or Raspberry Pi. For the Leica MPi, the code runs on a Raspberry Pi Zero W, 
- Parity with Blossom's codebase. By the end of my PhD, Blossom could be remotely controlled through an `ngrok`-tunneled mobile webpage for the final 'phase' of my PhD: [variable-perspective telepresence evaluations](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=LzEyxcsAAAAJ&citation_for_view=LzEyxcsAAAAJ:0EnyYjriUFMC). 
- Extensibility. I only add new gadgets as they are available to me, e.g. MIDI controllers I own, joysticks, my Nintendo Switch controllers, etc. Adding new gadgets should be as simple as defining new classes in `r0b0.gadgets` and new configs in `config.gadgets`.
- Modularity through ignorance. Each Gadget neither knows nor cares about the other Gadgets in the network. Communication is facilitated by user-defined translation functions that connect Gadgets.

## Structure
The metaphor I held in mind comes from connecting musical instruments (`Gadget`s) into performance setups (`Rig`s) wired with `Cable`s.
HID devices ('`Gadget`s') connect to form `Rig`s, [socket](https://socket.io)-connected networks of devices that emit and handle events.
Gadgets are unaware of each other.
The interesting bits happen in `Cable`s: small functions that translate information from input events into instructions for output events.
For example, a `cc2motor` Cable would translate a `midi_cc` event from the turning of a MIDI controller's knob into a `motor_position` event to actuate a robot's motor.
Building a Rig consists of:
- Creating Gadget classes.
- Defining Gadget configs.
- Defining Rig configs.
- Creating Cable functions.

The following subsections provide overviews of each component; more detail and technical information are available within each module's respective README (e.g. `r0b0/gadgets/README.md`) and in `/docs`.

## Gadgets
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


### Rigs
Rigs are combinations of `Gadget`s connected to a `Host`, with messaging functions defined throughout the network.
Similar to `Gadget`s, `Rig` configurations are stored as `*.yaml` files within `config/rigs`.

### Host
The `Host` is the server through which Gadgets connect and communicate.
As the 'medium' of the Rig, the Host translates events through message functions.
Host subclasses [`flask_socketio.SocketIO`](https://flask-socketio.readthedocs.io/en/latest/getting_started.html#initialization) and acts as the central server/"medium" through which the devices communicate.
The [Flask-SocketIO](https://flask-socketio.readthedocs.io/en/latest/getting_started.html#initialization) and [Flask](https://flask.palletsprojects.com) docs have more information on how to route URLs and handle events.
The Host should be

e.g. defining the cc2motor comm
```
def cc2motor(midi_cc_msg):
	# do stuff using msg
	return motor_msg
```

### Cables
Cables (`r0b0.cables`) are the input-output translators for events.
They translate information from an input event into instructions for an output event.
They accept and return `dict`s in the form of [`socketio` events sent through `emit`s](https://python-socketio.readthedocs.io/en/latest/client.html#emitting-events).
The basic skeleton for `Cable` function is:

```
def input2output(data=None):
	# boilerplate to define the input event that triggers this function
  if data is None:
		return {'event':'input_event'}

	# perform 'translation'
  output_data = translation_function_that_outputs_a_dictionary(data)

	# output the data dictionary
	return {
		'event':'output_event',
    'data':output_data
		# add more arguments as needed
	}
```

Alternatively, combine the incoming data with the translated output data by using a dictionary update:
```
...
  data.update(translation_function_that_outputs_a_dictionary(data))

  return {
    'event':output_event,
    'data':data
  }
...
```

To connect `Gadget`s through a `Cable`, define the message function in any file in `r0b0/cables`, the input/transmitting `Gadget` (`tx_gadget`), and the output/receiving `Gadget` (`rx_gadget`) in the `Rig`'s `config.yaml`:
```
gadgets:
- inputGadget
- outputGadget
cables:
- msg_func: input2output
  tx_gadget: inputGadget
  rx_gadget: outputGadget
```

In this example, the rig will connect `inputGadget` to `outputGadget` through the `input2output` function.
The rig will listen to `input_event`s emitted by `inputGadget`, perform the 'translation' through `input2output`, then emit `output_event`s to `outputGadget`.


### Environment setup
Set up [conda](https://conda.io), then set up a conda environment and install some other dependencies with `pip` (because of issues with [`mouse`](https://github.com/boppreh/mouse/issues/75)). Docker maybe coming soon (maybe).
```
conda env create -f env.yaml
conda activate r0b0
pip3 install -r req.txt 
```


### Tapes
Continuing the music instrument metaphor, records of events can be saved to `Tape`s.
`Tape`s are raw `*.json` files that are lists of 
The `Tape` recorder is integrated into `Host`.
TODO - check over the terminology here
The recording `Gadget` must send a `record` event to the server.
TODO - is this integrated yet?
Playback of `Tape`s should work without the original recording `Gadget` being connected to the current `Rig`.
`Tape`s are saved with the naming convention: `{date}_{event_name}.json`.

## TODO
### Software
- [ ] Loopback events - events to self?
  - [ ] Maybe best to restrict to one gadget per name - make it explicit 
  - [ ] Just use new gadget configs as necessary
- [ ] DynamixelRobot - motor movement events
  - [ ] Implement motor events when moved
  - [ ] clean redundant functions
- [ ] Message functions - rebrand to `Cables`, clarify that can use multiple files in `r0b0/cables`
  - [ ] Split into separate files, in `msg_funcs.__init__.py` import all of the functions in every file in that folder
- [ ] Docker Image
- [ ] PyGame event loop in Rig - use pygame.event.set_allowed() to filter queueable events
- [ ] Occasional/eventual disconnects between mobile app controller and broadcaster
- [ ] In blsm_controller.js, enable reading the socketAddr from some file, i.e. start ngrok, save the tunneling url to a text file, read this tunneling url 
- [ ] Clean controller.js, broadcast.js
  - [ ] How to split up javascript files?
- [ ] Blossom Yaw is negated

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

