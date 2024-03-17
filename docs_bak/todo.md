

## TODO
'rev of blsm'
### Software
- [ ] Loopback events - events to self?
  - [ ] Maybe best to restrict to one gadget per name - make it explicit 
  - [ ] Just use new gadget configs as necessary
- [ ] DynamixelRobot - motor movement events
  - [ ] Implement motor events when moved - get/set `moving_threshold`
  - [ ] clean redundant functions
- [ ] Docker Image
- [ ] PyGame event loop in Rig - use pygame.event.set_allowed() to filter queueable events
- [ ] Occasional/eventual disconnects between mobile app controller and broadcaster
- [ ] In blsm_controller.js, enable reading the socketAddr from some file, i.e. start ngrok, save the tunneling url to a text file, read this tunneling url 
- [ ] Clean controller.js, broadcast.js
  - [ ] How to split up javascript files?
  - [ ] Combine redundant funtions (e.g. `getDevices`, `gotStream`)
  - [ ] scratch-based interface with React?
- [ ] Blossom
  - [ ] Yaw is negated
  - [ ] vertical height control using scrolling, like in a news feed
- [ ] Test blsm_dxl and blsm_ard at the same time
- [ ] MediaPipe implementation
- [ ] Gradio webui
- [ ] simplify to a cmdline arg
  - [ ] e.g. `rconnect sender receiver`
  - [ ] Add functionality to start just a single gadget, e.g. `python3 start.py single_gadget`, when `config/rigs/single_gadget.yaml` does not exist but `config/gadgets/single_gadget.yaml` does, will create a dummy rig config with `{gadgets: [single_gadget,],}`
- [ ] Test setting the operating mode on start
- [ ] Need better way to handle impotent events
  - [ ] e.g. moving a joystick in other axis when you only want something to happen when adjusting pitch

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

### Rig ideas
- midi to keyboard/mouse
  - Etch-a-sketch controls
- Tape player 
  - Inspiration from TE OB4, motorized faders
  - play -> move motor
  - physical manipulation

