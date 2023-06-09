# Gadgets

Gadgets, like Rigs, are defined through `*.yaml` files.
`type` is the universally required field and defines the class of Gadget to instantiate from `r0b0.gadgets`.

```
type: Gadget
```

Each `Gadget` requires more fields that act as the `kwargs` when instantiating their respective objects.
The following sections describe each `Gadget`'s fields through example.

## `DynamixelRobot`

A `DynamixelRobot` is a robot constructed of Dynamixel motors.
The control is built over [my fork of rdiverdi's `dynamixel_python` wrapper library](https://github.com/psychomugs/dynamixel_python) over the official [`dynamixel_sdk`](https://github.com/ROBOTIS-GIT/DynamixelSDK).
The `Gadget` fields include the `usb_port` and `baud_rate` of the [U2D2 communication converter](https://emanual.robotis.com/docs/en/parts/interface/u2d2/), and the description of the attached motors.
This is the configuration for the Blossom robot (`blsm_dxl.yaml`):
```
usb_port: /dev/tty.usbserial-FT1SF1UM
baud_rate: 57600
motors: 
- name: tower_1
  model: xl330-m288
  id: 1
- name: tower_2
  model: xl330-m288
  id: 2
- name: tower_3
  model: xl330-m288
  id: 3
- name: base
  model: xl330-m288
  id: 4
```

## `MIDIController`

Connect your MIDI controller through USB or bluetooth.
Define the `port_name` as listed in your MIDI settings (e.g. on macOS, the 'Audio MIDI Setup' app).
For a MIDI device that shows up as `MIDi Controller Name`:
```
type: MIDIController
port_name: MIDI Controller Name
```

## `ArduinoGadget`

`ArduinoGadget`s are enabled through [pyFirmata](https://github.com/tino/pyFirmata).
First, upload `StandardFirmata.ino` to the board.
Create a `{config}.yaml` with the board's `usb_port` and `baud_rate`.

```
type: ArduinoGadget
usb_port: /dev/cu.usbserial-ADAQDbKpQ
baud_rate: 115200
```

## `Page`

`Page` is a special type of `Gadget` that serves webpages.
Compared to other Gadgets, `Page` hooks more closely to the `Host`, the medium/server for the whole `Rig`.
Page defines how to route URLs and event emits.
```
type: Page
route_urls:
  /: index.html
  /someOtherPage: someOtherPage.html
event_kwargs:
  some_event:
    kwarg_1: arg_1
    kwarg_2: arg_2
```

### `MobilePage`

`MobilePage` is a subclass that features extra handlers for mobile-specific events, e.g. [`DeviceOrientation` events](https://developer.mozilla.org/en-US/docs/Web/API/Window/deviceorientation_event).

## `PiGadget`

`PiGadget`s can only run on Raspberry Pis.

## `PyGameGadget`

`PyGameGadget`s are enabled through [`pygame`](https://www.pygame.org) and enable real-time input (e.g. hardware joysticks, keyboard presses) and output (e.g. key emulation).
<!-- Because of PyGame's handling of events in loops, -->
Because `pygame` makes it difficult to handle events in non-main threads, using a PyGameGadget will make the overall `Rig` go into an event handler loop.

## `Mouse`

`Mouse` is a wrapper for [boppreh's `mouse` library](https://github.com/boppreh/mouse) and enables real-time mouse emulation, including motion and button presses.
*Note: there may be some macOS-specific installation issues regarding Darwing compatibility; [the solution here worked for me](https://github.com/boppreh/mouse/issues/75).*

## `Tape`

A `Tape` is a special `Gadget` that stores a stream of events as a `json`.
This is analogous to `rosbag`s but less bespoke to robots.

To start recording