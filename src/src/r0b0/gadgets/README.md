# Gadgets

Gadgets represent devices or software that send/receive messages.
Gadgets subclass `socketio.Client`, which means that they connect to a `socketio.Server`  — the Rig — to emit and handle events.
Gadgets are defined through `*.yaml` files.
`type` is the universally required field and defines the class of Gadget to instantiate from `r0b0.gadgets`.

```
type: Gadget
```

## `Message`s

Gadgets usually emit messages as `dict`ionaries, but sometimes require more complicated payloads.
An example is the MIDI message from the [`mido.Message` object](https://mido.readthedocs.io/en/latest/messages.html), which handles a lot of internal MIDI-specific stuff.
`socketio` can't send these objects and breaking out the object's fields on every emit/handle would be redundant.
If these bespoke objects are required, they are [`pickle`d](https://docs.python.org/3/library/pickle.html) into a `data['msg']` field in the payload.
The base `Gadget` class has a base `Message` class that is just a dictionary wrapper to enable extensibilty with other custom Gadgets.

```
class Message(object):
    def __init__(self, *args, **kwargs):
        self.__dict__.update(**kwargs)

class Gadget(Client, Thread):
    def __init__(self, config: dict, **kwargs):
        ...
        self.message = Message
```

### Emitting

For example, to emit a random MIDI note from within a Gadget object:
```
  def emit_random_note(self,):
    dict_to_emit = {
      'event':'midi_on',
      'data':{
        'msg':mido.Message('note_on',random.choice(range(40,60))),
      },
      'namespace':self.namespace
    }
    self.emit(self,**dict_to_emit)
```
This will call the `Gadget.emit` function, which uses the `@encode_msg` decorator to `pickle.dump` whatever is it `data['msg']`.
The function definition is [here](/r0b0/utils/loaders.py) — it essentially calls `data['msg']=pickle.dumps(data['msg'])` if `data` does have a `'msg'` to unpack.
If circumventing the `Gadget.emit` function in any Gadget subclass by calling `Client.emit` directly, make sure to add the `@encode_msg` decorator.
```
  # inside a custom Gadget class
  @encode_msg
  def emit(self,event,data,**kwargs):
    ...
    Client.emit(self, event, data, **kwargs)
```

### Handling

Gadget handler functions should be decorated with `@decode_msg` to `pickle.load` any potential messages.
The function definition is [here](/r0b0/utils/loaders.py) — like the inverse of `@encode_msg`, it calls `data['msg']=pickle.loads(data['msg'])` if `data` does have a `'msg'` to pack.
```
  # inside a custom Gadget class
  @decode_msg
  def event_handler(self,data):
    msg = data['msg']
    # msg will be of type self.message
```

### Usage note
You may not need to use `pickle`-ing at all if the `data` messages can just be plain `dict`s without `'msg'` fields. 
It's still recommended to use these decorators in case `pickle`-ing becomes useful in the future.

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
Compared to other Gadgets, `Page` hooks more closely to the server of the Rig.
Page defines how to route URLs and event emits.
```
type: Page
route_urls:
  /: index.html
  /somepage: somepage.html
  /someotherpage: someotherpage.html
event_kwargs:
  some_event:
  some_other_event:
    kwarg_1: arg_1
    kwarg_2: arg_2
```
For now, the actual html pages are pretty spartan and written in 
Every event that the webpage emits that should be handled by the backend should be added to `event_kwargs`.
Arguments are packaged into `data['kwargs']` and are optional, e.g. `some_event` does not need additional arguments and is thus blank.
<!-- Note that arguments are optional, e.g. `some_event` has -->

When a Page is added, the Rig will use `route_urls` to call [`Flask.add_url_rule`](https://tedboy.github.io/flask/generated/generated/flask.Flask.add_url_rule.html) and `event_kwargs` to set event handlers with [`socketio.Server.on`](https://python-socketio.readthedocs.io/en/latest/server.html#event-callbacks).

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

The `rig` functions as a tape player with functionality to record, load, and play tapes.
### Recording
To start recording, send a `record` event with the `event` to record, e.g. as seen in Blossom's `r0b0/rigs/static/controller.js`:
```
socket.emit("record", {
    record: recording,        // boolean to start (True) or stop (stop)
    event: "device_motion",   // the event to record
    id: socket.id,            // a unique identifier for the gadget recording, used by the recorder
  });
```
In the Blossom interface, press the large red 'record' button in the center to start recording, move the phone around, and press the 'record' button again to stop.
This will save a time-labelled stream of `device_motion` events to `tapes/20230720_device_motion.json`.

### Loading
In the command line interface:
```
rig.on_load({'tape_name':'20230720_device_motion.json'})
```

### Playing
In the command line interface:
```
rig.on_play({'tape_name':'20230720_device_motion.json'})
```

### TODO

The docs for this are a work in progress - a TODO is to create a higher-level interface than calling the `rig` functions directly.
Also, a kludge is that playback requires the original gadget — the one that emitted the events that are recorded — to be connected during playback.
Another TODO is to spoof this gadget, e.g. recording a sequence with a midi controller, then playing it back without the original midi controller connected.
The hinges on the Cable functions working without the Gadgets connected — maybe this is already supported? The Cable functions don't need to know whether the Gadgets are connected in the first place; all they are concerned with is the input/output namespace.
This should be fine with the `blsm` configuration that only has the `blsm_phone` and `blsm_dxl` gadgets, as the `blsm_phone` gadget does not require any hardware.