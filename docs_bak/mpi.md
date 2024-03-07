# MPi

## Hardware
- Raspberry Pi Zero
- [Raspberry Pi HQ Camera](https://www.raspberrypi.com/products/raspberry-pi-high-quality-camera/)
- [WaveShare 1.3" LCD HAT](https://www.waveshare.com/wiki/1.3inch_LCD_HAT)

### Wiring

The shutter will trigger according to the `shutter` pin defined at [`mpi_button.yaml`](/config/gadgets/mpi_button.yaml), currently set to 12.
I connected this pin to the center pin of the Leica's flash sync socket.
To complete the circuit, wire the ground to the camera body.
You could also wire this to a standard push button.

Connect the camera and LCD hat as normal.

## Rig

The `Rig` definition is at [`mpi.yaml`](/config/rigs/mpi.yaml).
The `mpi_button` [`PiButton`](/r0b0/gadgets/pi_button.py) `Gadget` sends button messages to the `mpi_camera` [`PiCamera`](/r0b0/gadgets/pi_camera.py) `Gadget`.
The LCD module's buttons are hardcoded to control shutter speed; you can look into this at the [`PiCamera`](/r0b0/gadgets/pi_camera.py) configuration.

### `pi_camera`

## Starting the script

From the top-level directory, start the `mpi` `Rig`:
```
python3 start.py mpi
```

[You can set up a service to start this script on booting](https://gist.github.com/emxsys/a507f3cad928e66f6410e7ac28e2990f).
The startup service I use is [`mpi.service`](/r0b0/services/mpi.service).