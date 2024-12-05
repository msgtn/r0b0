---
tags: daily_note_241204
creation_date: 241204_152436
title: 241204_152436
allDay: false
startTime: 15:24
endTime: 15:24
date: 2024-12-04
completed: null
---

# Instructions

*Last modified: 241205*

These instructions are for replicating the [Leica MPi](https://msgtn.xyz/mpi).
This is an intermediate-level project and requires familiarity with:
- Physical fabrication: 3D printing, soldering
- Microcomputers: Raspberry Pi, Linux, Python
- Photography: basic concepts of exposure and focusing

These instructions are very much a living document.
Please post general questions as [issues in this repository](https://github.com/msgtn/r0b0/issues).
For detailed / business inquiries or direct commissions, please [contact me directly](mailto:m.jp.sgtn@icloud.com).

## Hardware

### Bill of materials

| Item                          | Description                                        | Quantity | Approx. cost | Link | Notes                                                                |
| ----------------------------- | -------------------------------------------------- | -------- | ------------ | ---- | -------------------------------------------------------------------- |
| Raspberry Pi Zero 2           | Main computer                                      | 1        | $15          | [Purchase](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/)     |                                                                      |
| Waveshare LCD module w/ D-Pad | Interface                                          | 1        | $15          | [Purchase](https://www.waveshare.com/1.3inch-lcd-hat.htm)     |                                                                      |
| PiSugar                       | Internal power supply                              | 1        | $40          | [Purchase](https://www.tindie.com/products/pisugar/pisugar-3-battery-for-raspberry-pi-zero/)     | Any Pi Zero-mountable model; recommended is the 3 for USB-C charging |
| M2x8 bolts                    | For assembling the housing                         | 5        | <$1          |      |                                                                      |
| M2 nuts                       | For assembling the housing and sync cable adapter  | 3        | <$1          |      |                                                                      |
| M2x4(?) bolts                 | For mounting the sensor                            | 2        | <$1          |      |                                                                      |
| Anti-static foam              | Spring for mounting sensor                         | 1        | <$1          |      | Commonly found on components with legs e.g. ICs, microcontrollers    |
| Wires                         | For wiring the sync cable                          | 2        | <$1          |      |                                                                      |
| 1x20 male header pins          | For stacking the LCD hat                           | 1        | <$1          |      |                                                                      |
| 1x20 right-angle female header | For stacking the LCD while exposing the Pi's GPIOs | 1        | <$1          | [Purchase](https://www.digikey.com/en/products/detail/sullins-connector-solutions/PPPC201LGBN-RC/775953)     |                                                                      |


Choose one of the following sensors:

| Item                          | Resolution | Crop factor | Approx. cost | Link | Notes                        |
| ----------------------------- | ---------- | ----------- | ------------ | ---- | ---------------------------- |
| Raspberry Pi HQ Camera Module | 12MP       | 5.5x        | $50          | [Purchase](https://www.raspberrypi.com/products/raspberry-pi-high-quality-camera/)     | Official first-party product | 
| Arducam OwlSight              | 64MP       | 3.7x        | $50          | [Purchase](https://www.arducam.com/product/arducam-1-1-32-64mp-autofocus-camera-module-for-raspebrry-pi/)     | Requires disassembly         |

### Print

| Item               | Description                                                        | Link | Notes    |
| ------------------ | ------------------------------------------------------------------ | ---- | -------- |
| Housing            | Main housing                                                       | [Print](https://github.com/msgtn/r0b0/blob/mpi/docs/assets/mpi/mpi-housing.stl)     |          |
| Back plate         | Mounts to the back of the camera                                   | [Print](https://github.com/msgtn/r0b0/blob/mpi/docs/assets/mpi/mpi-back-plate-arducam.stl)     |          |
| Top cover          | Protects top of module, slides next to GPIO header                 | [Print](https://github.com/msgtn/r0b0/blob/mpi/docs/assets/mpi/mpi-top-cover.stl)     | Optional |
| Side cover         | Protects SD card                                                   | [Print](https://github.com/msgtn/r0b0/blob/mpi/docs/assets/mpi/mpi-side-cover.stl)     | Optional |
| Sync cable adapter | Syncs mechanical and electronic shutters through flash sync socket | [Print](https://github.com/msgtn/r0b0/blob/mpi/docs/assets/mpi/mpi-plug.stl)     |          |



### Assembly

- Solder headers to the outer row of the Pi.
	- Solder the male header pins to the inner row (odd-numbered pins from 1-39), with the long end facing upwards from the Pi.
	- Solder the right-angle female headers to the outer row (even-numbered pins from 1-39). Solder from underneath, with the male ends facing upwards from the Pi (same as the header pins).
	- [ ] Solder from underneath, with the male ends of the pins facing upwards from the Pi
	- [ ] May need to shave off part of the header plastic
- Connect camera flex cable to the Pi.
	- Do not connect the camera module yet. Fold — but **do not crease** — the cable into a right angle such that the other end of the cable hangs past the bottom of the Pi (the edge with the USB and HDMI ports) with the contacts facing upwards.
	-  [ ] Diagram
- Stack the components
	- Mount the PiSugar to the bottom of the Pi with the included M2.5 screws. The PiSugar's spring-loaded pins should contact the relevant pins on the Pi. 
	- Mount the LCD module to the top of the Pi, onto the header pins. This should keep the camera cable folded.
- Insert the Pi into the housing.
	- The LCD module buttons should align with the cutouts on the housing. Ensure that the camera cable hangs out of the housing.
	- Optionally, slot the top and side covers over the headers and SD card slot, respectively.
- Attach back plate. 
	- First, fit 2x M2 nuts into the nut traps on the back plate.
	- Next, thread the camera cable through the bottom opening on the back plate.
	- Finally, affix the back plate to the housing with 4x M2x8 bolts.
- Mount sensor.
	- First, connect the camera cable to the sensor module and lock the pins.
	- Place antistatic foam over the nut traps. Lightly poke holes in the foam, over the nuts, to simplify threading.
	- Mount the sensor module, aligning the mounting holes to the nuts and squeezing the foam. Screw in the shorter M2 bolts, not all the way due to the battery right behind the nuts, but just enough to fasten the sensor. Tighten both sides evenly until the sensor board is parallel to the back plate.
- Assemble plug adapter
	- Place nut in rear end and thread bolt from the inner barrel.
	- Plug adapter into one of the flash sync sockets (*Note: AFAICT, the sockets are functionally identical for this use, but I plug into the left socket with the lightning symbol*).
	- Hand-unscrew the bolt by turning the thread clockwise until it stops. It should be contacting the center pin. To check, probe the bolt and ground (the outer edge of the socket or any exposed metal on the camera body) — pressing the shutter button should close the circuit, even if the shutter has not been armed by advancing the film lever.
	- Wrap an exposed end of wire around the bolt. Connect the other end to the Pi's GPIO 23 (pin 16, 8th from the left end of the row).
	- With another wire, affix one exposed end to ground (e.g. pinch it in the hot/cold shoe with a hot shoe cover) and insert the other end to any of the ground pins on the Pi (e.g. pin 14, 7th from the left end and next to the GPIO to the center pin).

### Installation
- Remove baseplate
- Remove film door
- Hinge the module in, lock with the baseplate
- Connect plug adapter

## Software

### Installation
- Flash Raspberry Pi image onto a microSD card.
	- The system has only been tested on the stock [Bookworm 64-bit](https://www.raspberrypi.com/software/operating-systems/#raspberry-pi-os-64-bit) OS flashed with the [official imager](https://github.com/raspberrypi/rpi-imager).
- Boot Pi.
	- Insert SD card into the Pi.
	- Turn on the Pi by either turning on the PiSugar (hold down the left button below the Pi's HDMI port until all four battery indicators and the Pi's green power LED illuminate) or plugging a micro-USB into the Pi.
- `ssh` into the Pi using the credentials you set in the imager.
- Set up Wi-Fi access point.
	- [This repo](https://github.com/pi-top/Wi-Fi-Access-Point-and-Station-Mode) will enable the Pi to broadcast an access point while connected to a local network. 
- Download and install [r0b0].
```
git clone https://github.com/msgtn/r0b0
cd r0b0
git checkout mpi
pip install -e .
pip install -r requirements/mpi.txt
```
- [ ] Update requirements with extra mpi-specific dependencies
- Set up the MPi service so the script starts on boot.
	- First, modify `r0b0/src/r0b0/services/mpi.service` to point `WorkingDirectory` with your username, e.g. if the output of `whoami` is `pi`, edit `WorkingDirectory=/home/pi/r0b0`.
```
# copy the services file and change its permissions
cp src/r0b0/services/mpi.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/mpi.service

# enable and start service
sudo systemctl daemon-reload
sudo systemctl enable mpi.service
sudo systemctl start mpi

# 
```


### Usage
- The service should start on boot.
	- Otherwise, manually start: `cd && cd r0b0 && python3 start.py --config ./config/rigs/mpi.yaml`
	- Only one process using the camera can be running at a time; ensure no other `python` processes are running (e.g. use `ps aux | grep python` to view process IDs and `sudo kill` them)
- Control the service with `sudo systemctl {command} mpi`, replacing `{command}` with one of:
	- `status`: get the status, e.g. if it is active or has failed
	- `start`: starts te service
	- `stop`: stops the service
	- `restart`: restarts the service
	- `enable` / `disable`: enable / disable the service starting on boot

### Taking photos
- Set shutter speed with the directional pad.
	- Up: 1/2
	- Left: 1/15
	- Down: 1/60
	- Right: 1/250
- Take photo
	- Set the camera's shutter speed to "Bulb," keeping the shutter open while the button is pressed.
	- Crank the film advance lever to arm the shutter.
	- Press the shutter button; hold for a second to ensure the electronic shutter completes the exposure.
- Viewing
	- Photos will be saved to `r0b0/tapes/` with the format `picam_YYMMDD_HHmmss`.
	- If the photos are completely or partially black, the shutter may be closing before the electronic shutter completes the exposure. Try holding the shutter button down longer to ensure the entire exposure is recorded.

### Calibration
For accurate focus, the sensor must sit at the same [flange distance](https://en.wikipedia.org/wiki/Flange_focal_distance) as the original film plane.
On first assembly, the sensor will likely be out of calibration.
The bolts and springy anti-static foam adjusts the flange distance: 
- Tightening / screwing the bolts clockwise will move the sensor away from the front of the camera, increasing the flange distance.
- loosening / unscrewing the bolts counter-clockwise will move the sensor toward the front of the camera, decreasing the flange distance.

To calibrate, select an object and take focus-bracketed photos at three distances: in focus, slightly farther, and slightly closer.
For example, if focusing on the object places the focus at 1.5m according to the lens markings, take another photo at 1m, and another at 2m.

- If the correctly focused photo is the sharpest, the flange distance is correct and calibration is complete
- If the closer-focused photo is the sharpest, focus is correct when the lens is farther and the flange distance is larger. Move the sensor away from the front of the camera by tightening / screwing the bolts clockwise to increase the flange distance.
- If the farther-focused photo is the sharpest, focus is correct when the lens is closer and the flange distance is smaller. Move the sensor toward the front of the camera by loosening / unscrewing the bolts counter-clockwise to decrease the flange distance.

Repeat the process of bracketing focus across the three distances and adjusting the flange distance until the correctly focused photo is the sharpest.
