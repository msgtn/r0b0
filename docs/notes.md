
## Troubleshooting

### Motor settings
Setting motor info e.g. IDs needs torque to be disabled.
For example, to set the ID of motor 1 to 7 in using `r0b0.scripts.motor_calib.py`:
```
set_param('torque_enable',{1:False})
set_param('id',{1:7})
```

### 230712
- testing xl320
- ~~does not work with r0b0.scripts.motor_calib~~
- works under pypot, with params:
  - port: /dev/tty.usbmodem212401
  - baudrate: 1000000 (1e6)
- actually works, but dynamixel_python does not get the expected response, so pinging the motor fails.
- the issue is in the expected return packet
- Even using the U2D2 instead of the USB2AX (as old blsm did), still fails on expected

### 230713
- on raspberry pi, set up environment with `venv` instead of conda
- issues with rtmidi - look recent according to github issues
- running r0b0.scripts.motor_calib, needed to update the USB port
  - switched USB_PORT = '/dev/tty.usbserial-FT1SF1UM' -> '/dev/ttyUSB0'
  - Found portname by running `ls /dev/tty*` when U2D2 motor controller is both plugged and unplugged, and pinpointing what goes away

- in using free ngrok account
  - had to update both `host.py` and `controller.js` with the ngrok-generated url (.e.g https://16ac-32-221-140-83.ngrok-free.app)

### 231215
Set up `sphinx`.
Rough steps from the top-level directory:
```
pip3 install sphinx sphinx-rtd-theme
cd docs
sphinx-quickstart
```

We have to make several updates in `conf.py`.
First, update the theme by adding:
```
html_theme = 'sphinx_rtd_theme'
```
Next, add the root directory to the path:
```
import os, sys
sys.path.insert(0, os.path.abspath('..'))
```
Add extension support:
```
extensions = ['sphinx.ext.autodoc']
```

Create the source files and build the html:
```
sphinx-apidoc -f -o source ../src/r0b0/
make html
```

Serve the page at `localhost:8000`:
```
python3 -m http.server -d _build/html
```
References: [these](https://www.sphinx-doc.org/en/master/usage/quickstart.html) [guides](https://betterprogramming.pub/auto-documenting-a-python-project-using-sphinx-8878f9ddc6e9).

To *initialize* the docs at [`msgtn.github.io/r0b0`](msgtn.github.io/r0b0):
```
git subtree push --prefix docs/_build/html origin gh-pages
```
To *update* the docs with subsequent changes, call `make rebuild` from `docs/`, then, per [this guide](https://stephenlee.info/version%20control/2021/01/13/git-force-push-subtree.html):

## 240307
Problem - using the package as installed with `pip` renders the paths in `config.py` (e.g. `CONFIG_DIR`, `TAPES_DIR`) relative to the package's installation location in an environment.
It would be better to point these to a local user-defined location, relative to where the package is called rather than where it is installed.
What should this look like
```
config = {
  ...ke
  'tapes_dir': 'path/to/tapes/to/load'
  ...
}

```