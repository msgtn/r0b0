Blossom
=======

*2023-07-14 This page is a work-in-progress.* This page covers the build
and use of a Blossom robot, specifically: - Building the robot hardware
- Setting up the software - Controlling the robot using the mobile
interface

Build
-----

The total component cost for the base Dynamixel-powered configuration is
less than $200. The platform also supports a cheaper configuration using
micro servos and an Arduino for approximately $50.

Parts
~~~~~

The parts to print in ``*.stl`` format are `available
here <https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/>`__.
Each directory contains the whole runner
(e.g. `blsm_A <https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_A/blsm_A.stl>`__
) or individual parts
(e.g. `A1 <https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_A/A1.stl>`__).
I printed the parts on an entry-level `Creality Ender
3 <https://www.creality.com/products/ender-3-3d-printer>`__ and sliced
the files in Cura using `this
profile <https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm-020.curaprofile>`__.
The instructions are `available
here <https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm.pdf>`__.

+----------+------------+---------------------+---------------+-------+
| Part     | D          | Quantity            | Method        | A     |
|          | escription |                     |               | pprox |
|          |            |                     |               | imate |
|          |            |                     |               | total |
|          |            |                     |               | cost  |
+==========+============+=====================+===============+=======+
| blsm_A   | Part       | 1                   | `Print <htt   | NA    |
|          | runner     |                     | ps://github.c |       |
|          |            |                     | om/msgtn/r0b0 |       |
|          |            |                     | /blob/main/do |       |
|          |            |                     | cs/assets/bls |       |
|          |            |                     | m/blsm_A/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| blsm_B   | Part       | 2                   | `Print <htt   | NA    |
|          | runner     |                     | ps://github.c |       |
|          |            |                     | om/msgtn/r0b0 |       |
|          |            |                     | /blob/main/do |       |
|          |            |                     | cs/assets/bls |       |
|          |            |                     | m/blsm_B/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| blsm_C   | Part       | 2                   | `Print <htt   | NA    |
|          | runner     |                     | ps://github.c |       |
|          |            |                     | om/msgtn/r0b0 |       |
|          |            |                     | /blob/main/do |       |
|          |            |                     | cs/assets/bls |       |
|          |            |                     | m/blsm_C/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| blsm_D   | Part       | 1                   | `Print <htt   | NA    |
|          | runner     |                     | ps://github.c |       |
|          |            |                     | om/msgtn/r0b0 |       |
|          |            |                     | /blob/main/do |       |
|          |            |                     | cs/assets/bls |       |
|          |            |                     | m/blsm_D/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| blsm_E   | Part       | 3                   | `Print <htt   | NA    |
|          | runner     |                     | ps://github.c |       |
|          |            |                     | om/msgtn/r0b0 |       |
|          |            |                     | /blob/main/do |       |
|          |            |                     | cs/assets/bls |       |
|          |            |                     | m/blsm_E/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| blsm_F   | Part       | 3                   | `Print <htt   | NA    |
|          | runner     |                     | ps://github.c |       |
|          |            |                     | om/msgtn/r0b0 |       |
|          |            |                     | /blob/main/do |       |
|          |            |                     | cs/assets/bls |       |
|          |            |                     | m/blsm_F/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| blsm_M   | Part       | 4                   | `Print <htt   | NA    |
|          | runner     |                     | ps://github.c |       |
|          |            |                     | om/msgtn/r0b0 |       |
|          |            |                     | /blob/main/do |       |
|          |            |                     | cs/assets/bls |       |
|          |            |                     | m/blsm_M/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| blsm_SR  | *S*\ lip   | 1                   | `Print the    | NA or |
|          | *R*\ ing   |                     | static        | $     |
|          | for        |                     | non-rotating  | 22.00 |
|          | rotating   |                     | ‘dummy’       |       |
|          | the upper  |                     | mo            |       |
|          | body       |                     | del <https:// |       |
|          |            |                     | github.com/ms |       |
|          |            |                     | gtn/r0b0/blob |       |
|          |            |                     | /main/docs/as |       |
|          |            |                     | sets/blsm/bls |       |
|          |            |                     | m_SR.stl>`__, |       |
|          |            |                     | or use `the   |       |
|          |            |                     | actual        |       |
|          |            |                     | 12-wire slip  |       |
|          |            |                     | ring to       |       |
|          |            |                     | enable        |       |
|          |            |                     | continuous    |       |
|          |            |                     | rotation      |       |
|          |            |                     | beyond 360    |       |
|          |            |                     | degrees <http |       |
|          |            |                     | s://www.spark |       |
|          |            |                     | fun.com/produ |       |
|          |            |                     | cts/13065>`__ |       |
+----------+------------+---------------------+---------------+-------+
| String   | String for | 1                   | Purchase      | $     |
|          | actuating  |                     | (             | 5-$20 |
|          | the head   |                     | e.g. `fishing |       |
|          |            |                     | line <htt     |       |
|          |            |                     | ps://www.powe |       |
|          |            |                     | rpro.com/cont |       |
|          |            |                     | ent/powerpro/ |       |
|          |            |                     | northamerica/ |       |
|          |            |                     | us/en/homepag |       |
|          |            |                     | e/PDP.P-POWER |       |
|          |            |                     | PRO.html>`__, |       |
|          |            |                     | `twine        |       |
|          |            |                     | <https://www. |       |
|          |            |                     | amazon.com/Wh |       |
|          |            |                     | ite-Cotton-Bu |       |
|          |            |                     | tchers-Twine- |       |
|          |            |                     | String/dp/B09 |       |
|          |            |                     | TQXBFYD/>`__) |       |
+----------+------------+---------------------+---------------+-------+
| Rubber   | Rubber     | 6                   | Purchase      | $8    |
| bands,   | bands for  |                     | e.g. `black   |       |
| 4mm      | hanging    |                     | rubber        |       |
| diameter | the head   |                     | bands         |       |
|          | platform   |                     | <https://www. |       |
|          |            |                     | amazon.com/Ru |       |
|          |            |                     | bber-200pcs-E |       |
|          |            |                     | lastic-Sturdy |       |
|          |            |                     | -School/dp/B0 |       |
|          |            |                     | 924HDQXQ/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| TOTAL    |            |                     |               | $1    |
|          |            |                     |               | 3-$50 |
+----------+------------+---------------------+---------------+-------+

Motors and electronics
~~~~~~~~~~~~~~~~~~~~~~

The standard full-featured configuration uses Dynamixel ‘smart’ servos.
Dynamixels have a lot of nice features built in, such as different
operating modes (position- or velocity-control modes), velocity and
acceleration profiles, and PID tweaking.

+----------+------------+---------------------+---------------+-------+
| Part     | D          | Quantity            | Method        | A     |
|          | escription |                     |               | pprox |
|          |            |                     |               | imate |
|          |            |                     |               | total |
|          |            |                     |               | cost  |
+==========+============+=====================+===============+=======+
| D        | Motor      | 4                   | `Purchas      | $100  |
| ynamixel |            |                     | e <https://ww | ($2   |
| XL       |            |                     | w.robotis.us/ | 5/ea) |
| 330-M288 |            |                     | dynamixel-xl3 |       |
|          |            |                     | 30-m288-t>`__ |       |
+----------+------------+---------------------+---------------+-------+
| D        | Cables     | 6                   | `Purchase <ht | NA    |
| ynamixel | (included  |                     | tps://www.rob | (inc  |
| X3P      | with XL330 |                     | otis.us/robot | luded |
| Cable    | motors)    |                     | -cable-x3p-18 | with  |
|          |            |                     | 0mm-10pcs>`__ | XL330 |
|          |            |                     |               | m     |
|          |            |                     |               | otors |
|          |            |                     |               | —     |
|          |            |                     |               | only  |
|          |            |                     |               | buy   |
|          |            |                     |               | if    |
|          |            |                     |               | need  |
|          |            |                     |               | sp    |
|          |            |                     |               | ares) |
+----------+------------+---------------------+---------------+-------+
| D        | Motor      | 1                   | `Pur          | $     |
| ynamixel | controller |                     | chase <https: | 32.00 |
| U2D2     |            |                     | //www.robotis |       |
|          |            |                     | .us/u2d2/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| TOTAL    |            |                     |               | $132  |
+----------+------------+---------------------+---------------+-------+

The Arduino-powered configuration is much cheaper (the total cost
equivalent to just one Dynamixel) and more hackable, but the movement is
noisier and less smooth. The left-right yaw rotation is also limited to
±90°. This version is more readily hackable with whatever can interface
with an Arduino.

+----------+------------+---------------------+---------------+-------+
| Part     | D          | Quantity            | Method        | A     |
|          | escription |                     |               | pprox |
|          |            |                     |               | imate |
|          |            |                     |               | total |
|          |            |                     |               | cost  |
+==========+============+=====================+===============+=======+
| Arduino  | Micro      | 1                   | `Purc         | $15   |
| (or      | controller |                     | hase <https:/ |       |
| clone)   | and cable  |                     | /www.amazon.c |       |
|          |            |                     | om/ELEGOO-Boa |       |
|          |            |                     | rd-ATmega328P |       |
|          |            |                     | -ATMEGA16U2-C |       |
|          |            |                     | ompliant/dp/B |       |
|          |            |                     | 01EWOE0UU>`__ |       |
+----------+------------+---------------------+---------------+-------+
| Micro    | Small      | 4                   | `Pur          | $10   |
| servo    | servos     |                     | chase <https: |       |
|          | with basic |                     | //www.amazon. |       |
|          | position   |                     | com/Dorhea-Ar |       |
|          | control    |                     | duino-Helicop |       |
|          |            |                     | ter-Airplane- |       |
|          |            |                     | Walking/dp/B0 |       |
|          |            |                     | 7Q6JGWNV/>`__ |       |
+----------+------------+---------------------+---------------+-------+
| TOTAL    |            |                     |               | $25   |
+----------+------------+---------------------+---------------+-------+

Wiring
~~~~~~

For the Dynamixel-powered configuration, `refer to the wiring
documentation to set up the U2D2 controller with an external power
supply <https://github.com/msgtn/r0b0/blob/main/docs/wiring.md>`__. For
the Arduino-powered design, refer to `this Fritzing
diagram <https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_ard.png>`__.
Either configuration will require USB breakouts for power and some
cables.

+----+---------------+----------+-------+-----------------------------+
| Pa | Description   | Quantity | M     | Approximate total cost      |
| rt |               |          | ethod |                             |
+====+===============+==========+=======+=============================+
| U  | Breaks out    | 1        | `Purc | $6                          |
| SB | power         |          | hase  |                             |
| br | connections   |          | <http |                             |
| ea | to ease       |          | s://w |                             |
| ko | supplying     |          | ww.am |                             |
| ut | power         |          | azon. |                             |
|    |               |          | com/T |                             |
|    |               |          | reedi |                             |
|    |               |          | x-Typ |                             |
|    |               |          | e-C-B |                             |
|    |               |          | reako |                             |
|    |               |          | ut-Co |                             |
|    |               |          | nnect |                             |
|    |               |          | or-Co |                             |
|    |               |          | nvert |                             |
|    |               |          | er/dp |                             |
|    |               |          | /B096 |                             |
|    |               |          | M2HQL |                             |
|    |               |          | K>`__ |                             |
+----+---------------+----------+-------+-----------------------------+
| M  | Connects      | Several  | `Pu   | $7                          |
| al | components    |          | rchas |                             |
| e– |               |          | e <am |                             |
| Fe |               |          | azon. |                             |
| ma |               |          | com/E |                             |
| le |               |          | legoo |                             |
| a  |               |          | -EL-C |                             |
| nd |               |          | P-004 |                             |
| M  |               |          | -Mult |                             |
| al |               |          | icolo |                             |
| e– |               |          | red-B |                             |
| Ma |               |          | readb |                             |
| le |               |          | oard- |                             |
| w  |               |          | ardui |                             |
| ir |               |          | no/dp |                             |
| es |               |          | /B01E |                             |
|    |               |          | V70C7 |                             |
|    |               |          | 8>`__ |                             |
|    |               |          | or    |                             |
|    |               |          | use   |                             |
|    |               |          | spare |                             |
|    |               |          | c     |                             |
|    |               |          | ables |                             |
|    |               |          | and   |                             |
|    |               |          | b     |                             |
|    |               |          | readb |                             |
|    |               |          | oards |                             |
+----+---------------+----------+-------+-----------------------------+
| T  |               |          |       | $13                         |
| OT |               |          |       |                             |
| AL |               |          |       |                             |
+----+---------------+----------+-------+-----------------------------+

Hardware
~~~~~~~~

The minimal M2 hardware required is all available in `this
set <https://www.amazon.com/gp/product/B082XR52P1/>`__ for $10.

======= ======== ===================================
Part    Quantity Notes
======= ======== ===================================
M2x8mm  14       
M2x10mm 12       
M2 nuts 12       Only necessary if using SG90 servos
======= ======== ===================================

Tools and miscellaneous
~~~~~~~~~~~~~~~~~~~~~~~

+----------+-----------------------------+---------------------+-------+
| Part     | Description                 | Quantity            | Notes |
+==========+=============================+=====================+=======+
| Wire     | Cutting parts from the      | 1                   |       |
| cutters  | runners, cutting wires      |                     |       |
+----------+-----------------------------+---------------------+-------+
| El       | Connecting motors           | A couple meters     |       |
| ectrical |                             | worth               |       |
| wire     |                             |                     |       |
+----------+-----------------------------+---------------------+-------+
| USB wall | Powering the motors         | 1                   | Id    |
| adapter  |                             |                     | eally |
|          |                             |                     | 15W   |
|          |                             |                     | (5    |
|          |                             |                     | V/3A) |
|          |                             |                     | or    |
|          |                             |                     | gr    |
|          |                             |                     | eater |
|          |                             |                     | to    |
|          |                             |                     | s     |
|          |                             |                     | upply |
|          |                             |                     | suffi |
|          |                             |                     | cient |
|          |                             |                     | power |
|          |                             |                     | to    |
|          |                             |                     | the   |
|          |                             |                     | m     |
|          |                             |                     | otors |
+----------+-----------------------------+---------------------+-------+

.. _wiring-1:

Wiring
~~~~~~

Follow the `wiring
instructions <https://github.com/msgtn/r0b0/blob/main/docs/wiring.md>`__.

Software
--------

Environment setup
~~~~~~~~~~~~~~~~~

Set up `conda <https://conda.io>`__, then set up a conda environment and
install some other dependencies with ``pip`` (because of issues with
`mouse <https://github.com/boppreh/mouse/issues/75>`__). Docker maybe
coming soon (maybe).

::

   conda env create -f env.yaml
   conda activate r0b0
   pip3 install -r req.txt 

To enable ``https`` for the control page, generate some keys with
``openssl``. Since this is self-signing(*?*), you can safely hit ‘Enter’
to accept the defaults for all fields.

::

   openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout r0b0/key.pem -out r0b0/csr.pem

ngrok setup
~~~~~~~~~~~

*This has only been tested on iOS.* Phone-based motion control is
enabled through ``ngrok``. ``ngrok`` opens a tunnel to a local port
(e.g. ``localhost:8080``) through a URL. Tunneling enables sending data
transmission even from non-local networks — this enables telepresence by
sending phone orientation data and WebRTC handshaking through the
tunnel. If you’re not interested in motion control, you can skip this
section.

Sign up for `ngrok <https://ngrok.com>`__. Continue on with the guides
until you can run ``ngrok`` as a terminal command - this will probably
require some ``sudo apt``\ ing (Linux) or ``brew``\ ing (macOS) and some
``authtoken``\ ing. Start a tunnel to ``https://{hostname}:{port}`` that
the ``blsm`` rig is running on, e.g. with the defaults of
``hostname=localhost`` and ``port=8080``:

::

   ngrok http https://localhost:8080

The terminal will show you the forwarding URL, e.g.:

::

   ...
   Forwarding http://someRandomLettersAndNumbers.ngrok.app -> https://localhost:8080
   Forwarding https://someRandomLettersAndNumbers.ngrok.app -> https://localhost:8080
   ...

*This next part is a kludge.* We need to update this address in three
files: ``r0b0/rigs/static/controller.js``,
``r0b0/rigs/static/player.js``, and ``r0b0/rigs/host.py``. This address
is stored as ``socketAddr`` and ``SOCKET_ADDR`` towards the top of each
file — modify these to
``https://someRandomLettersAndNumbers.ngrok.app``: In ``controller.js``
and ``player.js``:

::

   const socketAddr = "https://someRandomLettersAndNumbers.ngrok.app"

In ``host.py``:

::

   SOCKET_ADDR = "https://someRandomLettersAndNumbers.ngrok.app"

Note that ``ngrok`` must be running in a separate terminal — start it,
then open another terminal to continue the instructions.

If you have a paid ``ngrok`` subscription, you can add a ``--subdomain``
argument to the tunnel command to maintain a consistent forwarding URL.
For example, to set the forwarding URL to
``https://mysubdomain.ngrok.io``:

::

   ngrok http https://localhost:8080 --subdomain=mysubdomain

Motor calibration (Dynamixel models only)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Next, we will calibrate the motors. This is only necessary for Dynamixel
motors First, we need to figure out the USB port that the motor
controller (e.g. U2D2, USB2AX) is connected to. Run ``ls /dev/tty*``
twice, once with the motor controller connected and again with it
disconnected, and take note of the port that disappeared,
e.g. ``/dev/tty.usbserial-FT1SF1UM``. Open
``r0b0/scripts/motor_calib.py`` and modify the parameters (motor model,
USB port, baud rate) towards the top for your robot’s configuration
(XL330 for the new version of the robot, XL320 for the old version):

::

   # an example for XL330 motors
   MOTOR_MODEL,USB_PORT,BAUD_RATE = 'xl330-m288','/dev/tty.usbserial-FT1SF1UM',57600
   # an example for XL320 motors
   MOTOR_MODEL,USB_PORT,BAUD_RATE = 'xl320','/dev/tty.usbmodem212401',1e6

With one motor connected at a time, run this calibration script:

::

   python3 -m r0b0.scripts.motor_calib

This will scan for connected motors, and should find the connected
motor, usually with ID 1 if it has not yet been set. The script will
pause at ``(Pdb)`` — this means that the script has started successfully
and is now in a debugging loop. To set the ID, for example from 1 to 2:

::

   m1 = dxl_mgr.dxl_dict['1']
   m1.set_torque_enable(False)
   m1.set_id(2)
   m2 = dxl_mgr.dxl_dict['2']
   m2.set_torque_enable(True)

To test if the ID was changed successfully, we can toggle the LED.

::

   m2.set_led(True)
   m2.set_led(False)

To set the motor to the default position:

::

   # for XL330
   m2.set_goal_position(1000) # for the towers:1000; for the base: 2000
   # for XL320
   m2.set_goal_position(700) # for the towers:700 ; for the base: 500

To stop the script, type ``Ctrl+D``. Repeat this for motor IDs 3 and 4.

Starting the ``blsm`` rig
-------------------------

Dynamixel
~~~~~~~~~

Start the ``blsm`` rig configuration, which contains the ``blsm_dxl``
robot as a ``DynamixelRobot`` and the ``bslm_phone`` browser-based
interface as a ``Page``. The rig uses the ``motion2motor`` cable to
translate ``device_motion`` events from the page (when accessed from a
mobile browser) into ``position`` events for the motor.

In ``/config/gadgets/blsm_dxl.yaml``
(`here <https://github.com/msgtn/r0b0/blob/main/config/gadgets/blsm_ard.yaml>`__),
modify ``usb_port`` with the port we found during the motor calibration
step:

::

   type: DynamixelRobot
   usb_port: /dev/tty.usbserial-FT1SF1UM   # modify this

In a separate terminal window from the ``ngrok`` tunnel script,

::

   python3 start.py --config blsm

Arduino
~~~~~~~

We must first flash the Arduino with the pyFirmata firmware, which
enables the Arduino to be controlled from Python through the `Arduino
gadget class <../r0b0/gadgets/arduino.py>`__. Connect the Arduino to the
computer. Open
`r0b0/gadgets/Standardfirmata.ino <../r0b0/gadgets/StandardFirmata/StandardFirmata.ino>`__
in the `Arduino IDE <https://www.arduino.cc/en/software>`__. To find the
port that the Arduino is connected to, use the Arduino IDE (``Tools`` >
``Port``). Upload the firmware to the board (``Sketch`` > ``Upload``).

Next, we need to modify the configuration at
``/config/gadgets/blsm_ard.yaml``
(`here <https://github.com/msgtn/r0b0/blob/main/config/gadgets/blsm_ard.yaml>`__)
with the ``usb_port`` and motor ``id``\ s. For the motor IDs, refer to
the `Fritzing
diagram <https://github.com/msgtn/r0b0/blob/main/docs/assets/blsm/blsm_ard.png>`__
and modify according to your specific build:

::

   type: ArduinoRobot
   usb_port: /dev/cu.usbserial-ADAQDbKpQ # modify this to the port that the Arduino is connected to
   baud_rate: 57600
   timeout: 2
   motors:
   - name: base
     id: 9         # modify this to the pin that the BASE motor is connected to
   - name: tower_1
     id: 10        # modify this to the pin that the FRONT head motor is connected to
   - name: tower_2
     id: 6         # modify this to the pin that the LEFT head motor is connected to
   - name: tower_3
     id: 5         # modify this to the pin that the RIGHT head motor is connected to

Telepresence
------------

Video (optional)
~~~~~~~~~~~~~~~~

Connect a USB webcam to your computer. With the prior scripts running
(``start.py`` and the ``ngrok`` tunnel), on the desktop/laptop computer
controlling the robot, navigate to
``https://localhost:8080/broadcaster``. This page contains the controls
for WebRTC media sources. Select the connected webcam in the dropdown,
which should begin a video feed on the page.

Control
~~~~~~~

In a mobile browser (e.g. Safari), navigate to the forwarding URL
(``https://someRandomLettersAndNumbers.ngrok.app`` in the above
example). *Note: since the ssl certificates were self signed, you will
probably run into a privacy warning on your browser.* `Here’s a guide on
how to bypass this, which should be safe since this is being developed
locally
anyways. <https://www.vultr.com/docs/how-to-bypass-the-https-warning-for-self-signed-ssl-tls-certificates/>`__

You should see video feed from the webcam selected in
``https://localhost:8080/broadcaster``. Hold the phone straight out, as
if you were taking a picture of something directly in front of you.
Toggle the ‘head’ switch to turn on control and begin transmitting the
phone orientation to the robot. The motor controller should start
blinking blue to indicate that it is sending motor commands. The robot’s
head should be moving in response to the phone motion.

Recording movements
~~~~~~~~~~~~~~~~~~~

To begin recording a movement, ensure that the control switch is on and
click the large red recording button in the center. Move the phone to
control the robot, then click the recording button again to stop. This
will save the motion as a ``Tape`` in the ``/tapes`` directory (more
documentation
`here <https://github.com/msgtn/r0b0/blob/main/r0b0/gadgets/README.md>`__).

Player
~~~~~~

In either the desktop or mobile browser, navigate to the Player page at
``https://someRandomLettersAndNumbers.ngrok.app/player``. Click ‘Update’
to populate the dropdown with the tape files in ``tapes``. Select a tape
and click ‘Play’ to begin playback. If you create new movement
recordings using the controller interface, you can repopulate the
dropdown by clicking ‘Update’ without having to refresh the page. Note
that tapes are only loaded once in the backend, so if you manually
rename files, you must restart the whole ``start.py`` script to override
the cached tape.

You can also call this function from the command line. For example, to
play ``tapes/demo_tape.json``:

::

   rig.play('demo_tape')

Troubleshooting
---------------

Motor settings
~~~~~~~~~~~~~~

Setting motor info e.g. IDs needs torque to be disabled. For example, to
set the ID of motor 1 to 7 in using ``r0b0.scripts.motor_calib.py``:

::

   set_param('torque_enable',{1:False})
   set_param('id',{1:7})

Interface issues
~~~~~~~~~~~~~~~~

On the mobile interface, turning on the control switch should first
prompt a request for access to the device orientation. If this is not
popping up, ensure that ``socketAddr``/``SOCKET_ADDR`` are defined
appropriately in ``r0b0/rigs/static/controller.js``,
``r0b0/rigs/static/player.js``, and ``r0b0/rigs/host.py``. They should
be set to the ``ngrok`` address tunnelling to
``https://localhost:8080``,
e.g. ``https://104e-32-221-140-83.ngrok-free.app``.

Slow control
~~~~~~~~~~~~

There is a bit of lag between the phone control and the robot control,
which is to be expected considering the data passing through the
network. Try the following if the lag is too large for your application.

Networking
^^^^^^^^^^

Ensure that the phone controller is connected to the same network as the
robot’s computer.

Motor parameters
^^^^^^^^^^^^^^^^

The robot configuration at ``/config/gadgets/blsm_dxl.yaml``
(`here <https://github.com/msgtn/r0b0/blob/main/config/gadgets/blsm_dxl.yaml>`__)
contains parameters for the motor movement, such as the goal/profile
velocity/acceleration. On startup, ``/r0b0/gadgets/dxl_robot.py``
(`here <https://github.com/msgtn/r0b0/blob/main/src/r0b0/gadgets/dxl_robot.py>`__)
configures these parameters during startup. You can tune these values,
and `refer to the motor
documentation <https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/>`__
for available parameters.

To set motor parameters, add values as entries in the configuration
file. Any writable parameter can be set in the configuration file — just
add the entry as lower cased and underscored (e.g. ‘Profile Velocity’ ->
``profile_velocity``) For example, in ``/config/gadgets/blsm_dxl.yaml``
(`here <https://github.com/msgtn/r0b0/blob/main/config/gadgets/blsm_dxl.yaml>`__),
to set ``tower_1``\ ’s `Profile
Velocity <https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#profile-velocity>`__
and `Profile
Acceleration <https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#profile-acceleration>`__
to 300 and 100, respectively:

::

   - name: tower_1
     model: xl330-m288
     id: 1
     operating_mode: 3
     profile_velocity: 300       
     profile_acceleration: 100

Setting ``operating_mode: 3`` sets the motors to position control mode,
per the
`documentation <https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#operating-mode>`__.
Faster velocity and acceleration will yield snappier movements at the
risk of jerkiness.
