# Examples

In this directory are example scripts for using `r0b0`.

## Create keys
Because the host creates a secure network with SSL, we must create keys.
From within this `examples/` directory:
```
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out csr.pem
```
Using the default answers (by pressing `Enter` repeatedly) is fine.
When you instantiate a `Rig` object, pass the location of these files as `certfile` and `keyfile`, respectively:
```
rig = Rig(
    ...
    certfile='path/to/crs.pem',
    keyfile='path/to/key.pem'
)
```

## `move_mouse_with_keys.py`
Example rig that maps keys to absolute mouse positions.
Use the keys [[Q,W,E],[A,S,D],[Z,X,C]] to move the mouse.
Note that the PyGame window that starts up *must* be focused for the program to register the key presses.
```
python3 move_mouse_with_keys.py
```

