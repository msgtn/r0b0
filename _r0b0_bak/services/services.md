#rpi Setting up a system service to run on boot
- Edit `/lib/systemd/system/hello.service`
```
sudo chmod 644 /lib/systemd/system/hello.service
chmod +x /home/pi/hello_world.py
sudo systemctl daemon-reload
sudo systemctl enable hello.service
sudo systemctl start hello.service
```
