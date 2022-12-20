## Blossom App - Web Browser Version

Minimal functionality version of the app that runs in a browser at `blossombot.com` (redirects to `https://blossomapp.ngrok.io`)

## Local Development

Requires Node.js and npm (tested using Node.js 8.x and npm 5.x).

Start `ngrok`, get the control address
```bash
$ ./ngrok.sh 
$ 
$ npm i
$ npm start
```


Start `ngrok`:
```bash
$ ./ngrok.sh
```

In a new tab, get the control address:
```bash
$ ./get_ctrl.sh
```

In the `blossom` directory (separate repo) sure the robot is running on `localhost` on port `4000`:
```bash
$ python3 start.py -n woody -b --host localhost --port 4000
```
If not connected to a robot (for testing purposes), replace `woody` with `test`.

Start the server:
```bash
$ npm start
```

By default, the application will serve on port `8000` and is thus accessible from `https://blossomapp.ngrok.io`. To start and configure the video stream, go to `https://localhost:8000/broadcast.html`.

Access `https://blossomapp.ngrok.io` from a mobile device, enter the password (probably `test`), and take `Control` of the robot.

## Troubleshooting

Running on `https` is important; `DeviceMotion` doesn't work without it, and permissions must be given if on iOS.