#! /bin/sh
curl localhost:4040/api/tunnels/blossomctrl/ | jq ".public_url" | tr -d '"' > public/ctrl_addr.txt
curl localhost:4040/api/tunnels/blossomapp/ | jq ".public_url" | tr -d '"' > public/app_addr.txt