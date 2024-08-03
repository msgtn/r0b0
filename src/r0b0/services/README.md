# Purpose
These `systemd` services start scripts on boot.

# Usage
```
# Copy services to the appropriate folder
sudo cp ./*.service /lib/systemd/system

# Enable the services to start on boot, after network initialization
sudo systemctl daemon-reload
sudo systemctl enable mpi.service
sudo systemctl restart mpi.service
```


