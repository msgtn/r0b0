blsm:
	-. scripts/ngrok.sh &
	uv run python3 src/r0b0/ros2/blsm.py

blsm-stop:
	-ps aux | grep blsm | grep -v grep | awk '{print $$2}' | xargs -n1 sudo kill  -9
	-ps aux | grep ngrok.sh  |  grep -v grep | awk '{print $$2}' | xargs -n1 sudo kill -9

keys:
	mkdir -p .keys
	openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout .keys/key.pem -out .keys/csr.pem

docker-build:
	docker build -t r0b0:latest .

docker-run-pi:
	docker run --name r0b0-blsm --privileged -v /dev/serial0:/dev/serial0 -p 8080:8080 -it r0b0:latest

docker-run-desktop:
	docker run --privileged -v /dev/ttyACM0:/dev/ttyACM0 -p 8080:8080 -it r0b0:latest
