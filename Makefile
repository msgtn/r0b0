# Environment variables:
#   DEVICE        - Serial port (default: /dev/ttyACM0)
#   MOTOR_TYPE    - Motor type: "dxl" (default) or "servo"
#   DISABLE_VIDEO - Set to 1 to disable video feed (default: 0)
# Example: DEVICE=/dev/ttyUSB0 MOTOR_TYPE=servo make docker-run
# Example: DISABLE_VIDEO=1 make docker-run
DOCKER_FLAGS ?= -it
DEVICE ?= /dev/ttyACM0

blsm:
	-. scripts/ngrok.sh &
	uv run python3 src/r0b0/ros2/blsm.py

blsm-stop:
	-ps aux | grep blsm | grep -v grep | awk '{print $$2}' | xargs -n1 sudo kill  -9
	-ps aux | grep ngrok.sh  |  grep -v grep | awk '{print $$2}' | xargs -n1 sudo kill -9

keys:
	mkdir -p .keys
	openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout .keys/key.pem -out .keys/csr.pem -subj "/C=US/O=r0b0/CN=localhost"

.PHONY: service
service:
	sudo cp ./service/blsm.service /etc/systemd/system/blsm.service
	sudo cp ./service/pico-reset.service /etc/systemd/system/pico-reset.service
	sudo systemctl daemon-reload
	sudo systemctl kill blsm
	-docker ps | grep r0b0 | awk '{print $$NF}' | xargs -n1 docker rm -f
	sudo systemctl enable blsm
	sudo systemctl enable pico-reset
	sudo systemctl start blsm
	sudo systemctl start pico-reset

docker-build:
	docker build -t r0b0:latest .

docker-run:
	-docker stop r0b0 2>/dev/null
	-docker rm r0b0 2>/dev/null
	UID=1000 DEVICE=${DEVICE} docker compose up

docker-stop:
	-docker compose down
	-docker stop r0b0 2>/dev/null
	-docker rm r0b0 2>/dev/null

docker-build-run:
	make docker-build
	make docker-run

pico-reset:
	python3 scripts/pico_reset_cli.py
