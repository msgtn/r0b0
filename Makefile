PORT ?= 8080
DOCKER_FLAGS ?= -it

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
	sudo systemctl daemon-reload
	sudo systemctl kill blsm
	-docker ps | grep r0b0 | awk '{print $$NF}' | xargs -n1 docker rm -f
	sudo systemctl enable blsm
	sudo systemctl start blsm

docker-build:
	docker build -t r0b0:latest .

docker-run:
	UID=1000 docker compose -f ./docker-compose.yml run blsm

docker-build-run:
	make docker-build
	make docker-run
