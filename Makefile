MAV_ID ?=1
DEVICE ?=/dev/ttyACM0
BAUD ?=115200

req:
	sudo apt-get update && sudo apt-get install -y podman tmux && touch req

image: req
	podman build -t swarm .

usb: image
	DEVICE=/dev/ttyACM0 BAUD=$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

gpio: image
	DEVICE=/dev/ttyAMA0 BAUD=$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

custom: image
	DEVICE=$(DEVICE) MAV_ID=$(MAV_ID) ./pod.sh
