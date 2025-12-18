MAV_ID ?=1
DEVICE ?=/dev/ttyACM0
BAUD ?=115200
IMAGE ?=swarm
CMD ?=tmux

req:
	sudo apt-get update && sudo apt-get install -y podman tmux && touch req

image: req
	podman build -t $(IMAGE) .

usb: image
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyACM0 BAUD=$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

gpio: image
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyAMA0 BAUD=$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

custom: image
	CMD=tmux IMAGE=swarm DEVICE=$(DEVICE) MAV_ID=$(MAV_ID) ./pod.sh
