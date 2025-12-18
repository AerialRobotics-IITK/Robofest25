MAV_ID ?=1
DEVICE ?=/dev/ttyACM0
BAUD ?=115200
IMAGE ?=swarm
CMD ?=tmux
USER_UID ?=1000
USER_GID ?=1000
FCU_URL ?=udp://:14550@

req:
	sudo apt-get update && sudo apt-get install -y podman tmux && touch req

image: req
	podman build -t $(IMAGE) .

usb: image
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyACM0 FCU_URL=serial:///dev/ttyACM0:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

gpio: image
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyAMA0 FCU_URL=serial:///dev/ttyAMA0:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

custom: image
	CMD=tmux IMAGE=swarm DEVICE=$(DEVICE) FCU_URL=serial://$(DEVICE):$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

local: image
	podman run -it --rm --net host -e MAV_ID -e FCU_URL=$(FCU_URL) --group-add keep-groups $(IMAGE) $(CMD)

ardupilot:
	git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

arduimg: ardupilot
	cd ardupilot && PATH=$(PATH) PWD=$(PWD)/ardupilot podman build . -t ardupilot --build-arg USER_UID=$(USER_UID) --build-arg USER_GID=$(USER_GID)

ardu: arduimg
	podman run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest bash
