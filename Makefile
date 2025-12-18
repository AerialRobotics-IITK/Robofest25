MAV_ID ?=1
DEVICE ?=/dev/ttyACM0
BAUD ?=115200
IMAGE ?=swarm
CMD ?=tmux
USER_UID ?=1000
USER_GID ?=1000

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

ardupilot:
	git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

arduimg: ardupilot
	cd ardupilot && PATH=$(PATH) PWD=$(PWD)/ardupilot podman build . -t ardupilot --build-arg USER_UID=$(USER_UID) --build-arg USER_GID=$(USER_GID)

ardu: arduimg
	podman run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest bash

arduland: arduimg
	podman run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot"\
  -v /run/user/$(USER_UID):/run/user/$(USER_UID):Z \
  -e WAYLAND_DISPLAY=wayland-0 \
  -e XDG_RUNTIME_DIR=/run/user/$(USER_UID) \
	--userns=keep-id ardupilot:latest bash

