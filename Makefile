MAV_ID ?=1
DEVICE ?=/dev/ttyACM0
BAUD ?=115200
IMAGE ?=swarm
CMD ?=tmux
USER_UID ?=1000
USER_GID ?=1000
FCU_URL ?=udp://:14551@
NUM ?=3
SINGLE_CMD ?=sim_vehicle.py -v ArduCopter --out=udp:0.0.0.0:14550 --out=udp:0.0.0.0:14551 --console --map
SWARM_CMD ?=sim_vehicle.py -v Copter --out=udp:0.0.0.0:14550 --out=udp:0.0.0.0:14551 --out=udp:0.0.0.0:14552 --out=udp:0.0.0.0:14553 --console --count $(NUM) --auto-sysid --location CMAC --auto-offset-line 90,10 --mcast
MAV_SWARM ?=bash -c "source install/setup.bash && ros2 launch swarm swarm_mavros.launch.py"

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
	podman run -it --rm --net host -e MAV_ID -e NUM=$(NUM) -e FCU_URL=$(FCU_URL) --group-add keep-groups $(IMAGE) $(CMD)

ardupilot:
	git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

arduimg: ardupilot req
	cd ardupilot && PATH=$(PATH) PWD=$(PWD)/ardupilot podman build . -t ardupilot --build-arg USER_UID=$(USER_UID) --build-arg USER_GID=$(USER_GID)

ardu: arduimg
	podman run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest bash

uav1: arduimg
	podman run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest $(SINGLE_CMD)

swarm: arduimg
	podman run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest $(SWARM_CMD)

mavswarm: image
	podman run -it --rm --net host -e MAV_ID -e NUM=$(NUM) -e FCU_URL=$(FCU_URL) --group-add keep-groups $(IMAGE) $(MAV_SWARM)

ardugzimg: req
	podman build -t ardu-gz --build-arg USER_UID=$(USER_UID) --build-arg USER_GID=$(USER_GID) ./ardupilot_gazebo_swarm

ardugz: ardugzimg
	IMG=ardu-gz CMD=bash ./gui.sh

gziris: ardugzimg
	IMG=ardu-gz CMD="gz sim -v4 -r iris_runway.sdf" ./gui.sh &
	podman run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
