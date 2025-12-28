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
SWARM_CMD ?=sim_vehicle.py -v Copter --out=udp:0.0.0.0:14550 --out=udp:0.0.0.0:14551 --out=udp:0.0.0.0:14552 --out=udp:0.0.0.0:14553 --console --count $(NUM) --auto-sysid --location CMAC --auto-offset-line 0,2 --mcast
SWARM_ARDU_GZ_CMD ?=sim_vehicle.py -v Copter -f gazebo-iris --out=udp:0.0.0.0:14550 \
										--out=udp:0.0.0.0:14551 \
										--out=udp:0.0.0.0:14552 \
										--out=udp:0.0.0.0:14553 \
										--console --count $(NUM) --auto-sysid --location CMAC --auto-offset-line 0,2 --mcast --model JSON
MAV_SWARM ?="ros2 launch swarm swarm_mavros.launch.py"

req:
	sudo apt-get update && sudo apt-get install -y podman tmux && touch req

image: req
	podman build -t $(IMAGE) .

usb: image
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyACM0 FCU_URL=serial:///dev/ttyACM0:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

gpio: image
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyAMA0 FCU_URL=serial:///dev/ttyAMA0:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

gpio4: image
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyS0 FCU_URL=serial:///dev/ttyS0:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

ama2: image
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyS0 FCU_URL=serial:///dev/AMA2:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

deploy: image
	touch deploy && CMD=tmux IMAGE=swarm DEVICE=$(DEVICE) FCU_URL=serial://$(DEVICE):$(BAUD) MAV_ID=$(MAV_ID) ./deploy.sh

run:
	podman start -ai swarm_cont

custom: image
	CMD=tmux IMAGE=swarm DEVICE=$(DEVICE) FCU_URL=serial://$(DEVICE):$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

local: image
	podman run -it --rm --net host \
		--privileged \
  	-v /run/udev:/run/udev \
		-e MAV_ID -e NUM=$(NUM) -e FCU_URL=$(FCU_URL) \
  	-v "$(PWD)/workspace/src:/workspace/src" \
		--group-add keep-groups $(IMAGE) $(CMD)

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

ardugzswarm: arduimg
	podman run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest $(SWARM_ARDU_GZ_CMD)

mavswarm: image
	podman run -it --rm --net host -e MAV_ID -e NUM=$(NUM) -e FCU_URL=$(FCU_URL) -v "$(PWD)/workspace/src:/workspace/src" --group-add keep-groups $(IMAGE) $(MAV_SWARM)

ardugzimg: req arduimg
	podman build -t ardu-gz --build-arg NUM=$(NUM) --build-arg USER_UID=$(USER_UID) --build-arg USER_GID=$(USER_GID) ./ardupilot_gazebo_swarm

ardugz: ardugzimg
	IMG=ardu-gz CMD=bash ./gui.sh

gziris: ardugzimg
	IMG=ardu-gz CMD="gz sim -v4 -r iris_runway.sdf" ./gui.sh &
	podman run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest sim_vehicle.py -v ArduCopter -f gazebo-iris --out=udp:0.0.0.0:14550 --out=udp:0.0.0.0:14551 --model JSON --console

gzswarm: ardugzimg
	IMG=ardu-gz CMD="gz sim -v4 -r generated_swarm.sdf" ./gui.sh
