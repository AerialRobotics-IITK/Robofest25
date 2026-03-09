MAV_ID ?=1
RUNTIME ?=podman
DEVICE ?=/dev/ttyACM0
BAUD ?=115200
FLAGS ?=
GUI_SCRIPT = gui_$(RUNTIME).sh
NVIDIA ?= 0

ifeq ($(NVIDIA),0)
NVIDIA_FLAGS :=
else ifeq ($(NVIDIA),1)
# Simple 'all' is the clearest and widely supported form
NVIDIA_FLAGS := --gpus all
else ifeq ($(NVIDIA),2)
# Select a specific device and capabilities. Adjust device index as needed.
NVIDIA_FLAGS := --gpus all,capabilities=compute,utility,graphics
else
$(error Invalid NVIDIA value '$(NVIDIA)'. Use 0, 1 or 2)
endif

FLAGS := $(FLAGS) $(NVIDIA_FLAGS)
WIFI_DEV ?=wlx3460f9ff4a4b
SSID ?=shadow
IMAGE ?=swarm
CONT_NAME ?=swarm_cont
REPLACE ?=0
CMD ?=tmux
USER_UID ?=1000
USER_GID ?=1000
FCU_PORT ?=14551
FCU_URL ?=udp://:$(FCU_PORT)@
NUM ?=3
SINGLE_CMD ?=sim_vehicle.py -v ArduCopter --out=udp:0.0.0.0:14550 --out=udp:0.0.0.0:14551 --console --map
SWARM_CMD ?=sim_vehicle.py -v Copter --out=udp:0.0.0.0:14550 --out=udp:0.0.0.0:14551 --out=udp:0.0.0.0:14552 --out=udp:0.0.0.0:14553 --console --count $(NUM) --auto-sysid --location CMAC --auto-offset-line 0,2 --mcast
SWARM_ARDU_GZ_CMD ?=sim_vehicle.py -v Copter -f gazebo-iris --out=udp:0.0.0.0:14550 \
										--out=udp:0.0.0.0:14551 \
										--out=udp:0.0.0.0:14552 \
										--out=udp:0.0.0.0:14553 \
										--out=udp:0.0.0.0:14554 \
										--out=udp:0.0.0.0:14555 \
										--console --count $(NUM) --auto-sysid --location CMAC --auto-offset-line 0,2 --mcast --model JSON
MAV_SWARM ?="ros2 launch swarm swarm_mavros.launch.py"

-include .env

pi-setup:
	sudo apt-get update && apt-get install -y podman neovim ripgrep fd-find chrony htop tmux git gh
	sudo systemctl enable --now chrony

image:
	$(RUNTIME) build -t $(IMAGE) .

usb:
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyACM0 RUNTIME=$(RUNTIME) REPLACE=$(REPLACE) CONT_NAME=$(CONT_NAME) \
			FCU_URL=serial:///dev/ttyACM0:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

gpio:
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyAMA0 RUNTIME=$(RUNTIME)  REPLACE=$(REPLACE) CONT_NAME=$(CONT_NAME) \
	FCU_URL=serial:///dev/ttyAMA0:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

gpio4:
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyS0 RUNTIME=$(RUNTIME)  REPLACE=$(REPLACE) CONT_NAME=$(CONT_NAME) \
	FCU_URL=serial:///dev/ttyS0:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

ama2:
	CMD=tmux IMAGE=swarm DEVICE=/dev/ttyS0 RUNTIME=$(RUNTIME)  REPLACE=$(REPLACE) CONT_NAME=$(CONT_NAME) \
	FCU_URL=serial:///dev/AMA2:$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh


custom:
	CMD=tmux IMAGE=swarm DEVICE=$(DEVICE) RUNTIME=$(RUNTIME)  REPLACE=$(REPLACE) CONT_NAME=$(CONT_NAME) \
	FCU_URL=serial://$(DEVICE):$(BAUD) MAV_ID=$(MAV_ID) ./pod.sh

local:
	$(RUNTIME) run -it --rm --net host \
  	--name $(CONT_NAME) \
		--privileged \
  	-v /run/udev:/run/udev \
		-e MAV_ID -e NUM=$(NUM) -e FCU_URL=$(FCU_URL) \
  	-v "$(PWD)/workspace/src:/workspace/src" \
		--group-add keep-groups $(IMAGE) $(CMD)

ardupilot:
	git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

arduimg: ardupilot
	cd ardupilot && PATH=$(PATH) PWD=$(PWD)/ardupilot $(RUNTIME) build . -t ardupilot --build-arg USER_UID=$(USER_UID) --build-arg USER_GID=$(USER_GID)

ardu: arduimg
	$(RUNTIME) run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest bash

uav1: arduimg
	$(RUNTIME) run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest $(SINGLE_CMD)

swarm: arduimg
	$(RUNTIME) run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest $(SWARM_CMD)

ardugzswarm: arduimg
	$(RUNTIME) run --rm --net host -it -v "$(PWD)/ardupilot:/ardupilot" --userns=keep-id ardupilot:latest $(SWARM_ARDU_GZ_CMD)

mavswarm:
	$(RUNTIME) run -it --rm --net host -e MAV_ID -e NUM=$(NUM) -e FCU_URL=$(FCU_URL) -v "$(PWD)/workspace/src:/workspace/src" --group-add keep-groups $(IMAGE) $(MAV_SWARM)

ardugzimg: arduimg
	$(RUNTIME) build -t ardu-gz --build-arg NUM=$(NUM) --build-arg USER_UID=$(USER_UID) --build-arg USER_GID=$(USER_GID) ./ardupilot_gazebo_swarm

ardugz: ardugzimg
	IMG=ardu-gz CMD=bash ./$(GUI_SCRIPT)


gziris: ardugzimg
	@echo $(FLAGS)
	IMG=ardu-gz CMD="./single_uav.sh" ./$(GUI_SCRIPT)

arduiris: arduimg
	$(RUNTIME) run --rm --net host --userns=keep-id -it -v "$(PWD)/ardupilot:/ardupilot" \
		ardupilot:latest sim_vehicle.py -v ArduCopter -f gazebo-iris \
		--out=udp:0.0.0.0:14550 --out=udp:0.0.0.0:14551 --model JSON --console

gzswarm: ardugzimg
	IMG=ardu-gz CMD="gz sim -v4 -r generated_swarm.sdf" ./$(GUI_SCRIPT)

hotspot:
	nmcli dev wifi hotspot ifname $(WIFI_DEV) ssid $(SSID) password $(WIFI_PASSWD)

gateway:
	WIFI_DEV=$(WIFI_DEV) ./Tools/internet_gateway.sh

