#!/bin/bash
podman run -it --rm --net host --device /dev/ttyACM0 --group-add keep-groups swarm tmux
