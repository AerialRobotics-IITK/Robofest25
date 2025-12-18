#!/bin/bash
podman run -it --rm --net host --device $DEVICE -e MAV_ID -e DEVICE=$DEVICE:$BAUD --group-add keep-groups swarm tmux
