#!/bin/bash
podman run --name swarm_cont -it --net host \
  -v "$(pwd)/workspace/src:/workspace/src" \
  -v /run/udev:/run/udev \
  --privileged \
  --device $DEVICE \
  -e MAV_ID -e FCU_URL \
  --group-add keep-groups $IMAGE $CMD
  # --device $DEVICE \
