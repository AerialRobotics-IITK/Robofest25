#!/bin/bash
$RUNTIME run -it --rm --net host \
  -v "$(pwd)/workspace/src:/workspace/src" \
  -v /run/udev:/run/udev \
  --privileged \
  --device $DEVICE \
  -e MAV_ID -e FCU_URL \
  --group-add keep-groups $IMAGE $CMD
  # --device $DEVICE \
