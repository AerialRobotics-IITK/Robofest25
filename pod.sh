#!/bin/bash
if [ "$REPLACE" -eq 0 ]; then
  echo "Attempting to start existing container $CONT_NAME..."
  $RUNTIME start "$CONT_NAME" tmux 2>/dev/null ||
    $RUNTIME run -it --name $CONT_NAME \
      --net host \
      --replace \
      -v "$(pwd)/workspace/src:/workspace/src" \
      -v /run/udev:/run/udev \
      --privileged \
      --device $DEVICE \
      -e MAV_ID -e FCU_URL \
      --group-add keep-groups $IMAGE $CMD
else
  echo "Replacing container $CONT_NAME..."
  $RUNTIME run -it --name $CONT_NAME \
    --net host \
    --replace \
    -v "$(pwd)/workspace/src:/workspace/src" \
    -v /run/udev:/run/udev \
    --privileged \
    --device $DEVICE \
    -e MAV_ID -e FCU_URL \
    --group-add keep-groups $IMAGE $CMD

fi
