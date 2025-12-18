#!/bin/bash
podman run -it --rm --net host --device $DEVICE -e MAV_ID -e FCU_URL --group-add keep-groups $IMAGE $CMD
