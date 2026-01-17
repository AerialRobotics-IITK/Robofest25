#!/bin/bash
podman run --security-opt label=disable \
  --device=/dev/dri \
  -e XDG_RUNTIME_DIR=/tmp \
  -e DISPLAY \
  -e XAUTHORITY=$XAUTHORITY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
  --net host \
  --rm \
   $IMG $CMD 
