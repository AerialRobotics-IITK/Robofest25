#!/bin/bash

ETH=$(nmcli -t -f DEVICE,TYPE,STATE device status |
  awk -F: '$2=="ethernet" && $3=="connected" {print $1; exit}')

if [ -z "$ETH" ]; then
  echo "No active ethernet connection found"
  exit 1
fi

echo "Detected Ethernet Interface: $ETH"

sudo iptables -t nat -A POSTROUTING -o $ETH -j MASQUERADE
sudo iptables -A FORWARD -i $ETH -o $WIFI_DEV -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i $WIFI_DEV -o $ETH -j ACCEPT
