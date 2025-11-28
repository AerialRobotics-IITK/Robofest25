#!/bin/bash

# Batman-adv Mesh Network Setup Script

echo "Setting up Batman-adv mesh network..."

# Stop conflicting services
echo "Stopping conflicting services..."
sudo systemctl stop wpa_supplicant 2>/dev/null
sudo systemctl disable wpa_supplicant 2>/dev/null
sudo systemctl stop dhcpcd 2>/dev/null
sudo systemctl disable dhcpcd 2>/dev/null
sudo systemctl stop NetworkManager 2>/dev/null
sudo systemctl disable NetworkManager 2>/dev/null

# Unblock WiFi
echo "Unblocking WiFi..."
sudo rfkill unblock all
sleep 2

# Load batman-adv module
echo "Loading batman-adv module..."
sudo modprobe batman-adv

# Configure wireless interface
echo "Configuring wireless interface..."
sudo ip link set wlan0 down
sudo iwconfig wlan0 mode ad-hoc
sudo iwconfig wlan0 essid "meshnet"
sudo iwconfig wlan0 ap "02:12:34:56:78:9A"
sudo iwconfig wlan0 channel 1
sudo ip link set wlan0 up

# Add wireless interface to batman-adv
echo "Adding wlan0 to batman-adv..."
sudo batctl if add wlan0

# Bring up bat0 interface
echo "Bringing up bat0 interface..."
sudo ip link set up dev bat0

# Configure IP address
echo "Configuring IP address..."
sudo ip addr add 192.168.1.1/24 dev bat0

# Enable IP forwarding
echo "Enabling IP forwarding..."
echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward

# Configure NAT (optional)
echo "Configuring NAT..."
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i bat0 -o eth0 -j ACCEPT
sudo iptables -A FORWARD -i eth0 -o bat0 -m state --state RELATED,ESTABLISHED -j ACCEPT

echo "Mesh network setup complete!"
echo "Check status with: sudo batctl o"

