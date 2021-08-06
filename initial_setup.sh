#!/bin/bash

# This script does below operations
# - Make Jetson to respond the power button immediately
# - Disable screen lock
# - Install AX200 driver
# - Enable spidev0.*, spidev1.*

# Exit on error
set -e

# Shutdown immediately when the power button was pressed
gsettings set org.gnome.settings-daemon.plugins.power button-power shutdown
sudo hostnamectl set-chassis vm

# Disable screen lock
gsettings set org.gnome.desktop.screensaver lock-enabled false

# Install Intel AX200 driver
sudo add-apt-repository -y ppa:canonical-hwe-team/backport-iwlwifi
sudo apt-get update -y
sudo apt-get install backport-iwlwifi-dkms

# Create service that set firmware fallback timeout value to 1s at every boot time
echo "[Unit]
Description=Set Firmware fallback timeout to 1

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo 1 > /sys/class/firmware/timeout'
TimeoutSec=0
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target" | sudo tee /etc/systemd/system/fallback-timeout.service
sudo systemctl enable fallback-timeout.service

# Rewrite extlinux.conf
# fbcon=map:0 => map:1, Default display output HDMI to DisplayPort
# add pci=noaer,        Suppress PCIe bus error caused by AX200
sudo sed -i 's/fbcon=map:0/fbcon=map:1 pci=noaer/g' /boot/extlinux/extlinux.conf

# Enable spidev0.*, spidev1.*
sudo /opt/nvidia/jetson-io/config-by-function.py -o dtb 1="spi1 spi2"
echo "spidev" | sudo tee -a /etc/modules

# Set permission to current user to manipulate GPIOs
sudo cp -f 99-gpio.rules /etc/udev/rules.d/
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $(whoami)

# Request reboot to user
echo "
Please reboot"
