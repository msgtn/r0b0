#!/bin/sh
sudo cp /etc/dhcpcd.new /etc/dhcpcd.conf
sudo cp /etc/dnsmasq.new /etc/dnsmasq.conf
#enable hostapd
sudo systemctl enable hostapd