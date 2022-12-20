#!/bin/sh
sudo cp /etc/dhcpcd.bak /etc/dhcpcd.conf
sudo cp /etc/dnsmasq.bak /etc/dnsmasq.conf
#enable hostapd
sudo systemctl disable hostapd