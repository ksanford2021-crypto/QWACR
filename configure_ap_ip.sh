#!/bin/bash
# Auto-configure wlan0 static IP for AP mode
# This script runs at boot to set the AP IP address

# Wait for wlan0 to be available
sleep 5

# Check if wlan0 exists and hostapd is enabled
if ip link show wlan0 &> /dev/null && systemctl is-enabled hostapd &> /dev/null; then
    # Set static IP for AP mode
    ip addr show wlan0 | grep "192.168.4.1" &> /dev/null
    if [ $? -ne 0 ]; then
        echo "Setting wlan0 IP to 192.168.4.1"
        ip addr add 192.168.4.1/24 dev wlan0
    fi
    
    # Restart dnsmasq to serve DHCP
    systemctl restart dnsmasq
    
    echo "AP mode configured: wlan0 at 192.168.4.1"
else
    echo "AP mode not enabled, skipping wlan0 configuration"
fi
