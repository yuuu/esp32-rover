#pragma once

// Join the access point in CONFIG_ROVER_WIFI_SSID and wait until DHCP gives an address.
// Reconnects by itself when the link drops.
void wifi_connect(void);
