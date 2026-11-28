# c-slam-bridge

Makes the rover a Wi-Fi sensor and actuator node for a SLAM host
([`host/ruby-slam`](../../host/ruby-slam/)):

- Joins Wi-Fi and listens on UDP port 5600.
- Once a host sends it a packet, streams to that host:
  - wheel odometry (pose and measured velocities) every 20 ms;
  - the YDLIDAR X2's bytes as they come from its UART (GPIO44 / D7), unparsed, at least every 20 ms.
    The host parses them, so the firmware does not depend on the LiDAR being there.
- Drives at the velocity `(v, w)` the host commands, within `MAX_SPEED` / `MAX_TURN_RATE` and
  the acceleration limits. With no command for 500 ms it stops: it never moves on its own.
- The BOOT button is an emergency stop; press it again to release.

The packets are in `main/protocol.h` (and `host/ruby-slam/lib/rover_slam/protocol.rb`).
The parameters are in `main/bridge_config.h`; the hardware, including the LiDAR pins, in
`../components/rover/include/rover_config.h`.

## Wi-Fi settings
```sh
cp sdkconfig.local.example sdkconfig.local   # ignored by git
# set CONFIG_ROVER_WIFI_SSID and CONFIG_ROVER_WIFI_PASSWORD in it
rm -f sdkconfig                              # so the next build picks the settings up
```

Then build as in [../README.md](../README.md) (`-w /firmware/c-slam-bridge`). The log prints the
address the rover got; give it to `bin/slam_bridge`.
