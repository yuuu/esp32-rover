# ruby-slam

A small 2D SLAM for the rover in plain Ruby (the standard library only), and a simulator to try
it without the hardware.

- `Session` decodes the rover's UDP packets ([protocol](../../firmware/c-slam-bridge/main/protocol.h)),
  keeps the odometry, and parses the YDLIDAR X2's byte stream into rotations.
- `Slam` matches each rotation against the map built so far (correlative scan matching on a
  likelihood field, then a hill climb), starting from where the odometry says the rover went,
  and grows a log-odds occupancy grid from the corrected pose. No loop closure yet.
- `SimRover` stands in for the rover and the X2: it drives in a `World` of wall segments,
  its odometry drifts (a scale error on each wheel and noise), and it sends exactly the packets
  and X2 bytes the real ones would. The same `Session` and `Slam` run on its output.

Requires Ruby 3.1 or later (developed on 4.0); `rake` and `minitest` for the tests.

## Usage
```sh
bin/simulate [--seconds 120]       # offline; writes out/map.png, prints odometry vs SLAM error
bin/fake_rover [--port 5600]       # a simulated rover on UDP, in real time
bin/slam_bridge ADDRESS [--auto]   # drive the rover (or the fake one) and map
rake test
```

`bin/slam_bridge` sends commands at 10 Hz (keys: w/s forward/back, a/d turn, space stop, q quit;
`--auto` wanders by itself), runs SLAM on every rotation and writes `out/map.png` every 5 s
(red: odometry, blue: SLAM). Against `bin/fake_rover` it is the full loop over UDP without the hardware:

```sh
bin/fake_rover &
bin/slam_bridge 127.0.0.1 --auto --seconds 60
```

In `bin/simulate`'s 2-minute run (seeds 1 and 2), odometry ends about 1 m and 40° off and SLAM
within about 1 cm and 0.2°, at about 50 ms of CPU per rotation (the X2 turns at about 7 Hz).
The room is ideal (straight walls, no clutter or glass), so expect worse on a real floor.

## Before the first real run
- `LIDAR_ANGLE_OFFSET_DEG` in `lib/rover_slam/config.rb`: which way the X2's 0° faces on the
  rover. Put something straight ahead and check where it lands in the map.
- The X2's angles are taken as clockwise seen from above, as espinel's `examples/lidar` reads them.
- Scans are not corrected for the motion during a rotation; keep turns slow while mapping.
