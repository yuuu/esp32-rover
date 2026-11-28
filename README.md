# esp32-rover

A small, low-cost differential-drive rover for experimenting with SLAM. A Seeed XIAO ESP32-S3 drives two geared DC motors with encoders, and a YDLIDAR X2 2D LiDAR sits on top. The frame is 3D-printed and generated from a parametric FreeCAD script.

Inspired by the Vstone LightRover E32 and the Maker's Pet Mini.

> **Status:** early prototype. The rover drives a 50 cm square using wheel odometry only (in C and in Ruby). A Wi-Fi bridge and a host-side SLAM are written and tested in simulation; they have not run with the LiDAR yet.

## Hardware
| Part | Model |
|---|---|
| MCU | Seeed Studio XIAO ESP32-S3 |
| Motor driver | Toshiba TC78H653FTG (IN/IN mode) |
| Motors | MERCURY MG12A06E-060310-50 (6 V, 1:50, 7 PPR encoder) ×2 |
| Tires | FEETECH FS90R-W rubber tires (Φ60) on printed wheels |
| LiDAR | YDLIDAR X2 |
| Battery | 6× AA in a COMF BH-363-1B holder |
| Caster | Φ11.7 mm ball in a printed holder ×2 |

The motor supply (VM) comes from a 5 V L7805 regulator. Do not connect the battery directly to VM, because the TC78H653 is rated for at most 7.5 V.

The frame has three Φ130 mm decks:
- **Base:** motors, battery and casters
- **Mid deck:** main board
- **Top deck:** LiDAR

The overall height is about 141 mm.

## Repository layout
```
cad/
  frame.py        Parametric frame (FreeCAD Python). Single source for the model and STLs
  frame.FCStd     Generated assembly model
  test_fit.py     Generates small test-fit pieces
  stl/            Printable parts (Base_Plate, Mid_Deck, Top_Deck, Wheel_x2, Caster_x2)
  stl/test/       Test-fit pieces (generated, not tracked)
pcb/              Main board (KiCad): 60×60 mm, M3 holes on a 50 mm pitch
firmware/         Firmware projects, one directory each (see firmware/README.md)
  components/rover/  Rover hardware shared by the C projects (rover_config.h: pins, dimensions, gains)
  c-square/       C (ESP-IDF): drives a square by wheel odometry
  ruby-square/    Ruby (espinel): the same square run
  c-slam-bridge/  C (ESP-IDF): Wi-Fi bridge that streams odometry and LiDAR data to a SLAM host
host/
  ruby-slam/      Host-side SLAM in plain Ruby, with a simulator to try it without the hardware
```

## Building the frame
Requires FreeCAD (`freecadcmd`).

```sh
freecadcmd cad/frame.py      # writes cad/frame.FCStd and cad/stl/*.stl
freecadcmd cad/test_fit.py   # writes cad/stl/test/*.stl
```

All dimensions are constants at the top of `cad/frame.py`. The base bottom is flat, so it prints without supports. The wheels need supports for the outer rim.

## Firmware
Each directory under `firmware/` is a project of its own, named `<language>-<purpose>`. See [firmware/README.md](firmware/README.md) for the list and how to build each one.

| Project | |
|---|---|
| [`c-square`](firmware/c-square/) | Drives a 50 cm square by wheel odometry (feedforward + PI wheel speed control at 100 Hz, trapezoidal profiles) |
| [`ruby-square`](firmware/ruby-square/) | The same square run written in Ruby, compiled ahead of time by [espinel](https://github.com/yuuu/espinel) |
| [`c-slam-bridge`](firmware/c-slam-bridge/) | Joins Wi-Fi, streams odometry and the raw LiDAR bytes over UDP, and drives at the velocity the host commands |

> ⚠️ `c-square` and `ruby-square` start driving right after reset, including right after flashing. USB power alone can spin the motors, so lift the rover before flashing.

## SLAM (host side)
[`host/ruby-slam`](host/ruby-slam/) is a small scan-matching SLAM in plain Ruby (no gems). It talks to `c-slam-bridge`, and it comes with a simulated rover and room, so the whole pipeline can be tried before the LiDAR is mounted:

```sh
cd host/ruby-slam
bin/simulate                         # offline: writes out/map.png and compares SLAM with the ground truth
bin/fake_rover &                     # a stand-in for the rover on UDP
bin/slam_bridge 127.0.0.1 --auto     # the same program you point at the real rover
```

## Roadmap
- Run `c-slam-bridge` and `host/ruby-slam` with the LiDAR mounted; check the X2's angle offset
- Loop closure for the SLAM
- Tune the wheel speed controller
- Add a power switch, and replace the L7805 with a switching regulator

## License
MIT. See [LICENSE](LICENSE). This covers the firmware, the CAD scripts and generated models, and the KiCad board files.
