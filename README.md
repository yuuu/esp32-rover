# esp32-rover

A small, low-cost differential-drive rover for experimenting with SLAM. A Seeed XIAO ESP32-S3 drives two geared DC motors with encoders, and a YDLIDAR X2 2D LiDAR sits on top. The frame is 3D-printed and generated from a parametric FreeCAD script.

Inspired by the Vstone LightRover E32 and the Maker's Pet Mini.

> **Status:** early prototype. The firmware currently drives a 50 cm square using wheel odometry only. The LiDAR is mounted and wired but not used yet.

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
firmware/         ESP-IDF firmware (C)
  main/config.h   Pins, wheel mapping, dimensions, control gains, motion parameters
```

## Building the frame
Requires FreeCAD (`freecadcmd`).

```sh
freecadcmd cad/frame.py      # writes cad/frame.FCStd and cad/stl/*.stl
freecadcmd cad/test_fit.py   # writes cad/stl/test/*.stl
```

All dimensions are constants at the top of `cad/frame.py`. The base bottom is flat, so it prints without supports. The wheels need supports for the outer rim.

## Firmware
Built with ESP-IDF v5.5. Docker is the easiest way:

```sh
cd firmware
docker run --rm -v "$PWD":/project -w /project espressif/idf:v5.5.4 idf.py build
```

Flash with `idf.py flash`, or with the esptool command printed at the end of the build. On macOS, Docker cannot reach USB devices, so run esptool on the host.

### What it does
- Reads both encoders with the PCNT peripheral in 4× quadrature mode (1400 counts per wheel revolution).
- Estimates the pose (x, y, θ) from differential-drive odometry.
- Controls each wheel's speed with feedforward plus a PI controller, at 100 Hz.
- Drives a square: it goes straight, pauses, turns 90° in place, and repeats. Speed and turn rate follow a trapezoidal profile.

Behavior:
- The rover starts 3 s after power-on (`AUTO_START_MS`).
- Pressing the BOOT button while driving stops it. Pressing it again starts a new run.
- It keeps looping by default (`SQUARE_LAPS = 0`).

> ⚠️ The rover starts driving right after reset, including right after flashing. USB power alone can spin the motors, so lift the rover before flashing.

### Checking wheel direction
After changing the wiring, build the check mode, which spins each wheel in turn and prints its encoder counts:

```sh
idf.py -B build_check -DCMAKE_C_FLAGS=-DROVER_CHECK_MODE=1 build
```

Then adjust `LEFT_IS_A`, `MOTOR_*_INVERT` and `ENC_*_INVERT` in `config.h`.

## Roadmap
- Read the LiDAR scans and build a map with SLAM
- Tune the wheel speed controller
- Add a power switch, and replace the L7805 with a switching regulator

## License
MIT. See [LICENSE](LICENSE). This covers the firmware, the CAD scripts and generated models, and the KiCad board files.
