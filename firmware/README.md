# Firmware

Each directory here is a firmware project of its own, named `<language>-<purpose>`:
`c-` for C on ESP-IDF, `ruby-` for Ruby on [espinel](https://github.com/yuuu/espinel). Add new
projects the same way.

| Project | Language | What it does |
|---|---|---|
| [`c-square`](c-square/) | C (ESP-IDF v5.5) | Drives a 50 cm square by wheel odometry |
| [`ruby-square`](ruby-square/) | Ruby (espinel, ESP-IDF v6.1) | The same square run in Ruby |
| [`c-slam-bridge`](c-slam-bridge/) | C (ESP-IDF v5.5) | Streams odometry and LiDAR bytes to a SLAM host over Wi-Fi, drives at its commands |

## Shared code
- `components/rover/` is an ESP-IDF component the C projects share: motors (`motor`), encoders
  (`encoder`), odometry (`odometry`), wheel speed control (`drive`), the BOOT button (`button`)
  and the wheel direction check (`check`). `include/rover_config.h` holds every hardware value:
  pins, wheel mapping and direction, dimensions, wheel control gains. C projects add it with
  `set(EXTRA_COMPONENT_DIRS ../components)`, and keep their own parameters in their own header.
- Ruby projects cannot use the component; `ruby-square/rover_config.rb` mirrors
  `rover_config.h`. Keep the two the same.

## Building a C project
The project directories reach `../components`, so mount the whole `firmware/` directory:

```sh
cd firmware
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/firmware -w /firmware/c-square \
  espressif/idf:v5.5.4 idf.py build
```

Flash with the esptool command printed at the end of the build (on macOS, Docker cannot reach USB
devices, so run esptool on the host).

## Checking wheel direction
After changing the wiring, build `c-square` in check mode, which spins each wheel in turn and
prints its encoder counts:

```sh
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/firmware -w /firmware/c-square \
  espressif/idf:v5.5.4 idf.py -B build_check -DCMAKE_C_FLAGS=-DROVER_CHECK_MODE=1 build
```

Then adjust `LEFT_IS_A`, `MOTOR_*_INVERT` and `ENC_*_INVERT` in `components/rover/include/rover_config.h`
(and `ruby-square/rover_config.rb`).
