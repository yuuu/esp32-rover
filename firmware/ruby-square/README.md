# ruby-square

The [`c-square`](../c-square/) run written in Ruby: it drives a square with 50 cm sides using wheel
odometry only, with the same structure (`motor`, `encoder`, `odometry`, `drive`, `button`) and
the same parameters. It runs on [espinel](https://github.com/yuuu/espinel), which compiles Ruby
ahead of time with Spinel into an ESP-IDF firmware.

| File | |
|---|---|
| `main.rb` | the square run (state machine, 10 ms control loop) |
| `rover_config.rb` | the hardware, mirroring `../components/rover/include/rover_config.h` |
| `square_config.rb` | the driving parameters, the same as `../c-square/main/square_config.h` |
| `motor.rb`, `encoder.rb`, `odometry.rb`, `drive.rb`, `button.rb` | the rover, on espinel's `pwm`, `pulse_counter` and `gpio` packages |
| `sim/` | runs `main.rb` in CRuby against a simulated rover (no board needed) |

> ⚠️ Like `c-square`, the rover starts driving 3 s after reset. Lift it before flashing.

## Building
The encoders need espinel's `pulse_counter` package (the PCNT peripheral). Until it is merged,
use the espinel branch that adds it (`pcnt-encoder`).

```sh
cd path/to/espinel
bundle exec rake image                          # once, or after the Spinel checkout changes
bundle exec rake set-target TARGET=esp32s3      # once
bundle exec rake build PROJECT=path/to/esp32-rover/firmware/ruby-square
bundle exec rake flash monitor PROJECT=path/to/esp32-rover/firmware/ruby-square
```

To check the wheel directions, set `CHECK_MODE = true` in `square_config.rb`.

## Trying it without the robot
`sim/run.rb` loads `main.rb` in CRuby with stand-ins for espinel's packages: the PWM pins drive
simulated motors, the encoders count their wheels, and `sleep` advances a simulated clock.
It presses the BOOT button after the given number of seconds and prints the true pose:

```sh
ruby sim/run.rb 22    # about one lap
```

Under espinel's QEMU (`rake qemu PROJECT=...`) the program starts and the control loop keeps its
10 ms period, but the BOOT button reads as pressed there (no pull-up), so the run stops at once.
