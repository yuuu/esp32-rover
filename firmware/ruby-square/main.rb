# Drives a square with side SQUARE_SIDE_M using only encoder odometry: the Ruby version of
# firmware/c-square, run by espinel (https://github.com/yuuu/espinel).
#
# Starts AUTO_START_MS after power-on. Pressing the BOOT button while driving stops the rover;
# pressing it again restarts from there. Stops after SQUARE_LAPS laps (0 runs until stopped).
# With the start point as the origin and the initial heading as the x axis, the rover heads for
# the corners (0,0) -> (L,0) -> (L,+-L) -> (0,+-L) -> (0,0) in order.
# Each side repeats "straight -> stop -> turn 90 deg in place -> stop". While driving straight it
# steers toward the next corner computed from the pose, so turn errors and lateral drift are
# corrected on the next side.
#
#   bundle exec rake flash monitor PROJECT=<this directory>   (in the espinel checkout)
require_relative "rover_config"
require_relative "square_config"
require_relative "drive"
require_relative "button"

DT = CONTROL_PERIOD_MS / 1000.0
TURN_SIGN = SQUARE_TURN_LEFT ? 1.0 : -1.0
CORNER_X = [0.0, 1.0, 1.0, 0.0]
CORNER_Y = [0.0, 0.0, 1.0, 1.0]

def now = Process.clock_gettime(Process::CLOCK_MONOTONIC)

def clamp(v, lo, hi)
  return lo if v < lo
  return hi if v > hi
  v
end

def min(a, b) = a < b ? a : b
def max(a, b) = a > b ? a : b

def wrap_pi(a) = Math.atan2(Math.sin(a), Math.cos(a))

def deg(rad) = rad * 180.0 / Math::PI

def corner_x(i) = CORNER_X[i % 4] * SQUARE_SIDE_M
def corner_y(i) = CORNER_Y[i % 4] * SQUARE_SIDE_M * TURN_SIGN

def pose_text(odom)
  format("x=%.3f y=%.3f theta=%.1f deg", odom.x, odom.y, deg(odom.theta))
end

# Turn each wheel briefly and print how the count changes (for checking directions in rover_config.rb)
def check_wheels(drive)
  names = ["left", "right"]
  2.times do |w|
    before = drive.encoders[w].count
    puts "Turning the #{names[w]} wheel forward (1 s)"
    drive.motors[w].set(0.3)
    sleep 1
    drive.motors[w].set(0.0)
    sleep 0.3
    delta = drive.encoders[w].count - before
    puts format("%s: count %+d (%.2f rev)", names[w], delta, delta.to_f / ENC_COUNTS_PER_REV)
  end
  puts "Check: did the named wheel turn (else LEFT_IS_A), did it turn forward " \
       "(else MOTOR_*_INVERT), is the count positive (else ENC_*_INVERT)"
end

def run_square(drive, button)
  odom = drive.odometry
  state = :straight
  side = 0        # index of the current side (4 per lap; corners and heading follow from it)
  v = 0.0         # commanded linear speed (kept for the acceleration limit)
  w = 0.0         # commanded angular velocity for turns
  pause_ticks = 0
  log_ticks = 0
  next_wake = now

  puts format("Start: square with %.2f m sides", SQUARE_SIDE_M)
  while state != :done
    next_wake += DT
    wait = next_wake - now
    if wait > 0
      sleep wait
    else
      next_wake = now # fell behind: do not try to catch up
    end
    drive.tick(DT)
    if button.pressed?
      puts "Stopped by BOOT button"
      break
    end

    case state
    when :straight
      sx = corner_x(side)
      sy = corner_y(side)
      tx = corner_x(side + 1)
      ty = corner_y(side + 1)
      # remaining distance along the side
      ux = (tx - sx) / SQUARE_SIDE_M
      uy = (ty - sy) / SQUARE_SIDE_M
      remain = (tx - odom.x) * ux + (ty - odom.y) * uy
      if remain <= 0.0
        drive.stop
        v = 0.0
        puts "Side #{side} done: #{pose_text(odom)}"
        state = :pause_before_turn
        pause_ticks = PAUSE_MS / CONTROL_PERIOD_MS
      else
        # limit acceleration and keep a speed that can stop in the remaining distance (trapezoidal profile)
        v = min(v + LINEAR_ACCEL * DT, CRUISE_SPEED)
        v = min(v, Math.sqrt(2.0 * LINEAR_ACCEL * remain))
        v = max(v, MIN_SPEED)
        # head for the target corner; near the end the bearing is unstable, so follow the side direction
        heading = remain > 0.05 ? Math.atan2(ty - odom.y, tx - odom.x) : Math.atan2(uy, ux)
        err = wrap_pi(heading - odom.theta)
        drive.set(v, clamp(HEADING_KP * err, -TURN_RATE, TURN_RATE))
      end
    when :turn
      # the target heading advances +-90 deg per side (theta is continuous, so no wrapping)
      target = (side + 1) * TURN_SIGN * Math::PI / 2.0
      err = target - odom.theta
      if err.abs < TURN_TOLERANCE_RAD
        drive.stop
        w = 0.0
        puts format("Turn %d done: theta=%.1f deg", side, deg(odom.theta))
        side += 1
        if side % 4 == 0
          puts format("Lap %d done: x=%.3f y=%.3f (offset from start %.1f mm)", side / 4,
                      odom.x, odom.y, 1000.0 * Math.sqrt(odom.x * odom.x + odom.y * odom.y))
        end
        finished = SQUARE_LAPS > 0 && side >= 4 * SQUARE_LAPS
        state = finished ? :done : :pause_before_straight
        pause_ticks = PAUSE_MS / CONTROL_PERIOD_MS
      else
        rate = min(w.abs + ANGULAR_ACCEL * DT, TURN_RATE)
        rate = min(rate, Math.sqrt(2.0 * ANGULAR_ACCEL * err.abs))
        rate = max(rate, MIN_TURN_RATE)
        w = err < 0 ? -rate : rate
        drive.set(0.0, w)
      end
    when :pause_before_turn, :pause_before_straight
      pause_ticks -= 1
      state = state == :pause_before_turn ? :turn : :straight if pause_ticks <= 0
    end

    log_ticks += 1
    if log_ticks >= 500 / CONTROL_PERIOD_MS
      log_ticks = 0
      puts pose_text(odom)
    end
  end

  drive.stop
  puts "Stopped: #{pose_text(odom)}"
  button.wait_release # a held button would otherwise skip the next start wait
end

drive = Drive.new
drive.stop
button = Button.new
if AUTO_START_MS > 0
  puts "Starting in #{AUTO_START_MS} ms"
  sleep AUTO_START_MS / 1000.0
else
  puts "Press the BOOT button to start"
  button.wait_click
end
loop do
  if CHECK_MODE
    check_wheels(drive)
  else
    drive.odometry.reset
    run_square(drive, button)
  end
  puts "Press the BOOT button to start"
  button.wait_click
end
