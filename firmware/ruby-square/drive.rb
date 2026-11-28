require_relative "rover_config"
require_relative "motor"
require_relative "encoder"
require_relative "odometry"

# Wheel speed control (feedforward + PI) and pose update, for both wheels.
class Drive
  M_PER_COUNT = Math::PI * WHEEL_DIAMETER_M / ENC_COUNTS_PER_REV
  INTEGRAL_LIMIT = 0.1 # Keeps the integral term within about +-0.6 duty

  attr_reader :odometry, :motors, :encoders

  def initialize
    motor_a = [PIN_MOTOR_A_IN1, PIN_MOTOR_A_IN2]
    motor_b = [PIN_MOTOR_B_IN1, PIN_MOTOR_B_IN2]
    enc_a = [PIN_ENC_A_A, PIN_ENC_A_B]
    enc_b = [PIN_ENC_B_A, PIN_ENC_B_B]
    ml = LEFT_IS_A ? motor_a : motor_b
    mr = LEFT_IS_A ? motor_b : motor_a
    el = LEFT_IS_A ? enc_a : enc_b
    er = LEFT_IS_A ? enc_b : enc_a
    # index 0 is left, 1 is right
    @motors = [Motor.new(ml[0], ml[1], MOTOR_L_INVERT), Motor.new(mr[0], mr[1], MOTOR_R_INVERT)]
    @encoders = [Encoder.new(el[0], el[1], ENC_L_INVERT), Encoder.new(er[0], er[1], ENC_R_INVERT)]
    @odometry = Odometry.new(TRACK_WIDTH_M)
    @last_count = [@encoders[0].count, @encoders[1].count]
    @speed = [0.0, 0.0]    # measured speed [m/s] (after low-pass)
    @target = [0.0, 0.0]   # target speed [m/s]
    @integral = [0.0, 0.0]
    @dist = [0.0, 0.0]
    @stopped = true
  end

  # Target linear velocity v [m/s] and angular velocity w [rad/s] (counterclockwise positive)
  def set(v, w)
    @target[0] = v - w * TRACK_WIDTH_M / 2.0
    @target[1] = v + w * TRACK_WIDTH_M / 2.0
    @stopped = false
  end

  # Set the targets to 0, brake, and reset the speed controller integrators
  def stop
    2.times do |i|
      @target[i] = 0.0
      @integral[i] = 0.0
      @motors[i].set(0.0)
    end
    @stopped = true
  end

  # Call every control period: reads the encoders, updates the pose, and controls both wheel speeds
  def tick(dt)
    2.times do |i|
      count = @encoders[i].count
      @dist[i] = (count - @last_count[i]) * M_PER_COUNT
      @last_count[i] = count
      @speed[i] += WHEEL_SPEED_LPF * (@dist[i] / dt - @speed[i])
    end
    @odometry.update(@dist[0], @dist[1])
    return if @stopped # stay braked

    2.times do |i|
      err = @target[i] - @speed[i]
      integral = @integral[i] + err * dt
      integral = INTEGRAL_LIMIT if integral > INTEGRAL_LIMIT
      integral = -INTEGRAL_LIMIT if integral < -INTEGRAL_LIMIT
      @integral[i] = integral
      @motors[i].set(@target[i] / WHEEL_SPEED_AT_FULL_DUTY + WHEEL_KP * err + WHEEL_KI * integral)
    end
  end
end
