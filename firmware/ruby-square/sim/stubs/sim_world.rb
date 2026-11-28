# frozen_string_literal: true

# The simulated rover behind the stubbed espinel packages: a clock that
# sleep advances, two motors that follow the duty the PWM pins set, and the
# encoder counts their wheels turn.
module SimWorld
  DT = 0.001
  MOTOR_TAU = 0.05 # s, the motors' time constant
  MAX_SPEED = 0.62 # m/s at full duty (rover_config.rb assumes 0.60)

  @time = 0.0
  @duty = Hash.new(100.0) # by pin: percent
  @speed = Hash.new(0.0)  # by motor pins: m/s
  @count = Hash.new(0.0)  # by encoder pin_a: counts
  @truth = [0.0, 0.0, 0.0]
  @button_at = nil

  class << self
    attr_accessor :time, :button_at
    attr_reader :duty, :truth

    def count(pin_a) = @count[pin_a].round

    # IN/IN mode: forward is IN1 high, IN2 PWM; the fraction of IN2 low is the duty
    def motor_duty(in1, in2) = (@duty[in1] - @duty[in2]) / 100.0

    def advance(seconds)
      (seconds / DT).round.times { step }
    end

    def step
      @time += DT
      per_count = Math::PI * WHEEL_DIAMETER_M / ENC_COUNTS_PER_REV
      # left and right as rover_config.rb maps them, including the inverted motors and encoders
      wheels = [[PIN_MOTOR_B_IN1, PIN_MOTOR_B_IN2, PIN_ENC_B_A, MOTOR_L_INVERT, ENC_L_INVERT],
                [PIN_MOTOR_A_IN1, PIN_MOTOR_A_IN2, PIN_ENC_A_A, MOTOR_R_INVERT, ENC_R_INVERT]]
      wheels = wheels.reverse if LEFT_IS_A
      dist = wheels.map do |in1, in2, enc, motor_inv, enc_inv|
        target = motor_duty(in1, in2) * MAX_SPEED * (motor_inv ? -1 : 1)
        @speed[in1] += (target - @speed[in1]) * DT / MOTOR_TAU
        d = @speed[in1] * DT
        @count[enc] += d / per_count * (enc_inv ? -1 : 1)
        d
      end
      ds = (dist[0] + dist[1]) / 2
      dth = (dist[1] - dist[0]) / TRACK_WIDTH_M
      @truth[0] += ds * Math.cos(@truth[2] + dth / 2)
      @truth[1] += ds * Math.sin(@truth[2] + dth / 2)
      @truth[2] += dth
    end
  end
end

module Kernel
  def sleep(seconds = nil)
    SimWorld.advance(seconds.to_f)
    seconds.to_i
  end
end

module Process
  def self.clock_gettime(*) = SimWorld.time
end
