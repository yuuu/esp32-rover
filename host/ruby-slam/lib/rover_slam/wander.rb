# frozen_string_literal: true

module RoverSlam
  # A simple autonomous driver for mapping without a keyboard: drive forward,
  # and turn away when something is close in front, toward the more open side.
  class Wander
    def initialize(speed: 0.15, turn_rate: 1.2, clearance_m: 0.40, side_m: 0.22)
      @speed = speed
      @turn_rate = turn_rate
      @clearance = clearance_m
      @side = side_m
      @turning = 0.0
    end

    # points: [x, y] in the robot frame (Slam#robot_points). Answers [v, w].
    def command(points)
      front = points.select { |x, y| x.positive? && y.abs < @side }.map(&:first).min || Float::INFINITY
      if front < @clearance || (@turning != 0.0 && front < @clearance * 1.6)
        if @turning.zero?
          left = points.count { |x, y| y.positive? && Math.hypot(x, y) > 0.8 }
          right = points.count { |x, y| y.negative? && Math.hypot(x, y) > 0.8 }
          @turning = left >= right ? 1.0 : -1.0
        end
        [0.0, @turning * @turn_rate]
      else
        @turning = 0.0
        [front < 2 * @clearance ? @speed / 2 : @speed, 0.0]
      end
    end
  end
end
