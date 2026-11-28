# frozen_string_literal: true

module RoverSlam
  # A 2D pose: x, y [m] and theta [rad], counterclockwise.
  Pose = Struct.new(:x, :y, :theta) do
    def self.zero = new(0.0, 0.0, 0.0)

    def self.wrap(a) = Math.atan2(Math.sin(a), Math.cos(a))

    # This pose followed by delta, given in this pose's frame
    def compose(delta)
      c = Math.cos(theta)
      s = Math.sin(theta)
      Pose.new(x + c * delta.x - s * delta.y, y + s * delta.x + c * delta.y, theta + delta.theta)
    end

    # The pose other is at, seen from this one: compose(relative(other)) == other
    def relative(other)
      dx = other.x - x
      dy = other.y - y
      c = Math.cos(theta)
      s = Math.sin(theta)
      Pose.new(c * dx + s * dy, -s * dx + c * dy, other.theta - theta)
    end

    def distance(other) = Math.hypot(other.x - x, other.y - y)

    def to_s = format("(%.3f, %.3f, %.1f deg)", x, y, theta * 180 / Math::PI)
  end
end
