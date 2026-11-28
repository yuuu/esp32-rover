# Dead reckoning for a differential-drive robot.
# The origin is the start point, x is the initial heading (forward), y is left.
# theta [rad] is counterclockwise positive and does not wrap at +-pi (continuous)
class Odometry
  attr_reader :x, :y, :theta

  def initialize(track_width)
    @track_width = track_width
    reset
  end

  def reset
    @x = 0.0
    @y = 0.0
    @theta = 0.0
  end

  # Update the pose from the distance [m] each wheel traveled since the last call
  def update(dl, dr)
    ds = (dl + dr) / 2.0
    dtheta = (dr - dl) / @track_width
    # Assume motion along the mid-interval heading (2nd-order Runge-Kutta)
    mid = @theta + dtheta / 2.0
    @x += ds * Math.cos(mid)
    @y += ds * Math.sin(mid)
    @theta += dtheta
  end
end
