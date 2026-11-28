# frozen_string_literal: true

module RoverSlam
  # A floor plan of wall segments for the simulator, and what a beam hits in it.
  class World
    attr_reader :segments

    # segments: [[x1, y1, x2, y2], ...] in metres
    def initialize(segments)
      @segments = segments
    end

    # A 4 x 3 m room around the start point with a few boxes and a pillar in it:
    # enough features for scan matching, and a corridor-like stretch to make it work.
    def self.sample_room
      segs = []
      rect = lambda do |x0, y0, x1, y1|
        segs << [x0, y0, x1, y0] << [x1, y0, x1, y1] << [x1, y1, x0, y1] << [x0, y1, x0, y0]
      end
      rect.call(-0.8, -1.0, 3.2, 2.0)       # the room
      rect.call(1.0, 0.6, 1.5, 1.0)         # a box in the middle
      rect.call(2.6, -1.0, 3.2, -0.4)       # a cabinet in the corner
      rect.call(-0.8, 1.5, 0.2, 2.0)        # a shelf along the far wall
      segs << [0.6, -1.0, 0.6, -0.5]        # a partition sticking out of the wall
      rect.call(2.0, 1.2, 2.1, 1.3)         # a thin pillar
      new(segs)
    end

    # Distance from (x, y) along the angle to the nearest wall, or nil past max_range
    def raycast(x, y, angle, max_range)
      dx = Math.cos(angle)
      dy = Math.sin(angle)
      best = nil
      @segments.each do |x1, y1, x2, y2|
        ex = x2 - x1
        ey = y2 - y1
        denom = dx * ey - dy * ex
        next if denom.abs < 1e-12

        wx = x1 - x
        wy = y1 - y
        t = (wx * ey - wy * ex) / denom # along the beam
        u = (wx * dy - wy * dx) / denom # along the segment
        next if t <= 0 || u.negative? || u > 1

        best = t if best.nil? || t < best
      end
      best && best <= max_range ? best : nil
    end

    def bounds
      xs = @segments.flat_map { |s| [s[0], s[2]] }
      ys = @segments.flat_map { |s| [s[1], s[3]] }
      [xs.min, ys.min, xs.max, ys.max]
    end
  end
end
