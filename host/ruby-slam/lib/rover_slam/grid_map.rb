# frozen_string_literal: true

module RoverSlam
  # An occupancy grid in log-odds, centred on the start point, with a
  # likelihood field for scan matching: every cell that has become occupied
  # stamps a Gaussian around itself, so a point near a wall scores almost as
  # well as one on it.
  class GridMap
    HIT = 0.9
    MISS = -0.4
    LIMIT = 5.0
    OCCUPIED = 1.5 # log-odds above which a cell counts as a wall
    SIGMA_M = 0.05

    attr_reader :resolution, :size, :log_odds, :likelihood

    def initialize(resolution: 0.03, size_m: 16.0)
      @resolution = resolution
      @size = (size_m / resolution).ceil
      @half = @size / 2
      @log_odds = Array.new(@size * @size, 0.0)
      @likelihood = Array.new(@size * @size, 0.0)
      @stamped = Array.new(@size * @size, false)
      r = (3 * SIGMA_M / resolution).ceil
      @kernel = []
      (-r..r).each do |dy|
        (-r..r).each do |dx|
          d2 = (dx * dx + dy * dy) * resolution * resolution
          @kernel << [dx, dy, Math.exp(-d2 / (2 * SIGMA_M * SIGMA_M))] if dx * dx + dy * dy <= r * r
        end
      end
    end

    def cell(x, y) = [(x / @resolution).floor + @half, (y / @resolution).floor + @half]

    def inside?(i, j) = i >= 0 && j >= 0 && i < @size && j < @size

    def index(i, j) = j * @size + i

    # The world point at the middle of a cell
    def center(i, j) = [(i - @half + 0.5) * @resolution, (j - @half + 0.5) * @resolution]

    def occupied?(i, j) = inside?(i, j) && @log_odds[index(i, j)] > OCCUPIED

    def empty? = @stamped.none?

    # Insert one scan: points are world [x, y] of the hits, seen from origin [x, y]
    def insert(origin, points)
      oi, oj = cell(*origin)
      points.each do |x, y|
        hi, hj = cell(x, y)
        trace(oi, oj, hi, hj) { |i, j| add(i, j, MISS) }
        add(hi, hj, HIT)
      end
    end

    # The likelihood at a world point, interpolated between cells
    def likelihood_at(x, y)
      fx = x / @resolution + @half - 0.5
      fy = y / @resolution + @half - 0.5
      i = fx.floor
      j = fy.floor
      return 0.0 unless i >= 0 && j >= 0 && i + 1 < @size && j + 1 < @size

      ax = fx - i
      ay = fy - j
      k = index(i, j)
      l = @likelihood
      (l[k] * (1 - ax) + l[k + 1] * ax) * (1 - ay) + (l[k + @size] * (1 - ax) + l[k + @size + 1] * ax) * ay
    end

    # The part of the grid that has been seen, as [i0, j0, i1, j1], or nil
    def known_bounds
      i0 = j0 = @size
      i1 = j1 = -1
      @log_odds.each_with_index do |v, k|
        next if v.zero?

        i = k % @size
        j = k / @size
        i0 = i if i < i0
        i1 = i if i > i1
        j0 = j if j < j0
        j1 = j if j > j1
      end
      i1.negative? ? nil : [i0, j0, i1, j1]
    end

    private

    def add(i, j, delta)
      return unless inside?(i, j)

      k = index(i, j)
      v = (@log_odds[k] + delta).clamp(-LIMIT, LIMIT)
      @log_odds[k] = v
      stamp(i, j) if v > OCCUPIED && !@stamped[k]
    end

    def stamp(i, j)
      @stamped[index(i, j)] = true
      @kernel.each do |dx, dy, w|
        x = i + dx
        y = j + dy
        next unless inside?(x, y)

        k = index(x, y)
        @likelihood[k] = w if w > @likelihood[k]
      end
    end

    # Bresenham from (i0, j0) toward (i1, j1), yielding every cell before the last
    def trace(i0, j0, i1, j1)
      dx = (i1 - i0).abs
      dy = -(j1 - j0).abs
      sx = i0 < i1 ? 1 : -1
      sy = j0 < j1 ? 1 : -1
      err = dx + dy
      i = i0
      j = j0
      until i == i1 && j == j1
        yield i, j
        e2 = 2 * err
        if e2 >= dy
          err += dy
          i += sx
        end
        if e2 <= dx
          err += dx
          j += sy
        end
      end
    end
  end
end
