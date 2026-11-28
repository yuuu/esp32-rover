# frozen_string_literal: true

require_relative "pose"

module RoverSlam
  # Finds the pose at which a scan best fits the map: a brute-force search
  # over a window around the guess (correlative scan matching) on the map's
  # likelihood field, then a hill climb with interpolated scores for the
  # last bit of precision.
  class ScanMatcher
    def initialize(window_m: 0.15, window_deg: 8.0, step_deg: 1.0, max_points: 180)
      @window_m = window_m
      @window = window_deg * Math::PI / 180
      @step = step_deg * Math::PI / 180
      @max_points = max_points
    end

    # points: [x, y] in the robot frame. Answers [pose, score], the score the
    # mean likelihood of the points (0 to 1).
    def match(map, points, guess)
      pts = thin(points)
      best = coarse(map, pts, guess)
      refine(map, pts, best)
    end

    def score(map, pts, pose)
      c = Math.cos(pose.theta)
      s = Math.sin(pose.theta)
      total = 0.0
      pts.each { |x, y| total += map.likelihood_at(pose.x + c * x - s * y, pose.y + s * x + c * y) }
      total / pts.size
    end

    private

    def thin(points)
      return points if points.size <= @max_points

      step = points.size.to_f / @max_points
      Array.new(@max_points) { |i| points[(i * step).floor] }
    end

    # Every rotation in the window, and every whole-cell shift for each: the
    # points' cells are found once per rotation and shifted by integers.
    def coarse(map, pts, guess)
      res = map.resolution
      n = (@window_m / res).ceil
      size = map.size
      lik = map.likelihood
      best_score = -1.0
      best = guess
      steps = (@window / @step).round
      (-steps..steps).each do |k|
        th = guess.theta + k * @step
        c = Math.cos(th)
        s = Math.sin(th)
        cells = pts.map { |x, y| map.cell(guess.x + c * x - s * y, guess.y + s * x + c * y) }
        (-n..n).each do |dj|
          (-n..n).each do |di|
            total = 0.0
            cells.each do |i, j|
              ii = i + di
              jj = j + dj
              total += lik[jj * size + ii] if ii >= 0 && jj >= 0 && ii < size && jj < size
            end
            next unless total > best_score

            best_score = total
            best = Pose.new(guess.x + di * res, guess.y + dj * res, th)
          end
        end
      end
      best
    end

    def refine(map, pts, pose)
      best = score(map, pts, pose)
      dl = map.resolution / 2
      da = @step / 2
      6.times do
        improved = true
        while improved
          improved = false
          [[dl, 0, 0], [-dl, 0, 0], [0, dl, 0], [0, -dl, 0], [0, 0, da], [0, 0, -da]].each do |dx, dy, dth|
            cand = Pose.new(pose.x + dx, pose.y + dy, pose.theta + dth)
            sc = score(map, pts, cand)
            next unless sc > best

            best = sc
            pose = cand
            improved = true
          end
        end
        dl /= 2
        da /= 2
      end
      [pose, best]
    end
  end
end
