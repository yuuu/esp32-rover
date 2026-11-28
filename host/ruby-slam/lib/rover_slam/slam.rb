# frozen_string_literal: true

require_relative "config"
require_relative "grid_map"
require_relative "pose"
require_relative "scan_matcher"

module RoverSlam
  # Scan-to-map SLAM: each scan is matched against the map built so far,
  # starting from where the wheel odometry says the rover went since the
  # last scan, and the map is grown from the corrected pose. There is no
  # loop closure: drift that matching misses stays.
  class Slam
    MIN_SCORE = 0.25        # below this the match is not trusted, and odometry is kept
    MIN_POINTS = 60
    INSERT_DISTANCE_M = 0.05 # grow the map only after moving this far or turning this much
    INSERT_ANGLE = 5.0 * Math::PI / 180

    Step = Struct.new(:pose, :odom, :score, :matched, keyword_init: true)

    attr_reader :map, :pose, :trajectory, :odom_trajectory, :scans, :rejected

    def initialize(map: GridMap.new, matcher: ScanMatcher.new)
      @map = map
      @matcher = matcher
      @pose = nil
      @last_odom = nil
      @last_insert = nil
      @trajectory = []
      @odom_trajectory = []
      @scans = 0
      @rejected = 0
    end

    # scan: a YDLidarX2::Scan; odom: the odometry Pose at the time of the scan
    def process(scan, odom)
      points = robot_points(scan)
      @scans += 1
      if @pose.nil?
        @pose = odom.dup
        @last_odom = odom
        insert(points)
        return record(odom, 1.0, true)
      end

      guess = @pose.compose(@last_odom.relative(odom))
      @last_odom = odom
      matched = false
      score = 0.0
      if points.size >= MIN_POINTS && !@map.empty?
        found, score = @matcher.match(@map, points, guess)
        matched = score >= MIN_SCORE
      end
      @rejected += 1 unless matched
      @pose = matched ? found : guess
      insert(points) if matched && (@last_insert.distance(@pose) > INSERT_DISTANCE_M ||
                                    Pose.wrap(@pose.theta - @last_insert.theta).abs > INSERT_ANGLE)
      record(odom, score, matched)
    end

    # Scan samples as [x, y] in the robot frame
    def robot_points(scan)
      pts = []
      off = Config::LIDAR_ANGLE_OFFSET_DEG
      scan.angles_deg.each_with_index do |deg, i|
        r = scan.ranges_m[i]
        next if r < Config::LIDAR_MIN_M || r > Config::LIDAR_MAX_M

        a = (off - deg) * Math::PI / 180 # the X2 measures clockwise
        pts << [Config::LIDAR_X_M + r * Math.cos(a), Config::LIDAR_Y_M + r * Math.sin(a)]
      end
      pts
    end

    private

    def insert(points)
      c = Math.cos(@pose.theta)
      s = Math.sin(@pose.theta)
      world = points.map { |x, y| [@pose.x + c * x - s * y, @pose.y + s * x + c * y] }
      origin = [@pose.x + c * Config::LIDAR_X_M - s * Config::LIDAR_Y_M,
                @pose.y + s * Config::LIDAR_X_M + c * Config::LIDAR_Y_M]
      @map.insert(origin, world)
      @last_insert = @pose.dup
    end

    def record(odom, score, matched)
      @trajectory << @pose.dup
      @odom_trajectory << odom.dup
      Step.new(pose: @pose.dup, odom: odom, score: score, matched: matched)
    end
  end
end
