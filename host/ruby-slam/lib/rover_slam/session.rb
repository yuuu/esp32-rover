# frozen_string_literal: true

require_relative "pose"
require_relative "protocol"
require_relative "slam"
require_relative "ydlidar_x2"

module RoverSlam
  # What the host does with the rover's packets, the same for the real rover,
  # the fake one and the offline simulation: decode them, keep the odometry,
  # parse the X2 stream into rotations and run SLAM on each with the
  # odometry interpolated to the middle of the rotation.
  class Session
    attr_reader :slam, :parser, :odom, :last_scan, :last_step, :packets
    # the rover's clock [ms] at the middle of the last rotation, which the last step is for
    attr_reader :last_scan_t_ms

    def initialize(slam: Slam.new)
      @slam = slam
      @parser = YDLidarX2::Parser.new
      @history = [] # [t_ms, Pose], oldest first
      @odom = nil
      @last_scan = nil
      @last_step = nil
      @packets = Hash.new(0)
    end

    # One UDP payload. Answers the SLAM steps it completed.
    def receive(data)
      pkt = Protocol.decode(data)
      case pkt
      when Protocol::Odom
        @packets[:odom] += 1
        @odom = pkt
        @history << [pkt.t_ms, Pose.new(pkt.x, pkt.y, pkt.theta)]
        @history.shift while @history.size > 200
        []
      when Protocol::Lidar
        @packets[:lidar] += 1
        @parser.feed(pkt.bytes).filter_map { |scan| process(scan, pkt.t_ms) }
      else
        @packets[:other] += 1
        []
      end
    end

    # The odometry at t_ms, interpolated (the nearest end outside the history)
    def odom_at(t_ms)
      return nil if @history.empty?
      return @history.first[1] if t_ms <= @history.first[0]
      return @history.last[1] if t_ms >= @history.last[0]

      @history.each_cons(2) do |(ta, a), (tb, b)|
        next if t_ms > tb

        f = tb == ta ? 0.0 : (t_ms - ta).to_f / (tb - ta)
        return Pose.new(a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, a.theta + (b.theta - a.theta) * f)
      end
    end

    private

    def process(scan, t_ms)
      @last_scan = scan
      period_ms = scan.frequency.positive? ? 1000.0 / scan.frequency : 140.0
      @last_scan_t_ms = t_ms - period_ms / 2
      odom = odom_at(@last_scan_t_ms)
      return nil if odom.nil?

      @last_step = @slam.process(scan, odom)
    end
  end
end
