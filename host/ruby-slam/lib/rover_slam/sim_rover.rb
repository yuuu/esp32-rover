# frozen_string_literal: true

require_relative "config"
require_relative "pose"
require_relative "protocol"
require_relative "ydlidar_x2"

module RoverSlam
  # A simulated rover that speaks the bridge firmware's protocol: give it
  # velocity commands, step it, and it answers the UDP packets the real one
  # would send (odometry and raw X2 bytes). Its wheel odometry drifts like a
  # real one (a scale error on each wheel and noise), and its LiDAR sees the
  # World from the true pose.
  class SimRover
    ODOM_PERIOD = 0.02
    LIDAR_FLUSH = 0.02
    CMD_TIMEOUT = 0.5
    LINEAR_ACCEL = 0.5
    ANGULAR_ACCEL = 6.0
    SCAN_HZ = 7.0
    SAMPLE_RATE = 3000.0 # samples per second
    RANGE_NOISE_M = 0.01
    DROPOUT = 0.03
    MAX_RANGE_M = 8.0

    attr_reader :truth, :odom, :time

    # wheel_scale: how far each wheel really goes for what its encoder says [left, right]
    def initialize(world, start: Pose.zero, wheel_scale: [1.003, 0.998], wheel_noise: 0.02, seed: 1)
      @world = world
      @truth = start.dup
      @odom = Pose.zero
      @wheel_scale = wheel_scale
      @wheel_noise = wheel_noise
      @rng = Random.new(seed)
      @time = 0.0
      @v = 0.0
      @w = 0.0
      @cmd = [0.0, 0.0]
      @cmd_time = -1.0
      @meas_v = 0.0
      @meas_w = 0.0
      @odom_due = 0.0
      @lidar_due = LIDAR_FLUSH
      @phase = 0.0 # raw X2 angle [deg]
      @samples = []
      @lidar_bytes = +"".b
      @seq = Hash.new(0)
    end

    def command(v, w)
      @cmd = [v.clamp(-Config::MAX_SPEED, Config::MAX_SPEED), w.clamp(-Config::MAX_TURN_RATE, Config::MAX_TURN_RATE)]
      @cmd_time = @time
    end

    def stopped? = @time - @cmd_time > CMD_TIMEOUT || @cmd_time.negative?

    # Advance dt seconds; answers the packets sent meanwhile
    def step(dt)
      out = []
      move(dt)
      sweep(dt)
      @time += dt
      if @time >= @odom_due
        @odom_due += ODOM_PERIOD
        out << odom_packet
      end
      if @time >= @lidar_due
        @lidar_due += LIDAR_FLUSH
        unless @lidar_bytes.empty?
          out << Protocol.lidar(@lidar_bytes, t_ms: t_ms, seq: next_seq(:lidar))
          @lidar_bytes = +"".b
        end
      end
      out
    end

    def t_ms = (@time * 1000).round

    private

    def next_seq(kind) = (@seq[kind] += 1)

    def gauss
      Math.sqrt(-2 * Math.log(1 - @rng.rand)) * Math.cos(2 * Math::PI * @rng.rand)
    end

    def move(dt)
      tv, tw = stopped? ? [0.0, 0.0] : @cmd
      @v += (tv - @v).clamp(-LINEAR_ACCEL * dt, LINEAR_ACCEL * dt)
      @w += (tw - @w).clamp(-ANGULAR_ACCEL * dt, ANGULAR_ACCEL * dt)
      half = Config::TRACK_WIDTH_M / 2
      # what the wheels really do, then what their encoders say
      dl = (@v - @w * half) * dt
      dr = (@v + @w * half) * dt
      integrate(@truth, dl, dr)
      el = dl / @wheel_scale[0] * (1 + @wheel_noise * gauss)
      er = dr / @wheel_scale[1] * (1 + @wheel_noise * gauss)
      integrate(@odom, el, er)
      @meas_v = (el + er) / 2 / dt
      @meas_w = (er - el) / Config::TRACK_WIDTH_M / dt
    end

    def integrate(pose, dl, dr)
      ds = (dl + dr) / 2
      dth = (dr - dl) / Config::TRACK_WIDTH_M
      mid = pose.theta + dth / 2
      pose.x += ds * Math.cos(mid)
      pose.y += ds * Math.sin(mid)
      pose.theta += dth
    end

    def odom_packet
      flags = stopped? ? Protocol::FLAG_STOPPED : 0
      o = Protocol::Odom.new(t_ms: t_ms, x: @odom.x, y: @odom.y, theta: @odom.theta, v: @meas_v, w: @meas_w, flags: flags)
      Protocol.odom(o, seq: next_seq(:odom))
    end

    # The X2 turns clockwise seen from above; its raw angle advances SCAN_HZ turns a second
    def sweep(dt)
      step = 360.0 * SCAN_HZ / SAMPLE_RATE
      n = (SAMPLE_RATE * dt).round
      n.times do
        @samples << [@phase, measure(@phase)]
        flush_samples if @samples.size >= YDLidarX2::Encoder::SAMPLES_PER_PACKET
        @phase += step
        next if @phase < 360.0

        flush_samples
        @phase -= 360.0
        @lidar_bytes << YDLidarX2::Encoder.packet(1 | ((SCAN_HZ * 10).round << 1), [[@phase, 0.0]])
      end
    end

    def flush_samples
      return if @samples.empty?

      @lidar_bytes << YDLidarX2::Encoder.packet(0, @samples)
      @samples = []
    end

    # The distance [mm] the X2 reports at a raw angle, 0 for nothing
    def measure(raw_deg)
      return 0.0 if @rng.rand < DROPOUT

      lx = @truth.x + Math.cos(@truth.theta) * Config::LIDAR_X_M - Math.sin(@truth.theta) * Config::LIDAR_Y_M
      ly = @truth.y + Math.sin(@truth.theta) * Config::LIDAR_X_M + Math.cos(@truth.theta) * Config::LIDAR_Y_M
      # the reported angle includes the triangulation correction: find the distance at that angle
      d = beam(lx, ly, raw_deg)
      return 0.0 if d.nil?

      d = beam(lx, ly, raw_deg + YDLidarX2.correction_deg(d * 1000))
      return 0.0 if d.nil?

      (d + RANGE_NOISE_M * gauss) * 1000
    end

    def beam(lx, ly, x2_deg)
      angle = @truth.theta + (Config::LIDAR_ANGLE_OFFSET_DEG - x2_deg) * Math::PI / 180
      @world.raycast(lx, ly, angle, MAX_RANGE_M)
    end
  end
end
