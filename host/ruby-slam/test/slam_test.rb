# frozen_string_literal: true

require_relative "test_helper"

class SlamTest < Minitest::Test
  include RoverSlam

  def test_compose_and_relative_are_inverse
    a = Pose.new(1.0, 2.0, 0.7)
    b = Pose.new(-0.5, 0.3, -2.0)
    c = a.compose(a.relative(b))
    assert_in_delta b.x, c.x
    assert_in_delta b.y, c.y
    assert_in_delta b.theta, c.theta
  end

  def test_raycast_hits_the_nearest_wall
    world = World.new([[1.0, -1.0, 1.0, 1.0], [2.0, -1.0, 2.0, 1.0]])
    assert_in_delta 1.0, world.raycast(0.0, 0.0, 0.0, 5.0)
    assert_nil world.raycast(0.0, 0.0, Math::PI, 5.0)
    assert_nil world.raycast(0.0, 0.0, 0.0, 0.5)
  end

  def test_insert_marks_the_hit_occupied_and_the_way_free
    map = GridMap.new(resolution: 0.05, size_m: 4.0)
    3.times { map.insert([0.0, 0.0], [[1.0, 0.0]]) }
    assert map.occupied?(*map.cell(1.0, 0.0))
    assert map.log_odds[map.index(*map.cell(0.5, 0.0))].negative?
    assert_in_delta 1.0, map.likelihood_at(*map.center(*map.cell(1.0, 0.0))), 1e-9
  end

  # A scan of the sample room from one pose, matched starting from a guess
  # that is off, comes back to the true pose
  def test_scan_matching_recovers_an_offset
    world = World.sample_room
    truth = Pose.new(0.3, 0.2, 0.1)
    points = Array.new(360) do |i|
      a = i * Math::PI / 180
      d = world.raycast(truth.x, truth.y, truth.theta + a, 6.0)
      d && [d * Math.cos(a), d * Math.sin(a)]
    end.compact
    map = GridMap.new
    c = Math.cos(truth.theta)
    s = Math.sin(truth.theta)
    2.times { map.insert([truth.x, truth.y], points.map { |x, y| [truth.x + c * x - s * y, truth.y + s * x + c * y] }) }

    guess = Pose.new(truth.x + 0.08, truth.y - 0.06, truth.theta + 0.07)
    found, score = ScanMatcher.new.match(map, points, guess)
    assert_operator score, :>, 0.8
    assert_in_delta truth.x, found.x, 0.01
    assert_in_delta truth.y, found.y, 0.01
    assert_in_delta truth.theta, found.theta, 0.01
  end

  def test_session_interpolates_odometry
    session = Session.new
    session.receive(Protocol.odom(Protocol::Odom.new(t_ms: 100, x: 0.0, y: 0.0, theta: 0.0, v: 0, w: 0, flags: 0)))
    session.receive(Protocol.odom(Protocol::Odom.new(t_ms: 200, x: 1.0, y: 0.0, theta: 1.0, v: 0, w: 0, flags: 0)))
    mid = session.odom_at(150)
    assert_in_delta 0.5, mid.x
    assert_in_delta 0.5, mid.theta
    assert_in_delta 1.0, session.odom_at(999).x
  end

  # The whole pipeline, briefly: the simulated rover's packets through the
  # session keep SLAM within a few centimetres of the truth
  def test_simulated_run_tracks_the_truth
    rover = SimRover.new(World.sample_room, seed: 3)
    session = Session.new
    1000.times do |k|
      rover.command(0.15, k > 500 ? 0.4 : 0.0) if (k % 10).zero?
      rover.step(0.01).each { |p| session.receive(p) }
    end
    assert_operator session.slam.scans, :>, 50
    assert_equal 0, session.slam.rejected
    assert_operator rover.truth.distance(session.slam.pose), :<, 0.05
  end
end
