# frozen_string_literal: true

require_relative "test_helper"

class YDLidarX2Test < Minitest::Test
  include RoverSlam

  # One rotation of samples 1 degree apart at the given distance [mm]
  def rotation(dist_mm = 1000.0, start: 0.0)
    YDLidarX2::Encoder.rotation(Array.new(360) { |i| [start + i, dist_mm] }, 7.0)
  end

  def test_a_rotation_completes_at_the_next_start_packet
    parser = YDLidarX2::Parser.new
    assert_empty parser.feed(rotation)
    scans = parser.feed(rotation)
    assert_equal 1, scans.size
    scan = scans.first
    assert_equal 360, scan.size
    assert_in_delta 7.0, scan.frequency
    assert(scan.ranges_m.all? { |r| (r - 1.0).abs < 1e-9 })
    assert_equal 0, parser.errors
  end

  def test_angles_include_the_triangulation_correction
    parser = YDLidarX2::Parser.new
    parser.feed(rotation(500.0))
    scan = parser.feed(rotation(500.0)).first
    corr = YDLidarX2.correction_deg(500.0)
    assert_in_delta((10 + corr) % 360, scan.angles_deg[10], 0.02)
  end

  def test_bytes_may_come_in_any_pieces
    stream = rotation + rotation
    whole = YDLidarX2::Parser.new.feed(stream)
    parser = YDLidarX2::Parser.new
    pieces = []
    stream.bytes.each_slice(7) { |s| pieces.concat(parser.feed(s.pack("C*"))) }
    assert_equal whole.map(&:ranges_m), pieces.map(&:ranges_m)
  end

  def test_a_corrupt_packet_is_dropped_and_the_parser_resyncs
    stream = rotation(800.0)
    bad = stream.dup
    bad.setbyte(30, bad.getbyte(30) ^ 0xff) # inside the first data packet
    parser = YDLidarX2::Parser.new
    parser.feed("\x01\x02\xAA".b) # noise, ending in half a header
    parser.feed(bad)
    scan = parser.feed(rotation(800.0)).first
    assert_equal 1, parser.errors
    assert_equal 360 - YDLidarX2::Encoder::SAMPLES_PER_PACKET, scan.size
  end

  def test_zero_distances_are_not_samples
    parser = YDLidarX2::Parser.new
    parser.feed(YDLidarX2::Encoder.rotation(Array.new(100) { |i| [i * 3.6, i.even? ? 0.0 : 900.0] }, 7.0))
    scan = parser.feed(rotation).first
    assert_equal 50, scan.size
  end
end
