# frozen_string_literal: true

require_relative "test_helper"

class ProtocolTest < Minitest::Test
  include RoverSlam

  def test_odom_round_trip_matches_the_firmware_layout
    odom = Protocol::Odom.new(t_ms: 123_456, x: 1.5, y: -0.25, theta: 3.0, v: 0.2, w: -1.0, flags: 3)
    data = Protocol.odom(odom, seq: 7)
    # header (8) + odom_payload_t (5 floats + flags = 21)
    assert_equal 29, data.bytesize
    assert_equal [0x52, "O".ord, 7, 123_456], data.unpack("CCvV")
    back = Protocol.decode(data)
    assert_equal 123_456, back.t_ms
    assert_in_delta 1.5, back.x
    assert_in_delta(-0.25, back.y)
    assert_in_delta 3.0, back.theta, 1e-6
    assert_equal 3, back.flags
  end

  def test_command_is_two_floats
    data = Protocol.command(0.1, -0.5, seq: 1)
    assert_equal 16, data.bytesize
    cmd = Protocol.decode(data)
    assert_in_delta 0.1, cmd.v, 1e-6
    assert_in_delta(-0.5, cmd.w, 1e-6)
  end

  def test_lidar_carries_the_bytes_as_they_are
    bytes = "\xAA\x55\x01\x00".b
    pkt = Protocol.decode(Protocol.lidar(bytes, t_ms: 5))
    assert_equal bytes, pkt.bytes
    assert_equal 5, pkt.t_ms
  end

  def test_garbage_is_ignored
    assert_nil Protocol.decode("xx")
    assert_nil Protocol.decode("\x00O\x00\x00\x00\x00\x00\x00".b + "\x00" * 21)
  end
end
