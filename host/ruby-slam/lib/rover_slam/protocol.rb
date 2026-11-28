# frozen_string_literal: true

module RoverSlam
  # UDP packets between the rover and the host. The other side is
  # firmware/c-slam-bridge/main/protocol.h: keep the two the same.
  # Little-endian, packed: a header of magic 'R', type, seq (u16) and t_ms
  # (u32), then the payload.
  module Protocol
    MAGIC = 0x52
    CMD = "C".ord   # host -> rover: v, w (float32)
    ODOM = "O".ord  # rover -> host: x, y, theta, v, w (float32), flags (u8)
    LIDAR = "L".ord # rover -> host: raw YDLIDAR X2 bytes
    HEADER = "CCvV"
    HEADER_SIZE = 8
    DEFAULT_PORT = 5600

    FLAG_STOPPED = 0x01
    FLAG_ESTOP = 0x02

    Odom = Struct.new(:t_ms, :x, :y, :theta, :v, :w, :flags, keyword_init: true)
    Lidar = Struct.new(:t_ms, :bytes, keyword_init: true)
    Command = Struct.new(:t_ms, :v, :w, keyword_init: true)

    module_function

    def header(type, seq, t_ms) = [MAGIC, type, seq & 0xffff, t_ms & 0xffffffff].pack(HEADER)

    def command(v, w, seq: 0, t_ms: 0) = header(CMD, seq, t_ms) + [v, w].pack("e2")

    def odom(odom, seq: 0)
      header(ODOM, seq, odom.t_ms) + [odom.x, odom.y, odom.theta, odom.v, odom.w].pack("e5") + [odom.flags].pack("C")
    end

    def lidar(bytes, t_ms:, seq: 0) = header(LIDAR, seq, t_ms) + bytes.b

    # A packet as an Odom, Lidar or Command, or nil for anything else.
    def decode(data)
      data = data.b
      return nil if data.bytesize < HEADER_SIZE

      magic, type, _seq, t_ms = data.unpack(HEADER)
      return nil if magic != MAGIC

      body = data.byteslice(HEADER_SIZE..)
      case type
      when ODOM
        return nil if body.bytesize < 21

        x, y, theta, v, w = body.unpack("e5")
        Odom.new(t_ms: t_ms, x: x, y: y, theta: theta, v: v, w: w, flags: body.getbyte(20))
      when LIDAR
        Lidar.new(t_ms: t_ms, bytes: body)
      when CMD
        return nil if body.bytesize < 8

        v, w = body.unpack("e2")
        Command.new(t_ms: t_ms, v: v, w: w)
      end
    end
  end
end
