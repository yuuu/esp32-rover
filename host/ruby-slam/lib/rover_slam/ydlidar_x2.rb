# frozen_string_literal: true

module RoverSlam
  # The YDLIDAR X2's byte stream. The X2 has no commands: once powered it
  # spins and streams packets at 115200 baud, 8N1. Each packet is
  #
  #   PH (0xAA 0x55)  CT  LSN  FSA (2)  LSA (2)  CS (2)  LSN samples (2 each)
  #
  # little-endian. CT bit 0 marks the packet at the start of a rotation (its
  # upper bits are the scan frequency in 0.1 Hz); FSA and LSA are the angles
  # of the first and last sample (bits 15-1, in 1/64 degree); a sample is
  # the distance in 1/4 mm (0: nothing measured). CS is the XOR of every
  # other 16-bit word of the packet. The angles are clockwise seen from
  # above. (After espinel's examples/lidar/ydlidar_x2.rb.)
  module YDLidarX2
    BAUDRATE = 115_200
    # the X2's triangulation: the angle a sample at distance d (mm) is off by
    ANGLE_K = 21.8
    ANGLE_D = 155.3

    # One rotation: angles [deg, clockwise from the LiDAR's front] and
    # distances [m] of the samples that measured something, and the scan
    # frequency [Hz] the start packet gave.
    Scan = Struct.new(:angles_deg, :ranges_m, :frequency, keyword_init: true) do
      def size = ranges_m.size
    end

    module_function

    def correction_deg(dist_mm)
      Math.atan(ANGLE_K * (ANGLE_D - dist_mm) / (ANGLE_D * dist_mm)) * 180.0 / Math::PI
    end

    # Turns the byte stream into rotations. feed takes bytes as they come, in
    # any pieces, and answers the rotations they completed.
    class Parser
      attr_reader :errors, :packets

      def initialize
        @buf = +"".b
        @angles = []
        @ranges = []
        @frequency = 0.0
        @started = false
        @errors = 0
        @packets = 0
      end

      def feed(bytes)
        @buf << bytes.b
        scans = []
        loop do
          i = @buf.index("\xAA\x55".b)
          if i.nil?
            # keep a trailing 0xAA, which may be the first half of a header
            @buf = @buf.end_with?("\xAA".b) ? +"\xAA".b : +"".b
            break
          end
          @buf = @buf.byteslice(i..) if i.positive?
          break if @buf.bytesize < 10

          lsn = @buf.getbyte(3)
          size = 10 + lsn * 2
          break if @buf.bytesize < size

          packet = @buf.byteslice(0, size)
          if valid?(packet)
            @packets += 1
            scan = take(packet)
            scans << scan if scan
            @buf = @buf.byteslice(size..)
          else
            @errors += 1
            @buf = @buf.byteslice(2..) # look for the next header inside it
          end
        end
        scans
      end

      private

      def valid?(packet)
        words = packet.unpack("v*")
        # PH, CT|LSN, FSA, LSA, CS, samples...
        sum = words[0] ^ words[1] ^ words[2] ^ words[3]
        words[5..].each { |w| sum ^= w }
        sum == words[4]
      end

      def take(packet)
        ct = packet.getbyte(2)
        lsn = packet.getbyte(3)
        fsa, lsa = packet.unpack("x4vv")
        if ct & 1 == 1
          done = finish if @started
          @started = true
          @frequency = (ct >> 1) / 10.0
          return done
        end
        return nil unless @started && lsn.positive?

        first = (fsa >> 1) / 64.0
        last = (lsa >> 1) / 64.0
        span = last - first
        span += 360.0 if span.negative?
        step = lsn > 1 ? span / (lsn - 1) : 0.0
        packet.byteslice(10, lsn * 2).unpack("v*").each_with_index do |raw, i|
          dist = raw / 4.0
          next if dist <= 0

          @angles << ((first + step * i + YDLidarX2.correction_deg(dist)) % 360.0)
          @ranges << dist / 1000.0
        end
        nil
      end

      def finish
        scan = Scan.new(angles_deg: @angles, ranges_m: @ranges, frequency: @frequency)
        @angles = []
        @ranges = []
        scan.size.positive? ? scan : nil
      end
    end

    # Builds the bytes an X2 sends, for the simulator and the tests.
    module Encoder
      SAMPLES_PER_PACKET = 40

      module_function

      # A start packet, then the samples in packets of up to
      # SAMPLES_PER_PACKET. samples is an Array of [raw angle deg, dist mm]
      # with the raw angles (before the triangulation correction) evenly
      # spaced and increasing.
      def rotation(samples, frequency_hz)
        out = packet(1 | ((frequency_hz * 10).round.clamp(0, 127) << 1), [[samples.first[0], 0.0]])
        samples.each_slice(SAMPLES_PER_PACKET) { |chunk| out << packet(0, chunk) }
        out
      end

      def packet(ct, samples)
        fsa = (((samples.first[0] % 360.0) * 64).round << 1) | 1
        lsa = (((samples.last[0] % 360.0) * 64).round << 1) | 1
        dists = samples.map { |_, d| (d * 4).round.clamp(0, 0xffff) }
        cs = 0x55aa ^ (ct | (samples.size << 8)) ^ fsa ^ lsa
        dists.each { |d| cs ^= d }
        [0xaa, 0x55, ct, samples.size, fsa, lsa, cs, *dists].pack("CCCCvvvv*")
      end
    end
  end
end
