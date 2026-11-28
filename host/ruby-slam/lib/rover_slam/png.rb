# frozen_string_literal: true

require "zlib"

module RoverSlam
  # Writes an RGB image as a PNG with the standard library only.
  class Image
    attr_reader :width, :height

    def initialize(width, height, color = [255, 255, 255])
      @width = width
      @height = height
      @pixels = Array.new(width * height) { color }
    end

    def set(x, y, color)
      return unless x >= 0 && y >= 0 && x < @width && y < @height

      @pixels[y * @width + x] = color
    end

    def get(x, y) = @pixels[y * @width + x]

    def line(x0, y0, x1, y1, color)
      steps = [(x1 - x0).abs, (y1 - y0).abs, 1].max
      (0..steps).each do |k|
        set((x0 + (x1 - x0) * k.to_f / steps).round, (y0 + (y1 - y0) * k.to_f / steps).round, color)
      end
    end

    def dot(x, y, color, r = 1)
      (-r..r).each { |dy| (-r..r).each { |dx| set(x + dx, y + dy, color) } }
    end

    def to_png
      raw = +"".b
      @height.times do |y|
        raw << "\x00".b
        row = @pixels[y * @width, @width]
        raw << row.flatten.pack("C*")
      end
      "\x89PNG\r\n\x1a\n".b + chunk("IHDR", [@width, @height, 8, 2, 0, 0, 0].pack("NNCCCCC")) +
        chunk("IDAT", Zlib::Deflate.deflate(raw)) + chunk("IEND", "")
    end

    def save(path) = File.binwrite(path, to_png)

    private

    def chunk(type, data)
      [data.bytesize].pack("N") + type.b + data.b + [Zlib.crc32(type + data)].pack("N")
    end
  end

  # Draws a GridMap with trajectories on it: free cells white, unknown grey,
  # walls black; y up.
  module MapImage
    module_function

    # trajectories: { [r, g, b] => [Pose, ...] }; walls: optional [[x1, y1, x2, y2], ...] drawn faintly
    def render(map, trajectories: {}, walls: [], scale: 3, margin: 10)
      bounds = map.known_bounds || [map.size / 2 - 10, map.size / 2 - 10, map.size / 2 + 10, map.size / 2 + 10]
      i0, j0, i1, j1 = bounds
      i0 -= margin
      j0 -= margin
      i1 += margin
      j1 += margin
      w = (i1 - i0 + 1) * scale
      h = (j1 - j0 + 1) * scale
      img = Image.new(w, h, [200, 200, 200])
      (j0..j1).each do |j|
        (i0..i1).each do |i|
          next unless map.inside?(i, j)

          v = map.log_odds[map.index(i, j)]
          next if v.zero?

          p = 1.0 - 1.0 / (1.0 + Math.exp(v))
          g = (255 * (1 - p)).round.clamp(0, 255)
          x = (i - i0) * scale
          y = h - (j - j0 + 1) * scale
          scale.times { |dy| scale.times { |dx| img.set(x + dx, y + dy, [g, g, g]) } }
        end
      end
      to_px = lambda do |px, py|
        i, j = map.cell(px, py)
        [((i - i0) + 0.5) * scale, h - ((j - j0) + 0.5) * scale]
      end
      walls.each do |x1, y1, x2, y2|
        a = to_px.call(x1, y1)
        b = to_px.call(x2, y2)
        img.line(a[0].round, a[1].round, b[0].round, b[1].round, [230, 160, 60])
      end
      trajectories.each do |color, poses|
        poses.each_cons(2) do |p, q|
          a = to_px.call(p.x, p.y)
          b = to_px.call(q.x, q.y)
          img.line(a[0].round, a[1].round, b[0].round, b[1].round, color)
        end
        next if poses.empty?

        e = to_px.call(poses.last.x, poses.last.y)
        img.dot(e[0].round, e[1].round, color, 2)
      end
      img
    end
  end
end
