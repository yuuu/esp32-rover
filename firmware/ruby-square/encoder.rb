require "pulse_counter"

# A wheel encoder counted by the PCNT at x4. The count increases when the wheel turns forward.
class Encoder
  def initialize(pin_a, pin_b, invert)
    @counter = PulseCounter.new(pin_a, pin_b)
    @invert = invert
  end

  def count
    c = @counter.count
    @invert ? -c : c
  end
end
