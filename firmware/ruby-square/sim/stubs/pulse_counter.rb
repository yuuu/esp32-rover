# frozen_string_literal: true

require_relative "sim_world"

# espinel's PulseCounter, counting SimWorld's wheels
class PulseCounter
  def initialize(pin_a, _pin_b, glitch_ns: 1000, pull_up: true)
    @pin_a = pin_a
  end

  def count = SimWorld.count(@pin_a)
end
