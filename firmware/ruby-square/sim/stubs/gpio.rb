# frozen_string_literal: true

require_relative "sim_world"

# espinel's GPIO: the BOOT button is pressed for 0.2 s at SimWorld.button_at
class GPIO
  IN = 1
  PULL_UP = 0

  def initialize(pin, _mode, _pull = nil)
    @pin = pin
  end

  def read
    at = SimWorld.button_at
    at && SimWorld.time >= at && SimWorld.time < at + 0.2 ? 0 : 1
  end
end
