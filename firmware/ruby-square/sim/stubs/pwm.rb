# frozen_string_literal: true

require_relative "sim_world"

# espinel's PWM, driving SimWorld's motors
class PWM
  def initialize(pin, frequency: nil, duty: 50)
    @pin = pin
    SimWorld.duty[pin] = duty.to_f
  end

  def duty(percent)
    SimWorld.duty[@pin] = percent.to_f.clamp(0.0, 100.0)
    nil
  end
end
