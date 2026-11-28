require "pwm"

# One motor on the TC78H653FTG in IN/IN mode.
#
# One input is held H and PWM goes to the other, switching between drive and brake.
# Compared with drive / coast (one input held L), duty maps to speed more linearly, even at low speed.
#   Forward: IN1=H, IN2=PWM (brake while H) -> duty d is the fraction of IN2 that is L
#   Reverse: IN1=PWM, IN2=H
#   Stop:    IN1=IN2=H (brake)
class Motor
  PWM_FREQ_HZ = 20_000

  def initialize(pin_in1, pin_in2, invert)
    # 100% duty is H: brake from the start
    @in1 = PWM.new(pin_in1, frequency: PWM_FREQ_HZ, duty: 100)
    @in2 = PWM.new(pin_in2, frequency: PWM_FREQ_HZ, duty: 100)
    @invert = invert
  end

  # duty: -1.0 (reverse) to +1.0 (forward). 0 brakes
  def set(duty)
    duty = -duty if @invert
    duty = 1.0 if duty > 1.0
    duty = -1.0 if duty < -1.0
    off = (1.0 - duty.abs) * 100.0
    if duty >= 0.0
      @in1.duty(100)
      @in2.duty(off)
    else
      @in1.duty(off)
      @in2.duty(100)
    end
  end
end
