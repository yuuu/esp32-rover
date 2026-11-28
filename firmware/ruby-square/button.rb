require "gpio"
require_relative "rover_config"

# The XIAO BOOT button, used to start and stop runs
class Button
  def initialize
    @pin = GPIO.new(PIN_START_BUTTON, GPIO::IN, GPIO::PULL_UP)
  end

  def pressed? = @pin.read == 0

  # Wait until the button is released (a held button would otherwise count as the next click)
  def wait_release
    sleep 0.02 while pressed?
  end

  # Wait for a press and release, then one more second so the hand is off the robot
  def wait_click
    sleep 0.02 until pressed?
    wait_release
    sleep 1
  end
end
