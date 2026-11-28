# frozen_string_literal: true

# Runs ../main.rb in CRuby against a simulated rover (stubs/): the espinel
# packages are stand-ins that drive and count simulated wheels, and sleep
# advances a simulated clock. The run is stopped as if by the BOOT button
# after LAPS laps' worth of time, and the true pose is printed beside the
# odometry, to check the square logic without the robot.
#
#   ruby firmware/ruby-square/sim/run.rb [seconds]
$LOAD_PATH.unshift File.join(__dir__, "stubs")
require "sim_world"

seconds = (ARGV[0] || 30).to_f
SimWorld.button_at = seconds

class StopRun < StandardError; end

# the program waits for a click after the run: end there
module ButtonStop
  def wait_click
    raise StopRun if SimWorld.time > SimWorld.button_at

    super
  end
end
require_relative "../button"
Button.prepend(ButtonStop)

begin
  load File.expand_path("../main.rb", __dir__)
rescue StopRun
  x, y, th = SimWorld.truth
  puts format("truth: x=%.3f y=%.3f theta=%.1f deg (simulated %.1f s)", x, y, th * 180 / Math::PI, SimWorld.time)
end
