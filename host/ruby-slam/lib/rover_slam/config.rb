# frozen_string_literal: true

module RoverSlam
  # The rover as the host sees it. Dimensions follow cad/frame.py and
  # firmware/components/rover/include/rover_config.h.
  module Config
    TRACK_WIDTH_M = 0.092
    # The X2 sits over the midpoint of the axles (cad/frame.py: the LiDAR head at X=0, Y=0)
    LIDAR_X_M = 0.0
    LIDAR_Y_M = 0.0
    # Robot angle (counterclockwise from forward) of the X2's 0 degrees. Check it when the LiDAR
    # arrives: put an object straight ahead and see where it lands in the map.
    LIDAR_ANGLE_OFFSET_DEG = 0.0
    # Samples outside this range are not used (the X2 measures 0.12 to 8 m)
    LIDAR_MIN_M = 0.12
    LIDAR_MAX_M = 6.0

    # The limits the firmware applies (firmware/c-slam-bridge/main/bridge_config.h)
    MAX_SPEED = 0.30
    MAX_TURN_RATE = 2.5
  end
end
