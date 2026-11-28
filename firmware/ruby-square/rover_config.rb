# Hardware of the rover, the Ruby twin of
# firmware/components/rover/include/rover_config.h: keep the two the same.

# ================================================================ Pins
# Wiring of pcb/main (XIAO ESP32-S3 Dn -> GPIO).
# The TC78H653FTG motor driver runs with MODE=GND (IN/IN mode) and STBY tied to 3V3
PIN_MOTOR_A_IN1 = 1 # D0 -> IN1 -> J1 pin1 (OUT1)
PIN_MOTOR_A_IN2 = 2 # D1 -> IN2 -> J1 pin2 (OUT2)
PIN_MOTOR_B_IN1 = 3 # D2 -> IN3 -> J3 pin1 (OUT3)
PIN_MOTOR_B_IN2 = 4 # D3 -> IN4 -> J3 pin2 (OUT4)
# Encoder A/B phases. Power is J1/J3 pin3 (3V3), GND is J2/J4 pin3
PIN_ENC_A_A = 5 # D4 -> J2 pin1
PIN_ENC_A_B = 6 # D5 -> J2 pin2
PIN_ENC_B_A = 7 # D8 -> J4 pin1
PIN_ENC_B_B = 8 # D9 -> J4 pin2
PIN_START_BUTTON = 0 # XIAO BOOT button (LOW when pressed)
# YDLIDAR X2 (J5, PH 5P). The X2 only transmits, at 115200 baud 8N1
PIN_LIDAR_RX = 44    # D7 <- J5 pin2 (LiDAR Tx)
PIN_LIDAR_MCTR = 9   # D10 -> J5 pin5 (M_CTR; left as an input, the X2's default speed)

# ================================================================ Left/right mapping and direction
# Check these with CHECK_MODE = true in square_config.rb. The values below are verified on the robot
LEFT_IS_A = false     # true: left is motor A (J1) / encoder A (J2), false: left is B (J3/J4)
MOTOR_L_INVERT = true # true if a positive duty turns the wheel backward
MOTOR_R_INVERT = false # The motors face each other, so one side is reversed
ENC_L_INVERT = false  # true if the count decreases when the wheel turns forward
ENC_R_INVERT = true

# ================================================================ Robot
WHEEL_DIAMETER_M = 0.060 # FS90R-W tire, 60mm
TRACK_WIDTH_M = 0.092    # Wheel center planes at Y=+-46mm
# MG12A06E-060310-50: encoder 7PPR x gear ratio 50 x 4 (quadrature)
ENC_COUNTS_PER_REV = 7 * 50 * 4

# ================================================================ Control
CONTROL_PERIOD_MS = 10
# Approximate wheel speed at duty 1.0 (for feedforward; tune on the robot)
WHEEL_SPEED_AT_FULL_DUTY = 0.60 # [m/s]
WHEEL_KP = 1.5                  # speed error [m/s] -> duty
WHEEL_KI = 6.0
WHEEL_SPEED_LPF = 0.3           # low-pass filter on measured speed (1 disables it)
