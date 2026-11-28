# Rover hardware shared by the C projects under firmware/ (motors, encoders, odometry, wheel
# speed control, BOOT button). Projects add it with EXTRA_COMPONENT_DIRS ../components.
idf_component_register(
    SRCS "motor.c" "encoder.c" "odometry.c" "drive.c" "button.c" "check.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio
    PRIV_REQUIRES esp_driver_ledc esp_driver_pcnt
)
