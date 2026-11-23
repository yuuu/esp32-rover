idf_component_register(
    SRCS "main.c" "motor.c" "encoder.c" "odometry.c" "drive.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc esp_driver_pcnt esp_driver_gpio
)
