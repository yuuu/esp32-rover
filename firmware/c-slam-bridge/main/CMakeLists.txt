idf_component_register(
    SRCS "main.c" "wifi.c" "link.c" "lidar.c"
    INCLUDE_DIRS "."
    REQUIRES rover esp_wifi esp_netif esp_event nvs_flash esp_driver_uart lwip esp_timer
)
