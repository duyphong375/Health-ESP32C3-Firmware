idf_component_register(
    SRCS "app_rainmaker.c" "main.c" "max30102.c" "ssd1306.c" "pulse.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt esp_driver_i2c esp_driver_gpio esp_timer nvs_flash espressif__esp_rainmaker espressif__rmaker_app_network
)
