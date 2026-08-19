idf_component_register(
    SRCS "base_main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_wifi esp_event esp_netif console driver esp_timer
)
