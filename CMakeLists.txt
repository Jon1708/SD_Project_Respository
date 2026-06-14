idf_component_register(SRCS "main.c" "ble.c" "temp.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver esp_driver_ledc esp_driver_gpio bt nvs_flash esp_wifi esp_event esp_netif esp_http_server esp_timer esp_adc fatfs)
