idf_component_register(
        SRCS "esp_zb_gas_meter_zigbee.c" "esp_zb_gas_ota.c" "esp_zb_gas_meter.c" "esp_zb_gas_meter_adc.c" "esp_zb_gas_meter_zigbee.c" "esp_zb_gas_ota.c" "esp_zb_gas_led.c"
        INCLUDE_DIRS "."
        REQUIRES esp-zigbee-lib nvs_flash esp_timer esp_driver_gpio esp_adc app_update
)
