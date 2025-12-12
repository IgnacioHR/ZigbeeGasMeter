/*
 * Zigbee Gas Meter - An open-source Zigbee gas meter project.
 * Copyright (c) 2025 Ignacio Hernández-Ros.
 *
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. To view a copy of this license, visit
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 *
 * You may use, modify, and share this work for personal and non-commercial purposes, as long
 * as you credit the original author(s) and share any derivatives under the same license.
 */
#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "esp_zb_gas_version.h"

#define OTA_UPGRADE_MAX_DATA_SIZE       223
#define OTA_ELEMENT_HEADER_LEN          6

extern uint16_t zigbee_zcl_version;
extern uint8_t  hw_version;
extern uint16_t zigbee_stack_version;

esp_err_t check_boot_partition_change();
esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message);
esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message);