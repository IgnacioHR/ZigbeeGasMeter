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
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "zcl/esp_zigbee_zcl_metering.h"

#include "esp_zb_gas_meter.h"
#include "esp_zb_gas_meter_zigbee.h"
#include "esp_zb_gas_meter_adc.h"
#include "esp_zb_gas_meter_adc_zigbee.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

uint8_t battery_voltage = 0;
uint8_t battery_percentage = 0;
uint32_t battery_alarm_state = 0;
uint8_t battery_voltage_min = 70;
uint8_t battery_voltage_th1 = 72;

struct timeval last_battery_measurement_time;

adc_channel_t channel = ADC_CHANNEL_2; // ADC_CHANNEL_3 is a better option, but the current board was using 0 so let's change it now GPIO 3
adc_continuous_handle_t handle = NULL;

esp_err_t bat_adc_calibration_init(const adc_unit_t unit, const adc_channel_t channel, const adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;

    ESP_LOGD(TAG, "calibration scheme version is Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return ret;
}

esp_err_t bat_adc_calibration_deinit(const adc_cali_handle_t handle)
{
    ESP_LOGD(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_RETURN_ON_ERROR(adc_cali_delete_scheme_curve_fitting(handle), TAG, "Failed to delete ADC calibration curve");
    return ESP_OK;
}

bool IRAM_ATTR bat_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskGenericNotifyGiveFromISR(adc_task_handle, 1, &mustYield);

    return (mustYield == pdTRUE);
}

void bat_continuous_adc_init(const adc_channel_t channel, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = BAT_BUFFER_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = channel & 0x7;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

// adc task. Obtains battery voltage from adc conversion and formula.
// The task is deleted when the process finish
void adc_task(void *arg)
{
    ESP_LOGI(TAG, "ADC Task Init...");
    while (true) {
        ulTaskGenericNotifyTake(0, pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Starting battery measures");
        adc_cali_handle_t adc1_cali_chan0_handle = NULL;

        gpio_set_level(BAT_MON_ENABLE, 1);
        vTaskDelay(pdMS_TO_TICKS(5000));
        bat_continuous_adc_init(channel, &handle);
        adc_continuous_evt_cbs_t cbs = {
            .on_conv_done = bat_conv_done_cb,
        };
        ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
        ESP_ERROR_CHECK(adc_continuous_start(handle));
        ESP_ERROR_CHECK(bat_adc_calibration_init(ADC_UNIT_1, channel, ADC_ATTEN_DB_12, &adc1_cali_chan0_handle));

        uint8_t result[BAT_BUFFER_READ_LEN] = {0};
        memset(result, 0xcc, BAT_BUFFER_READ_LEN);

        ulTaskGenericNotifyTake(1, pdTRUE, portMAX_DELAY);
        gpio_set_level(BAT_MON_ENABLE, 0);

        uint32_t ret_num = 0;
        esp_err_t ret = adc_continuous_read(handle, result, BAT_BUFFER_READ_LEN, &ret_num, 0);
        if (ret == ESP_OK) {
            uint64_t total = 0;
            int16_t values = 0;
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = p->type2.channel;
                /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
                    uint32_t data = p->type2.data;
                    total += data;
                    values++;
                }
            }
            if (values > 0) {
                uint32_t average = total / values;
                int voltage;
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, average, &voltage));
                // convert to 2s lipo ranges and mult by 10
                float bat_voltage = ((float)(3.6084)*voltage + (float)0.062)/(float)(100.0); 
                battery_voltage = (uint8_t)(bat_voltage+(float)0.5);
                battery_percentage = (uint8_t)((bat_voltage-(float)(70.0))*(float)(13.6433)) + 7;
                if (battery_percentage > 200)
                    battery_percentage = 200;
                xEventGroupSetBits(report_event_group_handle, BATTERY_REPORT);

                if (battery_voltage < battery_voltage_th1) {
                    battery_alarm_state |= (1 << 1); // BatteryVoltageThreshold1 or BatteryPercentageThreshold1 reached for Battery Source 1
                } else {
                    battery_alarm_state &= ~(1 << 1);
                }
                if (battery_voltage < battery_voltage_min) {
                    battery_alarm_state |= (1 << 0); // BatteryVoltageMinThreshold or BatteryPercentageMinThreshold reached for Battery Source 1
                } else {
                    battery_alarm_state &= ~(1 << 0);
                }
                if (battery_alarm_state != 0 && (device_status & ESP_ZB_ZCL_METERING_GAS_LOW_BATTERY) == 0) {
                    device_status |= ESP_ZB_ZCL_METERING_GAS_LOW_BATTERY;
                    xEventGroupSetBits(report_event_group_handle, STATUS_REPORT);
                } else if (battery_alarm_state == 0 && (device_status & ESP_ZB_ZCL_METERING_GAS_LOW_BATTERY) != 0) {
                    device_status &= ~ESP_ZB_ZCL_METERING_GAS_LOW_BATTERY;
                    xEventGroupSetBits(report_event_group_handle, STATUS_REPORT);
                }

                ESP_LOGI(TAG, "Raw: %"PRIu32" Calibrated: %"PRId16"mV Bat Voltage: %1.2fv ZB Voltage: %d ZB Percentage: %d Alarm 0x%lx", 
                    average, voltage, bat_voltage/(float)(10.0), battery_voltage, battery_percentage, battery_alarm_state);
                gettimeofday(&last_battery_measurement_time, NULL);
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "ADC Task ESP_ERR_TIMEOUT");
            //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
            // TODO: indicate in status that battery can't be read
        }
        ESP_ERROR_CHECK(bat_adc_calibration_deinit(adc1_cali_chan0_handle));
        ESP_ERROR_CHECK(adc_continuous_stop(handle));
        ESP_ERROR_CHECK(adc_continuous_deinit(handle));
        ESP_LOGI(TAG, "Finishing battery measures");
    }
}
