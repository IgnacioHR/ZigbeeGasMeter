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
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "sys/time.h"
#include "esp_timer.h"

#include "esp_zb_gas_meter.h"
#include "esp_zb_gas_meter_adc.h"
#include "esp_zb_gas_meter_zigbee.h"
#include "esp_zb_gas_ota.h"

/* Experimental, check if we sleepy device can help the design */
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#endif

/* Hardware configuration */

// input - pin with gas magnetic sensor contact
#define PULSE_PIN GPIO_NUM_1

// input - pin for the main button
#define MAIN_BTN GPIO_NUM_0

#define EXTERNAL_ANTENNA

// amount of time to ignore a digital input pin interrupt repetition
#define DEBOUNCE_TIMEOUT 20 /* milliseconds */

/* Human interaction and device configuration */

// How long the MAIN BUTTON must be pressed to consider a LONG PRESS
#define LONG_PRESS_TIMEOUT 5 // seconds

#define NVS_NAMESPACE "gas_monitor"
#define NVS_KEY "counter"

#define TIME_TO_RESET_INSTANTANEOUS_D UINT32_C(TIME_TO_SLEEP_ZIGBEE_ON - 2000)

const char *TAG = "MICASA_GAS_METER";

// Last time a pulse was received
RTC_DATA_ATTR struct timeval last_pulse_time;

// When the main button is pressed a one time task is started
// to detect a long press without having to wait until
// the user releases the button
bool started_from_deep_sleep = false;

// true while leaving the network to prevent sending data to coordinator
bool leaving_network = false;

// sleep
RTC_DATA_ATTR struct timeval sleep_enter_time;

// Non volatile memory handle
nvs_handle_t my_nvs_handle;

TaskHandle_t adc_task_handle = NULL;
TaskHandle_t zigbee_task_handle = NULL;
TaskHandle_t deep_sleep_task_handle = NULL;
TaskHandle_t save_counter_task_handle = NULL;
TaskHandle_t btn_press_task_handle = NULL;
TaskHandle_t btn_release_task_handle = NULL;
QueueHandle_t deep_sleep_queue_handle = NULL;
TimerHandle_t reset_instantaneous_demand_timer = NULL;
TimerHandle_t deep_sleep_timer = NULL;
EventGroupHandle_t report_event_group_handle = NULL;
EventGroupHandle_t main_event_group_handle = NULL;

// Intelligent button gesture detection timers
TimerHandle_t t0_2s_since_press = NULL;
TimerHandle_t t4s_since_press = NULL;
TimerHandle_t t0_2s_since_release = NULL;
TimerHandle_t t0_5s_since_release = NULL;
TimerHandle_t t5s_since_release = NULL;

TaskHandle_t btn_task_handle = NULL;

#define CLICK_PRESS_TIME_MS      200    // MUST BE BIGGER THAN 150
#define CLICK_HOLD_TIME_MS       4000
#define CLICK_RELEASE_TIME_MS    200
#define BUTTON_EVENT_DELAY_MS    500
#define BUTTON_RESET_STATE_MS    5000

typedef enum ButtonState_e {
    NONE = 0,
    PRESS,
    RELEASE,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    UNKNOWN_CLICK,
    HOLD
} button_state_t;

button_state_t button_state = NONE;

// bool is_none = true;
// bool is_press = false;
// bool is_release = false;
// bool is_single_click = false;
// bool is_double_click = false;
// bool is_unknown_click = false;
// bool is_hold = false;

static portMUX_TYPE counter_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Load counter value from NVS
esp_err_t gm_counter_load_nvs()
{
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_nvs_handle);
    if (err == ESP_OK)
    {
        uint64_t saved_count = 0;
        err = nvs_get_u64(my_nvs_handle, NVS_KEY, &saved_count);
        if (err == ESP_OK)
        {
            current_summation_delivered.low = saved_count & 0x00000000FFFFFFFF;
            current_summation_delivered.high = (saved_count & 0x0000FFFF00000000) >> 32;
            current_summation_delivered_float = (float)saved_count;
            ESP_LOGI(TAG, "Counter loaded: low=%lu high=%u", current_summation_delivered.low, current_summation_delivered.high);
        }
        else
        {
            ESP_LOGI(TAG, "Counter not found in memory so starting from 0");
            err = ESP_OK;
        }
    }
    else
    {
        device_extended_status |= ESP_ZB_ZCL_METERING_NV_MEMORY_ERROR;
        device_status |= ESP_ZB_ZCL_METERING_GAS_CHECK_METER;
        xEventGroupSetBits(report_event_group_handle, STATUS_REPORT | EXTENDED_STATUS_REPORT);

        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
    }
    return err;
}

// Task to save the counter to NVS when value changes
void save_counter_task(void *arg)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t to_save_count = current_summation_delivered.high;
        to_save_count <<= 32;
        to_save_count |= current_summation_delivered.low;
        esp_err_t err = nvs_set_u64(my_nvs_handle, NVS_KEY, to_save_count);
        if (err == ESP_OK)
        {
            nvs_commit(my_nvs_handle);
            ESP_LOGI(TAG, "Counter stored: low=%lu high=%d", current_summation_delivered.low, current_summation_delivered.high);
            if (deep_sleep_task_handle != NULL)
            {
                TickType_t deep_sleep_time = dm_deep_sleep_time_ms();
                if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                    ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
            }
        }
        else
        {
            device_extended_status |= ESP_ZB_ZCL_METERING_NV_MEMORY_ERROR;
            device_status |= ESP_ZB_ZCL_METERING_GAS_CHECK_METER;
            xEventGroupSetBits(report_event_group_handle, STATUS_REPORT | EXTENDED_STATUS_REPORT);
            ESP_LOGE(TAG, "Error saving NVS: %s", esp_err_to_name(err));
        }
    }
}

// Set counter value and save
void gm_counter_set(const esp_zb_uint48_t *new_value)
{
    current_summation_delivered.low = new_value->low;
    current_summation_delivered.high = new_value->high;
    uint64_t current_summation_64 = current_summation_delivered.high;
    current_summation_64 <<= 32;
    current_summation_64 |= current_summation_delivered.low;
    current_summation_delivered_float = (float)current_summation_64;
    xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
    xTaskNotifyGive(save_counter_task_handle);
}

// Reset counter value to 0 and save
void gm_counter_reset()
{
    esp_zb_uint48_t zero = {
        .low = 0,
        .high = 0};
    gm_counter_set(&zero);
}

// Helper function to return the time in milliseconds since now and the other timeval
// received as a parameter
int time_diff_ms(const struct timeval *other)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int time_diff = (now.tv_sec - other->tv_sec) * 1000 + (now.tv_usec - other->tv_usec) / 1000;
    return time_diff;
}

// Check conditions to enable radio.
// There are 2 possible conditions:
// - time pass since last report
// - counter increased 10 times
void check_shall_enable_radio()
{
    if (zigbee_task_handle == NULL)
    {
        bool enable_radio =
            (last_report_sent_time.tv_sec == 0 && last_report_sent_time.tv_usec == 0) ||
            (time_diff_ms(&last_report_sent_time) / 1000 >= MUST_SYNC_MINIMUM_TIME);
        if (!enable_radio && last_summation_sent > 0)
        {
            uint64_t current_summation_64 = current_summation_delivered.high;
            current_summation_64 <<= 32;
            current_summation_64 |= current_summation_delivered.low;
            enable_radio = current_summation_64 - last_summation_sent >= COUNTER_REPORT_DIFF;
        }
        if (enable_radio)
        {
            xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
            xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
        }
    }
}

// Computes if it is needed to measure battery voltage
void check_shall_measure_battery()
{
    bool measure_battery =
        (last_battery_measurement_time.tv_sec == 0 && last_battery_measurement_time.tv_usec == 0) ||
        (time_diff_ms(&last_battery_measurement_time) / 1000 >= MUST_SYNC_MINIMUM_TIME);
    if (measure_battery)
    {
        // battery voltage is measured with zigbee radio
        // turned on
        xEventGroupSetBits(main_event_group_handle, SHALL_MEASURE_BATTERY);
        xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
    }
}

// After two consecutive values of current_summation_delivered this method
// computes the instantaneous_demand
// NOTE: there is one special task to set instantaneous demand to 0 when no
// values arrive
void gm_compute_instantaneous_demand(int time_diff_ms, bool fromISR)
{
    float time_diff_h = time_diff_ms / (1000.0 * 3600.0); // Convert time to hours/100
    int32_t _instantaneous_demand;
    if (time_diff_h > 0)
    {
        _instantaneous_demand = (int32_t)((1 / time_diff_h) + 0.5); // compute flow in m³/h
    }
    else
    {
        _instantaneous_demand = 0;
    }
    // check for a real change in the value
    esp_zb_int24_t new_instantaneous_demand;
    new_instantaneous_demand.low = _instantaneous_demand & 0x0000FFFF;
    new_instantaneous_demand.high = (_instantaneous_demand & 0x00FF0000) >> 16;
    if (new_instantaneous_demand.low != instantaneous_demand.low || new_instantaneous_demand.high != instantaneous_demand.high)
    {
        instantaneous_demand.low = new_instantaneous_demand.low;
        instantaneous_demand.high = new_instantaneous_demand.high;
        if (fromISR)
        {
            BaseType_t mustYield = pdFALSE;
            xEventGroupSetBitsFromISR(report_event_group_handle, INSTANTANEOUS_DEMAND_REPORT, &mustYield);
            portYIELD_FROM_ISR(mustYield);
            mustYield = pdFALSE;
            if (xTimerStartFromISR(reset_instantaneous_demand_timer, &mustYield) != pdPASS)
            {
                ESP_LOGE(TAG, "Can't reset instantaneous demand timer");
            }
            portYIELD_FROM_ISR(mustYield);
        }
        else
        {
            xEventGroupSetBits(report_event_group_handle, INSTANTANEOUS_DEMAND_REPORT);
            if (xTimerStart(reset_instantaneous_demand_timer, pdMS_TO_TICKS(100)) != pdPASS)
            {
                ESP_LOGE(TAG, "Can't reset instantaneous demand timer");
            }
        }
    }
}

// Adds one to current_summation_delivered and nothing else
void gm_counter_increment(const struct timeval *now, bool fromISR)
{
    if (fromISR)
    {
        taskENTER_CRITICAL_ISR(&counter_spinlock);
    }
    else
    {
        taskENTER_CRITICAL(&counter_spinlock);
    }
    bool debounce = false;
    bool compute_instantaneous_demand = false;
    int time_diff_ms = 0;
    if (last_pulse_time.tv_sec != 0 || last_pulse_time.tv_usec != 0)
    {
        time_diff_ms = (now->tv_sec - last_pulse_time.tv_sec) * 1000 +
                       (now->tv_usec - last_pulse_time.tv_usec) / 1000;
        // debounce
        debounce = time_diff_ms > 0 && time_diff_ms < 500;
        compute_instantaneous_demand = time_diff_ms > 0 && !debounce;
    }
    if (!debounce)
    {
        last_pulse_time.tv_usec = now->tv_usec;
        last_pulse_time.tv_sec = now->tv_sec;
    }
    if (fromISR)
    {
        taskEXIT_CRITICAL_ISR(&counter_spinlock);
    }
    else
    {
        taskEXIT_CRITICAL(&counter_spinlock);
    }
    if (debounce)
        return;

    current_summation_delivered.low += 1; // Adds up 1 cent of m³
    current_summation_delivered_float += 1;
    if (current_summation_delivered.low == 0)
    {
        current_summation_delivered.high += 1;
    }
    if (fromISR)
    {
        BaseType_t mustYield = pdFALSE;
        xEventGroupSetBitsFromISR(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT, &mustYield);
        portYIELD_FROM_ISR(mustYield);
        mustYield = pdFALSE;
        vTaskNotifyGiveFromISR(save_counter_task_handle, &mustYield);
        portYIELD_FROM_ISR(mustYield);
    }
    else
    {
        xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
        xTaskNotifyGive(save_counter_task_handle);
    }
    if (compute_instantaneous_demand)
        gm_compute_instantaneous_demand(time_diff_ms, fromISR);
}

// function called when the device leaves the zigee network
void leave_callback(esp_zb_zdp_status_t zdo_status, void *args)
{
    leaving_network = false;
    ESP_LOGI(TAG, "Leave status 0x%x", zdo_status);
}

// Compute how long to wait for sleep depending on device conditions
TickType_t dm_deep_sleep_time_ms()
{
    const TickType_t before_deep_sleep_time_ms = (zigbee_task_handle != NULL ? TIME_TO_SLEEP_ZIGBEE_ON : TIME_TO_SLEEP_ZIGBEE_OFF);
    ESP_LOGD(TAG, "Start one-shot timer for %ldms to enter the deep sleep", before_deep_sleep_time_ms);
    return pdMS_TO_TICKS(before_deep_sleep_time_ms);
}

// task to govern the deep sleep timeout
void deep_sleep_controller_task(void *arg)
{
    while (true)
    {
        TickType_t new_timer_value;
        if (xQueueReceive(deep_sleep_queue_handle, &new_timer_value, portMAX_DELAY) == pdTRUE)
        {
            if (new_timer_value == portMAX_DELAY)
            {
                if (xTimerStop(deep_sleep_timer, pdMS_TO_TICKS(100)) != pdPASS)
                {
                    ESP_LOGE(TAG, "Can't stop deep sleep timer");
                }
            }
            else
            {
                if (xTimerChangePeriod(deep_sleep_timer, new_timer_value, pdMS_TO_TICKS(100)) != pdPASS)
                {
                    ESP_LOGE(TAG, "Can't change deep sleep timer time");
                }
            }
        }
    }
}

// task to manage main button press events
void btn_press_task(void *arg)
{
    ESP_LOGI(TAG, "Main button press task enter");
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Main button device pressed");
        if (button_state == NONE) {
            button_state = PRESS;
            xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
        }
        if (xTimerIsTimerActive(t0_5s_since_release) != pdFALSE) {
            xTimerStop(t0_5s_since_release, pdMS_TO_TICKS(100));
        }
        if (xTimerIsTimerActive(t5s_since_release) != pdFALSE) {
            xTimerStop(t5s_since_release, pdMS_TO_TICKS(100));
        }
        int click_press_time_adjusted_msec = CLICK_PRESS_TIME_MS;
        int hold_time_adjusted_msec = CLICK_HOLD_TIME_MS;
        if (started_from_deep_sleep)
        {
            click_press_time_adjusted_msec -= 150; // measured time for the device to start
            hold_time_adjusted_msec -= 150;
        }
        ESP_LOGI(TAG, "Start long press timer for %dms", click_press_time_adjusted_msec);
        if (xTimerChangePeriod(t0_2s_since_press, pdMS_TO_TICKS(click_press_time_adjusted_msec), pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGE(TAG, "Can't set press timer time to %dms", click_press_time_adjusted_msec);
            return;
        };
        if (xTimerStart(t0_2s_since_press, pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGE(TAG, "Can't start press timer");
        }
        if (xTimerIsTimerActive(t0_2s_since_release) == pdFALSE) {
            ESP_LOGI(TAG, "Start hold timer for %dms", hold_time_adjusted_msec);
            if (xTimerChangePeriod(t4s_since_press, pdMS_TO_TICKS(hold_time_adjusted_msec), pdMS_TO_TICKS(100)) != pdPASS)
            {
                ESP_LOGE(TAG, "Can't set hold timer time to %dms", hold_time_adjusted_msec);
                return;
            };
            if (xTimerStart(t4s_since_press, pdMS_TO_TICKS(100)) != pdPASS)
            {
                ESP_LOGE(TAG, "Can't start hold timer");
            }
        }
    }
}

void btn_release_task(void *arg)
{
    ESP_LOGI(TAG, "Main button release task enter");
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Main button device released");
        if (button_state == PRESS) {
            button_state = RELEASE;
            xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
        }
        bool in_two_tenth_of_s_since_press = xTimerIsTimerActive(t0_2s_since_press) != pdFALSE;
        if (in_two_tenth_of_s_since_press) {
            xTimerStop(t0_2s_since_press, pdMS_TO_TICKS(100));
            if (button_state == SINGLE_CLICK) {
                button_state = DOUBLE_CLICK;
            }
            if (button_state != DOUBLE_CLICK) {
                button_state = DOUBLE_CLICK;
            }
        }
        if (xTimerIsTimerActive(t4s_since_press) != pdFALSE) {
            xTimerStop(t4s_since_press, pdMS_TO_TICKS(100));
        }
        if (xTimerStart(t0_2s_since_release, pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGE(TAG, "Can't start 0.2s timer after release");
        }
        if (xTimerStart(t0_5s_since_release, pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGE(TAG, "Can't start 0.5s timer after release");
        }
        if (xTimerStart(t5s_since_release, pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGE(TAG, "Can't start 5s timer after release");
        }
    }
}

void btn_task(void *arg)
{
    ESP_LOGI(TAG, "Main button action task");
    while (true)
    {
        button_state_t state;
        xTaskNotifyWait(0x00, 0xFF, &state, portMAX_DELAY);
        switch (state) {
            case PRESS:
                ESP_LOGI(TAG, "Button press");
                xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
                if (deep_sleep_task_handle != NULL)
                {
                    TickType_t deep_sleep_time = pdMS_TO_TICKS(TIME_TO_SLEEP_ZIGBEE_ON);
                    if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                        ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
                }
                break;
            case RELEASE:
                ESP_LOGI(TAG, "Button release");
                break;
            case SINGLE_CLICK:
                ESP_LOGI(TAG, "Single click detected");
                break;
            case DOUBLE_CLICK:
                ESP_LOGI(TAG, "Double click detected");
                break;
            case UNKNOWN_CLICK:
                ESP_LOGI(TAG, "Unknown click detected");
                break;
            case HOLD:
                ESP_LOGI(TAG, "Hold detected");
                break;
            case NONE:
                ESP_LOGI(TAG, "Button state reset");
                break;
            default:
                ESP_LOGI(TAG, "Unknown button state");
                break;
        }
    }
}

// set the instantaneous demand to 0 and indicates to report this value
void reset_instantaneous_demand_cb(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Reset instantaneous demand");
    if (instantaneous_demand.low != 0 || instantaneous_demand.high != 0)
    {
        instantaneous_demand.low = 0;
        instantaneous_demand.high = 0;
        xEventGroupSetBits(report_event_group_handle, INSTANTANEOUS_DEMAND_REPORT);
    }
}

// device main loop activities, note
// zigbee activities are not part of the
// main loop, just the critical activities
// to maintain the counter
void gm_main_loop_task(void *arg)
{
    ESP_LOGI(TAG, "Main Loop Task...");
    ESP_ERROR_CHECK(check_boot_partition_change());
    xEventGroupSetBits(main_event_group_handle, SHALL_START_DEEP_SLEEP);
    while (true)
    {
        EventBits_t uxBits = xEventGroupWaitBits(
            main_event_group_handle,
            SHALL_ENABLE_ZIGBEE | SHALL_MEASURE_BATTERY | SHALL_DISABLE_ZIGBEE | SHALL_START_DEEP_SLEEP | SHALL_STOP_DEEP_SLEEP,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(250));
        if (uxBits != 0)
        {
            if ((uxBits & SHALL_MEASURE_BATTERY) != 0)
            {
                ESP_LOGI(TAG, "Measuring battery capacity");
                xTaskNotifyGiveIndexed(adc_task_handle, 0);
            }
            if (((uxBits & SHALL_ENABLE_ZIGBEE) != 0) && zigbee_task_handle == NULL)
            {
#ifdef EXTERNAL_ANTENNA
                uint64_t antenna_switch_pin = 1ULL << GPIO_NUM_3;
                uint64_t antenna_mode_pin = 1ULL << GPIO_NUM_14;
                gpio_config_t anthena_conf = {
                    .intr_type = GPIO_INTR_DISABLE,
                    .mode = GPIO_MODE_OUTPUT,
                    .pin_bit_mask = antenna_switch_pin | antenna_mode_pin,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .pull_up_en = GPIO_PULLUP_DISABLE
                };
                ESP_ERROR_CHECK(gpio_config(&anthena_conf));

                ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_3, 0));
                vTaskDelay(pdMS_TO_TICKS(100));
                ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_14, 1));
#endif
                /* Start Zigbee stack task */
                TickType_t deep_sleep_time = pdMS_TO_TICKS(TIME_TO_SLEEP_ZIGBEE_ON);
                if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                    ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
                if (xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 10, &zigbee_task_handle) != pdTRUE)
                    ESP_LOGE(TAG, "can't create zigbee task");
                ESP_LOGI(TAG, "Turning on Zigbee radio");
            }
            if (((uxBits & SHALL_DISABLE_ZIGBEE) != 0) && zigbee_task_handle != NULL)
            {
                ESP_LOGD(TAG, "shall_disable_radio");
                // TODO: see how the ADC task terminates.
                // depends on https://github.com/espressif/esp-zigbee-sdk/issues/561
                // at the end it is required to set:
                //  zigbee_enabled = false;
                if (deep_sleep_task_handle != NULL)
                {
                    TickType_t deep_sleep_time = portMAX_DELAY;
                    if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                        ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
                }
                zigbee_task_handle = NULL;
            }
            if (((uxBits & SHALL_START_DEEP_SLEEP) != 0) && deep_sleep_task_handle == NULL)
            {
                ESP_LOGI(TAG, "Starting deep sleep task");
                ESP_ERROR_CHECK(xTaskCreate(deep_sleep_controller_task, "deep_sleep", 2048, NULL, 20, &deep_sleep_task_handle) != pdPASS);
                if (deep_sleep_task_handle != NULL)
                {
                    int level = gpio_get_level(PULSE_PIN);
                    TickType_t deep_sleep_time = level == 1 ? portMAX_DELAY : dm_deep_sleep_time_ms();
                    if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                        ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
                }
            }
            if (((uxBits & SHALL_STOP_DEEP_SLEEP) != 0) && deep_sleep_task_handle != NULL)
            {
                ESP_LOGI(TAG, "Stoping deep sleep task");
                vTaskDelete(deep_sleep_task_handle);
                deep_sleep_task_handle = NULL;
            }
        }
        else
        {
            check_shall_measure_battery();
            check_shall_enable_radio();
        }
    }
}

// callback to start deep sleep
void enter_deep_sleep_cb(TimerHandle_t xTimer)
{
    /* Enter deep sleep */
    if (deep_sleep_task_handle == NULL)
    {
        ESP_LOGI(TAG, "Enter deep sleep cancelled");
        return;
    }
    ESP_LOGI(TAG, "Enter deep sleep");
    gettimeofday(&sleep_enter_time, NULL);
    esp_deep_sleep_start();
}

void t0_2s_since_press_cb(TimerHandle_t xTimer)
{
    if (button_state == SINGLE_CLICK || button_state == DOUBLE_CLICK) {
        button_state = UNKNOWN_CLICK;
    }
}

void t4s_since_press_cb(TimerHandle_t xTimer)
{
    button_state = HOLD;
    xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
}

void t0_2s_since_release_cb(TimerHandle_t xTimer)
{
}

void t0_5s_since_release_cb(TimerHandle_t xTimer)
{
    xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
}

void t20s_since_release_cb(TimerHandle_t xTimer)
{
    button_state = NONE;
    xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
}

// configure deep sleep for the gas meter
esp_err_t gm_deep_sleep_init()
{
    const int gpio_pulse_pin = PULSE_PIN;
    const uint64_t gpio_pulse_pin_mask = 1ULL << gpio_pulse_pin;
    const int gpio_mainbtn_pin = MAIN_BTN;
    const uint64_t gpio_mainbtn_pin_mask = 1ULL << gpio_mainbtn_pin;

    // wake-up reason:
    struct timeval gpio_time;
    gettimeofday(&gpio_time, NULL);
    int sleep_time_ms = (gpio_time.tv_sec - sleep_enter_time.tv_sec) * 1000 +
                        (gpio_time.tv_usec - sleep_enter_time.tv_usec) / 1000;
    esp_sleep_wakeup_cause_t wake_up_cause = esp_sleep_get_wakeup_cause();
    switch (wake_up_cause)
    {
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep and boot: %dms", sleep_time_ms);
        xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
        xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
        break;
    }
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        bool resolved = false;
        uint64_t ext1mask = esp_sleep_get_ext1_wakeup_status();
        if ((ext1mask & gpio_mainbtn_pin_mask) == gpio_mainbtn_pin_mask)
        { // wakeup from MAIN_BTN
            ESP_LOGI(TAG, "Wake up from MAIN BUTTON. Time spent in deep sleep and boot: %dms", sleep_time_ms);
            started_from_deep_sleep = true;
            xEventGroupSetBits(main_event_group_handle, SHALL_MEASURE_BATTERY);
            xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
            xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
            int level = gpio_get_level(MAIN_BTN);
            if (level == 0)
            {
                xTaskNotifyGive(btn_release_task_handle);
            }
            else
            {
                xTaskNotifyGive(btn_press_task_handle);
            }
            resolved = true;
        }
        if ((ext1mask & gpio_pulse_pin_mask) == gpio_pulse_pin_mask)
        { // wakeup from PULSE_PIN
            ESP_LOGI(TAG, "Wake up from GAS PULSE. Time spent in deep sleep and boot: %dms", sleep_time_ms);
            // check_gpio_time = true;
            int level = gpio_get_level(PULSE_PIN);
            // if PULSE_PIN is low AND check_gpio_time is true we
            // miss the interrupt so count it now
            if (level == 0)
            {
                gm_counter_increment(&gpio_time, false);
            }
            else
            {
                TickType_t deep_sleep_time = portMAX_DELAY;
                if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                    ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
            }
            resolved = true;
        }
        if (!resolved)
        {
            ESP_LOGI(TAG, "Wake up from unknown GPIO. Time spent in deep sleep and boot: %dms", sleep_time_ms);
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        ESP_LOGI(TAG, "Not a deep sleep reset");
        xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
        xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
        break;
    }
    check_shall_enable_radio();
    check_shall_measure_battery();

    /* Set the methods of how to wake up: */
    /* 1. RTC timer waking-up */
    int report_time_s = (gpio_time.tv_sec - last_report_sent_time.tv_sec) +
                        (gpio_time.tv_usec - last_report_sent_time.tv_usec) / 1000000;

    const uint64_t wakeup_time_sec = MUST_SYNC_MINIMUM_TIME - report_time_s;
    ESP_LOGD(TAG, "Enabling timer wakeup, %llds", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));

    /* PULSE_PIN and MAIN_BTN wake up on pull up */

    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(
        gpio_pulse_pin_mask | gpio_mainbtn_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH));
    
    esp_deep_sleep_disable_rom_logging();

    return ESP_OK;
}

// PULSE_PIN - GPIO Interruption handler
void IRAM_ATTR gpio_pulse_isr_handler(void *arg)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    if ((last_pulse_time.tv_sec != 0 || last_pulse_time.tv_usec != 0) && time_diff_ms(&last_pulse_time) <= DEBOUNCE_TIMEOUT)
    {
        return; // DEBOUNCE
    }

    // Increment current_summation_delivery
    // read pin to determine failing or raising edge
    int level = gpio_get_level(PULSE_PIN);
    if (level == 0)
    {
        // failing
        gm_counter_increment(&now, true);
    }
    else if (deep_sleep_task_handle != NULL)
    {
        // rising, the device while NOT in deep sleep mode. At this moment, if there
        // is no activity in 30 seconds, the device will try to enter deep sleep but
        // it won't success because the pin level is already high so it will wake up
        // inmediatelly. The solution at this moment is to stop the timer interrupt
        // until the failing edge is detected
        TickType_t deep_sleep_time = portMAX_DELAY;
        BaseType_t mustYield = pdFALSE;
        if (xQueueSendToFrontFromISR(deep_sleep_queue_handle, &deep_sleep_time, &mustYield) != pdTRUE)
            ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
        portYIELD_FROM_ISR(mustYield);
    }
}

// MAIN_BTN - GPIO Interruption handler
void IRAM_ATTR gpio_btn_isr_handler(void *arg)
{
    // Last time wakeup pin was received
    static int64_t last_main_btn_time = 0;

    int level = gpio_get_level(MAIN_BTN);
    int64_t current_time = esp_timer_get_time();
    int64_t time_diff_ms = (current_time - last_main_btn_time) / 1000;
    if (last_main_btn_time > 0 && time_diff_ms <= DEBOUNCE_TIMEOUT)
        return; // DEBOUNCE

    BaseType_t mustYield = pdFALSE;
    if (level == 1)
    {
        started_from_deep_sleep = false;
        vTaskNotifyGiveFromISR(btn_press_task_handle, &mustYield);
    }
    if (level == 0)
    {
        vTaskNotifyGiveFromISR(btn_release_task_handle, &mustYield);
    }

    last_main_btn_time = current_time;
    portYIELD_FROM_ISR(mustYield);
}

// Configure interrupt to catch pulses from magnetic sensor
// NOTE: When the device is in deep sleep internal pull-up and pull-down
// resistors aren't used so they are disabled.
// In order to save battery I decided to use pull-down because this is
// how the sensor should stay most of the time
esp_err_t gm_gpio_interrup_init()
{
    uint64_t pulse_pin = 1ULL << PULSE_PIN;
    //      __
    gpio_config_t io_conf_pulse = {                                // ____|  |_____
                                   .intr_type = GPIO_INTR_ANYEDGE, //     ^--^- Interrupt both edges
                                   .mode = GPIO_MODE_INPUT,        // Input pin
                                   .pin_bit_mask = pulse_pin,
                                   .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                   .pull_up_en = GPIO_PULLUP_DISABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf_pulse), TAG, "Can't config gpio for PULSE_PIN and MAIN_PIN pins");

    uint64_t mainbtn_pin = 1ULL << MAIN_BTN;
    //      __
    gpio_config_t io_conf_mainbtn = {                                // ____|  |_____
                                     .intr_type = GPIO_INTR_ANYEDGE, //     ^--^- Interrupt both edges
                                     .mode = GPIO_MODE_INPUT,        // Input pin
                                     .pin_bit_mask = mainbtn_pin,
                                     .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                     .pull_up_en = GPIO_PULLUP_DISABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf_mainbtn), TAG, "Can't config gpio for PULSE_PIN and MAIN_PIN pins");

    // Configure and register interrupt service
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_LOWMED), TAG, "Can't install isr service");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(PULSE_PIN, gpio_pulse_isr_handler, NULL), TAG, "Can't add PULSE_PIN interrupt handler");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(MAIN_BTN, gpio_btn_isr_handler, NULL), TAG, "Can't add MAIN_BTN interrupt handler");

    // Enable voltage measure enable output pin
    uint64_t bat_volt_enable_pin = 1ULL << BAT_MON_ENABLE;
    gpio_config_t voltage_enable_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = bat_volt_enable_pin,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&voltage_enable_conf), TAG, "Can't config gpio for BAT_MON_ENABLE pin");

    return ESP_OK;
}

// configure power save
esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    // TODO: explore why this causes a problem
    // When the device enters deep sleep after a 3s period
    // caused by the counter going up one tick
    // the device wakes up on RTC automatically and this
    // is not desired as it will turn on Zigbee radio
    // as it is defined in code

    // .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

esp_err_t report_reset_reason()
{
    esp_reset_reason_t reset_reason = esp_reset_reason();
    switch (reset_reason)
    {
    case ESP_RST_UNKNOWN: //!< Reset reason can not be determined
        ESP_LOGI(TAG, "Unknown reset reason");
        break;
    case ESP_RST_POWERON: //!< Reset due to power-on event
        ESP_LOGI(TAG, "Powerdown reset reason");
        break;
    case ESP_RST_EXT: //!< Reset by external pin (not applicable for ESP32)
        ESP_LOGI(TAG, "external pin reset reason");
        break;
    case ESP_RST_SW: //!< Software reset via esp_restart
        ESP_LOGI(TAG, "Software reset reason");
        break;
    case ESP_RST_PANIC: //!< Software reset due to exception/panic
        device_extended_status |= ESP_ZB_ZCL_METERING_PROGRAM_MEMORY_ERROR;
        ESP_LOGI(TAG, "Esception/panic reset reason");
        return ESP_FAIL;
    case ESP_RST_INT_WDT: //!< Reset (software or hardware) due to interrupt watchdog
        ESP_LOGI(TAG, "Reset (Software of Hardware) reset reason");
        break;
    case ESP_RST_TASK_WDT: //!< Reset due to task watchdog
        ESP_LOGI(TAG, "Task watchdog reset reason");
        device_extended_status |= ESP_ZB_ZCL_METERING_WATCHDOG_ERROR;
        return ESP_FAIL;
    case ESP_RST_WDT: //!< Reset due to other watchdogs
        ESP_LOGI(TAG, "Other watchdog reset reason");
        device_extended_status |= ESP_ZB_ZCL_METERING_WATCHDOG_ERROR;
        return ESP_FAIL;
    case ESP_RST_DEEPSLEEP: //!< Reset after exiting deep sleep mode
        ESP_LOGI(TAG, "After exiting deep sleep reset reason");
        break;
    case ESP_RST_BROWNOUT: //!< Brownout reset (software or hardware)
        ESP_LOGI(TAG, "Brownout reset reason");
        device_extended_status |= ESP_ZB_ZCL_METERING_BATTERY_FAILURE;
        return ESP_FAIL;
    case ESP_RST_SDIO: //!< Reset over SDIO
        ESP_LOGI(TAG, "SDIO reset reason");
        break;
    case ESP_RST_USB: //!< Reset by USB peripheral
        ESP_LOGI(TAG, "USB reset reason");
        break;
    case ESP_RST_JTAG: //!< Reset by JTAG
        ESP_LOGI(TAG, "JTAG reset reason");
        break;
    case ESP_RST_EFUSE: //!< Reset due to efuse error
        ESP_LOGI(TAG, "Efuse error reset reason");
        break;
    case ESP_RST_PWR_GLITCH: //!< Reset due to power glitch detected
        ESP_LOGI(TAG, "Power glitch detected reset reason");
        break;
    case ESP_RST_CPU_LOCKUP: //!< Reset due to CPU lock up (double exception)
        ESP_LOGI(TAG, "CPU lock up reset reason");
        break;
    }
    return ESP_OK;
}

// Entry point
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGD(TAG, "\n");
    ESP_LOGI(TAG, "Starting Zigbee GasMeter...");
    esp_err_t reset_error = report_reset_reason();

    ESP_ERROR_CHECK((deep_sleep_timer = xTimerCreate("deep_sleep_timer", portMAX_DELAY, pdFALSE, "d_s_t", enter_deep_sleep_cb)) == NULL ? ESP_FAIL : ESP_OK);
    // ESP_ERROR_CHECK((long_press_timer = xTimerCreate("long_press_timer", portMAX_DELAY, pdFALSE, "l_p_t", longpress_cb)) == NULL ? ESP_FAIL : ESP_OK);

    // intelligent button timers
    ESP_ERROR_CHECK((t0_2s_since_press = xTimerCreate("0.2s_since_press", pdMS_TO_TICKS(CLICK_PRESS_TIME_MS), pdFALSE, "2_t_p", t0_2s_since_press_cb)) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK((t4s_since_press = xTimerCreate("4s_since_press", pdMS_TO_TICKS(CLICK_HOLD_TIME_MS), pdFALSE, "4_s_p", t4s_since_press_cb)) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK((t0_2s_since_release = xTimerCreate("0.2s_since_release", pdMS_TO_TICKS(CLICK_RELEASE_TIME_MS), pdFALSE, "2_t_r", t0_2s_since_release_cb)) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK((t0_5s_since_release = xTimerCreate("0.5s_since_release", pdMS_TO_TICKS(BUTTON_EVENT_DELAY_MS), pdFALSE, "5_t_r", t0_5s_since_release_cb)) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK((t5s_since_release = xTimerCreate("5s_since_release", pdMS_TO_TICKS(BUTTON_RESET_STATE_MS), pdFALSE, "5_s_r", t20s_since_release_cb)) == NULL ? ESP_FAIL : ESP_OK);

    ESP_ERROR_CHECK((deep_sleep_queue_handle = xQueueCreate(1, sizeof(TickType_t))) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK((main_event_group_handle = xEventGroupCreate()) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK((report_event_group_handle = xEventGroupCreate()) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK(xTaskCreate(save_counter_task, "save_counter", 2048, NULL, 15, &save_counter_task_handle) != pdPASS);
    ESP_ERROR_CHECK(xTaskCreate(btn_press_task, "btn_press", 2048, NULL, 5, &btn_press_task_handle) != pdPASS);
    ESP_ERROR_CHECK(xTaskCreate(btn_release_task, "btn_release", 2048, NULL, 5, &btn_release_task_handle) != pdPASS);
    ESP_ERROR_CHECK(xTaskCreate(btn_task, "btn_task", 2048, NULL, 5, &btn_task_handle) != pdPASS);
    ESP_ERROR_CHECK(xTaskCreate(adc_task, "adc", 4096, NULL, 10, &adc_task_handle) != pdPASS);
    ESP_ERROR_CHECK((reset_instantaneous_demand_timer = xTimerCreate("reset_inst_dema", pdMS_TO_TICKS(TIME_TO_RESET_INSTANTANEOUS_D), pdFALSE, "r_i_d", reset_instantaneous_demand_cb)) == NULL ? ESP_FAIL : ESP_OK);

    if (reset_error != ESP_OK) {
        xEventGroupSetBits(report_event_group_handle, EXTENDED_STATUS_REPORT);
    }

    ESP_ERROR_CHECK(gm_gpio_interrup_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    ESP_ERROR_CHECK(gm_counter_load_nvs());
    ESP_ERROR_CHECK(gm_deep_sleep_init());
    // ESP_ERROR_CHECK(config_led());

    // start main loop
    ESP_ERROR_CHECK(xTaskCreate(gm_main_loop_task, "gas_meter_main", 8192, NULL, tskIDLE_PRIORITY, NULL) != pdPASS);
}