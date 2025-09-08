#include <stdio.h>
#include <esp_log.h>
#include <driver/i2c_master.h>

#include "driver.h"
#include "sensor.h"
#include "console.h"

static const char* MAIN_TAG = "MAIN";

void app_main(void) {
    ESP_LOGI(MAIN_TAG, "Welcome to the camper app.");

    ESP_LOGI(MAIN_TAG, "Starting drivers...");
    driver_init();

    ESP_LOGI(MAIN_TAG, "Starting sensors...");
    sensor_init();

    ESP_LOGI(MAIN_TAG, "Starting console...");
    console_init();
}