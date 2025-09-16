/******************************************************************************
 * Includes
*******************************************************************************/
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "sensor.h"
#include "mpu6050.h"
#include "bme280_handler.h"
#include "bme280.h"
#include "ds3231.h"


/******************************************************************************
 * Macros
*******************************************************************************/


/******************************************************************************
 * Static variable definition
*******************************************************************************/

static const char* SENSOR_TAG = "SENSOR";
static sensor_data_t sensor_data;

sensor_type_t sensor_list[] = {
    // SENSOR_BME280,
    // SENSOR_MPU6050,
    // SENSOR_DS3231
};

static SemaphoreHandle_t sensor_data_mutex;

/******************************************************************************
 * Static function declaration
*******************************************************************************/


/******************************************************************************
 * Function definition
*******************************************************************************/

void sensor_init(void) {
    /* Create Mutex */
    sensor_data_mutex = xSemaphoreCreateMutex();

    /* Init sensors */
    for (uint32_t i=0; i<(sizeof(sensor_list)/sizeof(sensor_type_t)); i++) {
        switch (sensor_list[i]) {
            case SENSOR_BME280:
                ESP_LOGI(SENSOR_TAG, "BME280 init...");
                bme280_startup(BME280_I2C_ADDR1);
                break;
            
            case SENSOR_MPU6050:
                ESP_LOGI(SENSOR_TAG, "MPU6050 init...");
                mpu6050_init(MPU6050_I2C_ADDR1);
                break;

            case SENSOR_DS3231:
                ESP_LOGI(SENSOR_TAG, "DS3231 init...");
                ds3231_init(DS3231_I2C_ADDR);
                break;
            
            default:
                ESP_LOGE(SENSOR_TAG, "Not supported sensor type enum=%u!", sensor_list[i]);
                break;
        }
    }

    /* Start sensor task */
    xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, NULL);
}

void sensor_task(void *args) {
    while(1) {
        for (uint32_t i=0; i<(sizeof(sensor_list)/sizeof(sensor_type_t)); i++) {
            switch (sensor_list[i]) {
                case SENSOR_BME280:
                    ESP_LOGD(SENSOR_TAG, "BME280 reading data...");
                    if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                        bme280_get_data(&sensor_data.temperature, &sensor_data.pressure, &sensor_data.humidity);
                        xSemaphoreGive(sensor_data_mutex);
                    } else {
                        ESP_LOGE(SENSOR_TAG, "sensor_task: Could not get mutex!");
                    }
                    break;

                case SENSOR_MPU6050:
                    ESP_LOGI(SENSOR_TAG, "MPU6050 reading acc...");
                    
                    if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                        mpu6050_read_acc(&sensor_data.acc_x, &sensor_data.acc_y, &sensor_data.acc_z);
                        xSemaphoreGive(sensor_data_mutex);
                    } else {
                        ESP_LOGE(SENSOR_TAG, "sensor_task: Could not get mutex!");
                    }
                    break;

                case SENSOR_DS3231:
                    ESP_LOGI(SENSOR_TAG, "DS3231 reading time...");
                    ds3231_time_t ds_time;
                    ds3231_read_time(&ds_time);
                    break;
                
                default:
                    ESP_LOGE(SENSOR_TAG, "Not supported sensor type enum=%u!", sensor_list[i]);
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_DELAY_MS));
    }
}


void sensor_get_data(sensor_data_t *data) {
    /* Null pointer check */
    if (data == NULL) {
        ESP_LOGE(SENSOR_TAG, "sensor_get_data: Null pointer!");
        return;
    }

    /* Get mutex and copy data */
    if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
        *data = sensor_data;
        xSemaphoreGive(sensor_data_mutex);
    } else {
        ESP_LOGE(SENSOR_TAG, "sensor_get_data: Could not get mutex!");
        return;
    }
}


/******************************************************************************
 * Static function definition
*******************************************************************************/
