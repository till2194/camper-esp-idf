/******************************************************************************
 * Includes
*******************************************************************************/
#include <esp_log.h>
#include <esp_console.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "sensor.h"
#include "driver.h"
#include "mpu6050.h"
#include "bme280_handler.h"
#include "bme280.h"
#include "ds3231.h"
#include "sim800.h"


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
    // SENSOR_DS3231,
    SENSOR_SIM800
};

static SemaphoreHandle_t sensor_data_mutex;

/******************************************************************************
 * Static function declaration
*******************************************************************************/

int sensor_console_cmd(int argc, char** argv);
int time_console_cmd(int argc, char** argv);
int sensor_sim800_sms_handler(const char *sms);

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
                mpu6050_init(MPU6050_I2C_ADDR0);
                break;

            case SENSOR_DS3231:
                ESP_LOGI(SENSOR_TAG, "DS3231 init...");
                ds3231_init(DS3231_I2C_ADDR);
                break;

            case SENSOR_SIM800:
                ESP_LOGI(SENSOR_TAG, "SIM800 init...");
                sim800_init(DRIVER_UART_NUM, sensor_sim800_sms_handler);
                break;
            
            default:
                ESP_LOGE(SENSOR_TAG, "Not supported sensor type enum=%u!", sensor_list[i]);
                break;
        }
    }

    /* Add console command */
    esp_console_cmd_t cmd = {
        .command = "sensor-read",
        .help = "Print all sensors and their values",
        .hint = NULL,
        .func = sensor_console_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));

    /* TODO: move somewhere else */
    esp_console_cmd_t cmd2 = {
        .command = "time-set",
        .help = "Set the current time",
        .hint = NULL,
        .func = time_console_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd2));

    /* Start sensor task */
    xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, NULL);
}

void sensor_task(void *args) {
    while(1) {
        for (uint32_t i=0; i<(sizeof(sensor_list)/sizeof(sensor_type_t)); i++) {
            switch (sensor_list[i]) {
                case SENSOR_BME280:
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
                    ds3231_time_t ds_time;
                    ds3231_read_time(&ds_time);
                    if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                        ds3231_convert_dstime_to_tmtime(&ds_time, &sensor_data.time);
                        xSemaphoreGive(sensor_data_mutex);
                    } else {
                        ESP_LOGE(SENSOR_TAG, "sensor_task: Could not get mutex!");
                    }
                    break;

                case SENSOR_SIM800:
                    /* handled in sim800 uart task */
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

/**
 * @brief Console command to read sensors and print their values 
 * 
 * @param argc  : Argument count
 * @param argv  : Argument values
 * 
 * @return ESP_OK on success, error code otherwise
 */
int sensor_console_cmd(int argc, char** argv) {
    sensor_data_t data;
    sensor_get_data(&data);
    for (uint32_t i=0; i<(sizeof(sensor_list)/sizeof(sensor_type_t)); i++) {
        switch (sensor_list[i]) {
            case SENSOR_BME280:
                printf("Temperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%\n", data.temperature, data.pressure/100.0, data.humidity);
                break;
            
            case SENSOR_MPU6050:
                printf("Acc: %.2f %.2f %.2f\n", data.acc_x, data.acc_y, data.acc_z);
                break;
            
            case SENSOR_DS3231:
                char buffer[26];
                strftime(buffer, sizeof(buffer), "%H:%M:%S %d.%m.%Y" , &data.time);
                printf("Time: %s\n", buffer);
                break;
            
            case SENSOR_SIM800:
                printf("Signal strength: %d dBm, Bit error rate: %d\n", data.rssi, data.ber);
                break;

            default:
                ESP_LOGE(SENSOR_TAG, "Not supported sensor type enum=%u!", sensor_list[i]);
                break;
        }
    }
    return ESP_OK;
}


/**
 * @brief Sets the system time to the given time (console command) 
 * 
 * @param argc  : Argument count
 * @param argv  : Argument values
 * 
 * @return ESP_OK on success, error code otherwise 
 */
int time_console_cmd(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage: time-set <HH:MM:SS-DD.MM.YYYY>\n");
        return ESP_ERR_INVALID_ARG;
    }

    struct tm tm;
    if (strptime(argv[1], "%H:%M:%S-%d.%m.%Y", &tm) == NULL) {
        printf("Invalid date format. Use HH:MM:SS-DD.MM.YYYY\n");
        return ESP_ERR_INVALID_ARG;
    }

    ds3231_time_t ds_time;
    ds3231_convert_tmtime_to_dstime(&tm, &ds_time);
    ds3231_write_time(&ds_time);

    return ESP_OK;
}


/**
 * @brief Handler for incoming SMS messages from SIM800
 * 
 * @param sms : Pointer to the SMS message string
 * 
 * @return ESP_OK on success, error code otherwise
 */
int sensor_sim800_sms_handler(const char *sms) {
    ESP_LOGI(SENSOR_TAG, "Received SMS: %s", sms);
    return ESP_OK;
}