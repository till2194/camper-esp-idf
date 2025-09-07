/******************************************************************************
 * Includes
*******************************************************************************/
#include <esp_log.h>

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
static struct bme280_dev bme280_dev0;
static sensor_data_t sensor_data;

sensor_type_t sensor_list[] = {
    SENSOR_BME280,
    // SENSOR_MPU6050,
    SENSOR_DS3231
};


/******************************************************************************
 * Static function declaration
*******************************************************************************/


/******************************************************************************
 * Function definition
*******************************************************************************/

void sensor_init(void) {
    for (uint32_t i=0; i<(sizeof(sensor_list)/sizeof(sensor_type_t)); i++) {
        switch (sensor_list[i]) {
            case SENSOR_BME280:
                ESP_LOGI(SENSOR_TAG, "BME280 init...");
                bme280_startup(BME280_I2C_ADDR1, &bme280_dev0);
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
}

void sensor_task(void *args) {
    for (uint32_t i=0; i<(sizeof(sensor_list)/sizeof(sensor_type_t)); i++) {
        switch (sensor_list[i]) {
            case SENSOR_BME280:
                ESP_LOGI(SENSOR_TAG, "BME280 reading data...");
                // TODO: Implement periodic read
                break;

            case SENSOR_MPU6050:
                ESP_LOGI(SENSOR_TAG, "MPU6050 reading acc...");
                mpu6050_read_acc(&sensor_data.acc_x, &sensor_data.acc_y, &sensor_data.acc_z);
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
}


/******************************************************************************
 * Static function definition
*******************************************************************************/
