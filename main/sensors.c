#include <esp_log.h>

#include "sensors.h"
#include "mpu6050.h"
#include "bme280_handler.h"
#include "bme280.h"

static const char* SENSOR_TAG = "SENSORS";

static struct bme280_dev bme280_dev0;

sensor_type_t sensor_list[] = {
    SENSOR_BME280,
    // MPU_6050,
};

void sensors_init(void) {
    for (uint32_t i=0; i<(sizeof(sensor_list)/sizeof(sensor_type_t)); i++) {
        switch (sensor_list[i]) {
            case SENSOR_BME280:
                ESP_LOGI(SENSOR_TAG, "BME280 init...");
                bme280_startup(BME280_I2C_ADDR1, &bme280_dev0);
                break;
            case SENSOR_MPU6050:
                ESP_LOGI(SENSOR_TAG, "MPU6050 init...");
                mpu6050_init(MPU6050_ADDR1);
                break;
            default:
                ESP_LOGE(SENSOR_TAG, "Not supported sensor type enum=%u!", sensor_list[i]);
                break;
        }
    }
    
}

void sensors_task(void *args) {

}