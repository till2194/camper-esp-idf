#include <esp_log.h>

#include "sensors.h"
#include "mpu6050.h"
#include "bme280.h"

static const char* SENSOR_TAG = "SENSORS";

sensor_type_t sensor_list[] = {
    SENSOR_BME280
};

void sensors_init(void) {
    for (uint32_t i=0; i<(sizeof(sensor_list)/sizeof(sensor_type_t)); i++) {
        switch (sensor_list[i]) {
            case SENSOR_BME280:
                bme280_init(BME280_ADDR1);
                break;
            case SENSOR_MPU6050:
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