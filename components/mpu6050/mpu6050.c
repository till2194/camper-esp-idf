#include <esp_log.h>
#include <driver/i2c_master.h>

#include "mpu6050.h"

static const char* MPU6050_TAG = "MPU6050";

i2c_master_dev_handle_t mpu6050_dev_handle;

static uint8_t mpu6050_read_reg(uint8_t reg);

void mpu6050_init(uint16_t address) {
    i2c_master_bus_handle_t master_handle;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &master_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_handle, &dev_cfg, &mpu6050_dev_handle));

    /* Check Who am I register */
    uint8_t val = mpu6050_read_reg(MPU6050_REG_WHOAMI);
    ESP_LOGD(MPU6050_TAG, "WHO AM I: %u", val);
    if (val != MPU6050_REG_WHOAMI_VAL) {
        ESP_LOGE(MPU6050_TAG, "Register WHO AM I has wrong value 0x%X!", val);
    }
}

static uint8_t mpu6050_read_reg(uint8_t reg) {
    uint8_t val;
    ESP_LOGI(MPU6050_TAG, "Reading register 0x%X", reg);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050_dev_handle, &reg, 1, &val, 1, -1));
    return val;
}