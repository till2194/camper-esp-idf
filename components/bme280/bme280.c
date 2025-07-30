#include <esp_log.h>
#include <driver/i2c_master.h>

#include "bme280.h"

static const char* BME280_TAG = "BME280";

i2c_master_dev_handle_t bme280_dev_handle;

static uint8_t bme280_read_reg(uint8_t reg);

void bme280_init(uint16_t address) {
    i2c_master_bus_handle_t master_handle;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &master_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_handle, &dev_cfg, &bme280_dev_handle));

    /* Check ID register */
    uint8_t val = bme280_read_reg(BME280_REG_ID);
    ESP_LOGI(BME280_TAG, "ID: 0x%X", val);
    if (val != BME280_REG_ID_VAL) {
        ESP_LOGE(BME280_TAG, "Register ID has wrong value 0x%X!", val);
    }
}

static uint8_t bme280_read_reg(uint8_t reg) {
    uint8_t val;
    ESP_LOGI(BME280_TAG, "Reading register 0x%X", reg);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bme280_dev_handle, &reg, 1, &val, 1, -1));
    return val;
}