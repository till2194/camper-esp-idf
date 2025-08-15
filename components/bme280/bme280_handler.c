
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <esp_log.h>
#include <driver/i2c_master.h>

#include "bme280.h"
#include "bme280_handler.h"

/******************************************************************************
 *                               Macros                                       *
 ******************************************************************************/
#define BME280_TIMEOUT_MS   (-1)

/******************************************************************************
 *                    Static variable definition                              *
 ******************************************************************************/
static const char* BME280_TAG = "BME280";

static i2c_master_dev_handle_t i2c_dev_handle;

/******************************************************************************
 *                    Static function definition                              *
 ******************************************************************************/

/**
 * I2C read function map to ESP platform
 */
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{    
    i2c_master_dev_handle_t dev_handle = *(i2c_master_dev_handle_t*) intf_ptr;

    esp_err_t esp_ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, reg_data, length, BME280_TIMEOUT_MS);
 
    BME280_INTF_RET_TYPE bme280_ret;
    if (esp_ret == ESP_OK) {
        bme280_ret = BME280_OK;
    } else {
        ESP_LOGE(BME280_TAG, "I2C write failed: %s", esp_err_to_name(esp_ret));
        bme280_ret = BME280_E_COMM_FAIL;
    }

    return bme280_ret;
}

/**
 * I2C write function map to ESP platform
 */
BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_master_dev_handle_t dev_handle = *(i2c_master_dev_handle_t*) intf_ptr;

    uint8_t buffer_tx[length + 1];
    buffer_tx[0] = reg_addr;

    for (uint32_t i = 0; i< length; i++) {
        buffer_tx[i+1] = reg_data[i];
    }

    esp_err_t esp_ret = i2c_master_transmit(dev_handle, buffer_tx, sizeof(buffer_tx), BME280_TIMEOUT_MS);

    BME280_INTF_RET_TYPE bme280_ret;
    if (esp_ret == ESP_OK) {
        bme280_ret = BME280_OK;
    } else {
        ESP_LOGE(BME280_TAG, "I2C write failed: %s", esp_err_to_name(esp_ret));
        bme280_ret = BME280_E_COMM_FAIL;
    }

    return bme280_ret;
}

/**
 * SPI read function map to ESP platform
 */
BME280_INTF_RET_TYPE bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    ESP_LOGE(BME280_TAG, "SPI communication not implemented!");

    return BME280_E_COMM_FAIL;
}

/**
 * SPI write function map to ESP platform
 */
BME280_INTF_RET_TYPE bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    ESP_LOGE(BME280_TAG, "SPI communication not implemented!");
    
    return BME280_E_COMM_FAIL;
}

/**
 * Delay function map to ESP platform
 */
void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    esp_rom_delay_us(period);
}

/**
 *  @brief Prints the execution status of the APIs.
 */
void bme280_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        printf("%s\n", api_name);

        switch (rslt)
        {
            case BME280_E_NULL_PTR:
                ESP_LOGE(BME280_TAG, "Error [%d] : Null pointer error.", rslt);
                break;

            case BME280_E_COMM_FAIL:
                ESP_LOGE(BME280_TAG, "Error [%d] : Communication failure error.", rslt);
                break;

            case BME280_E_DEV_NOT_FOUND:
                ESP_LOGE(BME280_TAG, "Error [%d] : Device not found error.", rslt);
                break;

            case BME280_E_INVALID_LEN:
                ESP_LOGE(BME280_TAG, "Error [%d] : Invalid length error.", rslt);
                break;

            default:
                ESP_LOGE(BME280_TAG, "Error [%d] : Unknown error code", rslt);
                break;
        }
    }
}

int8_t bme280_startup(uint16_t address, struct bme280_dev *dev) {
    BME280_INTF_RET_TYPE rslt;
    uint32_t period;
    struct bme280_settings settings;

    dev->read = bme280_i2c_read;
    dev->write = bme280_i2c_write;
    dev->intf = BME280_I2C_INTF;
    dev->delay_us = bme280_delay_us;
    
    i2c_master_bus_handle_t master_handle;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &master_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    esp_err_t result = i2c_master_bus_add_device(master_handle, &dev_cfg, &i2c_dev_handle);
    ESP_ERROR_CHECK(result);

    /* Store I2C device handle pointer */
    dev->intf_ptr = &i2c_dev_handle;

    rslt = bme280_init(dev);
    bme280_error_codes_print_result("bme280_init", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bme280_get_sensor_settings(&settings, dev);
    bme280_error_codes_print_result("bme280_get_sensor_settings", rslt);

    /* Configuring the over-sampling rate, filter coefficient and standby time */
    /* Overwrite the desired settings */
    settings.filter = BME280_FILTER_COEFF_2;

    /* Over-sampling rate for humidity, temperature and pressure */
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;

    /* Setting the standby time */
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, dev);
    bme280_error_codes_print_result("bme280_set_sensor_settings", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, dev);
    bme280_error_codes_print_result("bme280_set_power_mode", rslt);

    /* Calculate measurement time in microseconds */
    rslt = bme280_cal_meas_delay(&period, &settings);
    bme280_error_codes_print_result("bme280_cal_meas_delay", rslt);

    ESP_LOGI(BME280_TAG, "Temperature calculation (Data displayed are compensated values)");
    ESP_LOGI(BME280_TAG, "Measurement time : %lu us", (long unsigned int)period);

    rslt = bme280_get_data(period, dev);
    bme280_error_codes_print_result("bme280_get_data", rslt);

    return 0;
}

int8_t bme280_get_data(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t done = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    while (done != 1)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

            ESP_LOGI(BME280_TAG, "T=%lf °C, p=%lf Pa, h=%lf %%rH", comp_data.temperature, comp_data.pressure, comp_data.humidity);
            
            done = 1;
        }
    }

    return rslt;
}