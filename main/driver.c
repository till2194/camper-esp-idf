#include <esp_log.h>
#include <driver/i2c_master.h>

#include "driver.h"

static const char* DRIVER_TAG = "DRIVER";

static void driver_i2c_init(void);

void driver_init(void) {
    ESP_LOGI(DRIVER_TAG, "I2C init...");
    driver_i2c_init();
}

static void driver_i2c_init(void) {
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
}