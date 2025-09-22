/******************************************************************************
 * Includes
*******************************************************************************/
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#include "driver.h"


/******************************************************************************
 * Macros
*******************************************************************************/


/******************************************************************************
 * Static variable definition
*******************************************************************************/

static const char* DRIVER_TAG = "DRIVER";


/******************************************************************************
 * Static function declaration
*******************************************************************************/

/**
 * @brief Inits the I2C driver
 * 
 */
static void driver_i2c_init(void);


/**
 * @brief Inits the UART driver
 * 
 */
static void driver_uart_init(void);


/**
 * @brief Inits the GPIO driver
 * 
 */
static void driver_gpio_init(void);


/******************************************************************************
 * Function definition
*******************************************************************************/

void driver_init(void) {
    ESP_LOGI(DRIVER_TAG, "I2C init...");
    driver_i2c_init();

    ESP_LOGI(DRIVER_TAG, "UART 2 init...");
    driver_uart_init();

    ESP_LOGI(DRIVER_TAG, "GPIO init...");
    driver_gpio_init();
}


/******************************************************************************
 * Static function definition
*******************************************************************************/

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


static void driver_uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = DRIVER_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(DRIVER_UART_NUM, DRIVER_UART_BUFFER_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(DRIVER_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(DRIVER_UART_NUM, DRIVER_UART_TX_PIN, DRIVER_UART_RX_PIN, -1, -1));
}


static void driver_gpio_init(void) {
    /* Config init  */
    gpio_config_t io_conf = {};

    /* Outputs */
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_GPO_HEATER, 0));
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_GPO_LED, 0));
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_GPO_LED1, 0));
    ESP_ERROR_CHECK(gpio_set_level(DRIVER_GPO_LED2, 0));

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << DRIVER_GPO_HEATER) | (1ULL << DRIVER_GPO_LED) | (1ULL << DRIVER_GPO_LED1) | (1ULL << DRIVER_GPO_LED2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* Inputs */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << DRIVER_GPI_SW1) | (1ULL << DRIVER_GPI_SW2);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}