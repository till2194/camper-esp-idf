/******************************************************************************
 * Includes
*******************************************************************************/
#include <esp_log.h>
#include <esp_console.h>

#include "console.h"
#include "sensor.h"

/******************************************************************************
 * Macros
*******************************************************************************/
#define PROMPT_STR "oscar> "

/******************************************************************************
 * Static variable definition
*******************************************************************************/

static const char* TAG = "CONSOLE";

/******************************************************************************
 * Static function declaration
*******************************************************************************/

void console_register_cmds(void);
int console_cmd_sensor(int argc, char** argv);

/******************************************************************************
 * Function definition
*******************************************************************************/

void console_init(void) {
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    repl_config.prompt = PROMPT_STR;
    repl_config.max_cmdline_length = 1024;

    console_register_cmds();

    /* Set up UART REPL console */
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    /* Start REPL console */
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}


/**
 * @brief Register console commands
 * 
 */
void console_register_cmds(void) {
    
    esp_console_register_help_command();

    esp_console_cmd_t cmd = {
        .command = "read-sensors",
        .help = "Print all sensors and their values",
        .hint = NULL,
        .func = console_cmd_sensor,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


/**
 * @brief Test command 
 * 
 * @param argc 
 * @param argv 
 */
int console_cmd_sensor(int argc, char** argv) {
    sensor_data_t data;
    sensor_get_data(&data);
    ESP_LOGI(TAG, "Temperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%", data.temperature, data.pressure/100.0, data.humidity);

    return ESP_OK;
}