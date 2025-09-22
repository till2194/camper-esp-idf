/******************************************************************************
 * Includes
*******************************************************************************/
#include <esp_log.h>
#include <esp_console.h>

#include "console.h"
#include "sensor.h"
#include "driver.h"

/******************************************************************************
 * Macros
*******************************************************************************/

#define CONSOLE_TAG "CONSOLE"
#define CONSOLE_PROMPT_STR "oscar> "

/******************************************************************************
 * Static variable definition
*******************************************************************************/


/******************************************************************************
 * Static function declaration
*******************************************************************************/

/**
 * @brief Console command to set a GPIO
 * 
 * @param argc : Argument count
 * @param argv : Argument values
 * 
 * @return ESP_OK on success
 */
static int console_gpio_set_cmd(int argc, char** argv) {
    if (argc < 3) {
        ESP_LOGE(CONSOLE_TAG, "Usage: gpio-set <pin> <0/1>");
        return ESP_FAIL;
    }
    int pin = atoi(argv[1]);
    int value = atoi(argv[2]);
    esp_err_t ret = gpio_set_level(pin, value);
    if (ret != ESP_OK) {
        ESP_LOGE(CONSOLE_TAG, "Failed to set GPIO %d to %d", pin, value);
        return ret;
    } else {
        printf("GPIO %d set to %d\n", pin, value);
    }

    return ESP_OK;
}


/******************************************************************************
 * Function definition
*******************************************************************************/

void console_init(void) {
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    repl_config.prompt = CONSOLE_PROMPT_STR;
    repl_config.max_cmdline_length = 1024;

    esp_console_register_help_command();

    /* Set up UART REPL console */
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    /* Start REPL console */
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    /* Set up GPIO command */
    esp_console_cmd_t cmd = {
        .command = "gpio-set",
        .help = "Set a GPIO",
        .hint = NULL,
        .func = console_gpio_set_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


/******************************************************************************
 * Static function definition
*******************************************************************************/
