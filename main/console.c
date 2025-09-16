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


/******************************************************************************
 * Static function declaration
*******************************************************************************/


/******************************************************************************
 * Function definition
*******************************************************************************/

void console_init(void) {
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    repl_config.prompt = PROMPT_STR;
    repl_config.max_cmdline_length = 1024;

    esp_console_register_help_command();

    /* Set up UART REPL console */
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    /* Start REPL console */
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}


/******************************************************************************
 * Static function definition
*******************************************************************************/
