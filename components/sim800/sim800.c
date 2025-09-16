/******************************************************************************
 * Includes
*******************************************************************************/
#include <esp_log.h>
#include <driver/uart.h>

#include "sim800.h"


/******************************************************************************
 * Macros
*******************************************************************************/

#define SIM800_UART_TIMEOUT_MS      (1000)  /* Timeout for UART read in milliseconds */
#define SIM800_UART_BUFFER_SIZE     (512)   /* UART response buffer size */
#define SIM800_MAX_RETRIES          (10)    /* Max retries for sending commands */


/******************************************************************************
 * Static variable definition
*******************************************************************************/

static const char* SIM800_TAG = "SIM800";
static uart_port_t SIM800_UART_NUM;


/******************************************************************************
 * Static function declaration
*******************************************************************************/

bool sim800_check_response(const char *resp);


/******************************************************************************
 * Function definition
*******************************************************************************/

int sim800_init(uart_port_t uart_num) {
    SIM800_UART_NUM = uart_num;
    ESP_LOGI(SIM800_TAG, "SIM800C init on UART %d...", uart_num);

    /* response buffer init */
    char *response = malloc(SIM800_UART_BUFFER_SIZE);
    if (!response) { 
        ESP_LOGE(SIM800_TAG, "malloc failed");
        return ESP_FAIL; 
    } else {
        memset(response, 0, sizeof(response));
    }
    
    /* Check AT -> OK */
    int retries = SIM800_MAX_RETRIES;
    while (retries-- > 0) {
        sim800_send_cmd(SIM800_CMD_CHECK);
        sim800_read_response(response);
        if (sim800_check_response(response)) {
            ESP_LOGI(SIM800_TAG, "SIM800C module is responsive.");
            break;
        }

        ESP_LOGW(SIM800_TAG, "No valid response, retrying... (%d left)", retries);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (retries <= 0) {
        ESP_LOGE(SIM800_TAG, "No response from SIM800C module after %d retries", SIM800_MAX_RETRIES);
        free(response);
        return ESP_FAIL;
    }

    /* Check Sim card */
    sim800_send_cmd(SIM800_CMD_INFO_SIM);
    sim800_read_response(response);
    if (!sim800_check_response(response)) {
        ESP_LOGE(SIM800_TAG, "No SIM card detected");
        free(response);
        return ESP_FAIL;
    }

    /* Check Network registration */
    sim800_send_cmd(SIM800_CMD_INFO_NETWORK);
    sim800_read_response(response);
    if (!sim800_check_response(response)) {
        ESP_LOGE(SIM800_TAG, "Not registered in network");
        free(response);
        return ESP_FAIL;
    }

    free(response);
    return ESP_OK;
}


void sim800_send_cmd(const char *cmd) {
    /* Null pointer check */
    if (cmd == NULL) {
        ESP_LOGE(SIM800_TAG, "Command is NULL");
        return;
    }

    uart_write_bytes(SIM800_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(SIM800_UART_NUM, SIM800_CMD_END, 2);
    ESP_LOGD(SIM800_TAG, ">> %s", cmd);
}


void sim800_read_response(char *response) {
    /* Null pointer check */
    if (response == NULL) {
        ESP_LOGE(SIM800_TAG, "Response buffer is NULL");
        return;
    }

    char buffer[SIM800_UART_BUFFER_SIZE];
    int len = uart_read_bytes(SIM800_UART_NUM, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(SIM800_UART_TIMEOUT_MS));
    if (len > 0) {
        buffer[len] = '\0';
        ESP_LOGD(SIM800_TAG, "<< %s", buffer);
        strncpy(response, buffer, SIM800_UART_BUFFER_SIZE - 1);
        response[SIM800_UART_BUFFER_SIZE - 1] = '\0';
    } else if (len == 0) {
        ESP_LOGW(SIM800_TAG, "No response received or timeout");
        response[0] = '\0';
    } else {
        ESP_LOGE(SIM800_TAG, "Error reading response");
        response[0] = '\0';
    }
}



/******************************************************************************
 * Static function definition
*******************************************************************************/

/**
 * @brief Checks if the response contains "OK" or "ERROR"
 * 
 * @param resp : Response string to check
 * 
 * @return true on succes, false on error
 */
bool sim800_check_response(const char *resp) {
    if (strstr(resp, SIM800_RSP_OK)) {
        return true;
    }
    if (strstr(resp, "+CME ERROR")) {
        ESP_LOGE(SIM800_TAG, "SIM800 CME ERROR: %s", resp);
        return false;
    }
    if (strstr(resp, "+CMS ERROR")) {
        ESP_LOGE(SIM800_TAG, "SIM800 CMS ERROR: %s", resp);
        return false;
    }
    if (strstr(resp, SIM800_RSP_ERROR)) {
        ESP_LOGE(SIM800_TAG, "SIM800 ERROR response: %s", resp);
        return false;
    }
    
    return false;
}