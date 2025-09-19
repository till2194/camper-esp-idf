/******************************************************************************
 * Includes
*******************************************************************************/
#include <esp_log.h>
#include <esp_console.h>
#include <driver/uart.h>

#include "sim800.h"


/******************************************************************************
 * Macros
*******************************************************************************/

#define SIM800_UART_TIMEOUT_MS      (100)  /* Timeout for UART read in milliseconds */
#define SIM800_UART_BUFFER_SIZE     (512)   /* UART response buffer size */
#define SIM800_MAX_RETRIES          (10)    /* Max retries for sending commands */


/******************************************************************************
 * Static variable definition
*******************************************************************************/

static const char* SIM800_TAG = "SIM800";
static uart_port_t sim800_uart_num;


/******************************************************************************
 * Static function declaration
*******************************************************************************/

bool sim800_check_response(const char *resp);
int sim800_console_cmd(int argc, char** argv);
void sim800_uart_receive_task(void *pvParameters);
void sim800_sms_receive(const char *sms);

/******************************************************************************
 * Function definition
*******************************************************************************/

int sim800_init(uart_port_t uart_num) {
    /* return value */
    esp_err_t ret = ESP_OK;

    /* Reset module with GPIO */
    /* TODO: implement GPIO logic */

    sim800_uart_num = uart_num;
    ESP_LOGI(SIM800_TAG, "SIM800C init on UART %d...", uart_num);

    /* response buffer init */
    char *response = malloc(SIM800_UART_BUFFER_SIZE);
    if (!response) { 
        ESP_LOGE(SIM800_TAG, "malloc failed");
        ret = ESP_FAIL; 
    } else {
        memset(response, 0, sizeof(response));
    }
    
    /* Check AT -> OK */
    if (ret == ESP_OK) {
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
            ret = ESP_FAIL;
        }
    }
    
    /* Check Sim card */
    if (ret == ESP_OK) {
        sim800_send_cmd(SIM800_CMD_INFO_SIM);
        sim800_read_response(response);
        if (!sim800_check_response(response)) {
            ESP_LOGE(SIM800_TAG, "No SIM card detected");
            free(response);
            ret = ESP_FAIL;
        }
    }

    /* Check Network registration */
    if (ret == ESP_OK) {
        sim800_send_cmd(SIM800_CMD_INFO_NETWORK);
        sim800_read_response(response);
        if (!sim800_check_response(response)) {
            ESP_LOGE(SIM800_TAG, "Not registered in network");
            free(response);
            ret = ESP_FAIL;
        }
    }

    /* Set SMS text mode */
    if (ret == ESP_OK) {
        sim800_send_cmd(SIM800_CMD_SMS_TEXT_MODE);
        sim800_read_response(response);
        if (!sim800_check_response(response)) {
            ESP_LOGE(SIM800_TAG, "Failed to set SMS text mode");
            ret = ESP_FAIL;
        }
    }

    /* Enable SMS notification */
    if (ret == ESP_OK) {
        sim800_send_cmd(SIM800_CMD_SMS_TX_CONFIG);
        sim800_read_response(response);
        if (!sim800_check_response(response)) {
            ESP_LOGE(SIM800_TAG, "Failed to enable SMS notifications");
            ret = ESP_FAIL;
        }
    }

    /* Add console command */
    if (ret == ESP_OK) {
        esp_console_cmd_t cmd = {
            .command = "sim-send",
            .help = "Send AT command to SIM800",
            .hint = NULL,
            .func = sim800_console_cmd,
        };
        ret = esp_console_cmd_register(&cmd);
    }

    /* Sart UART receive task */
    if (ret == ESP_OK) {
        xTaskCreate(sim800_uart_receive_task, "sim800_uart_rx", 4096, NULL, 10, NULL);
    }

    free(response);
    return ret;
}


void sim800_send_cmd(const char *cmd) {
    /* Null pointer check */
    if (cmd == NULL) {
        ESP_LOGE(SIM800_TAG, "Command is NULL");
        return;
    }

    uart_write_bytes(sim800_uart_num, cmd, strlen(cmd));
    uart_write_bytes(sim800_uart_num, SIM800_CMD_END, 2);
    ESP_LOGD(SIM800_TAG, ">> %s", cmd);
}


void sim800_read_response(char *response) {
    /* Null pointer check */
    if (response == NULL) {
        ESP_LOGE(SIM800_TAG, "Response buffer is NULL");
        return;
    }

    char buffer[SIM800_UART_BUFFER_SIZE];
    int len = uart_read_bytes(sim800_uart_num, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(SIM800_UART_TIMEOUT_MS));
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


/*
 * UART receive task to handle unsolicited notifications (e.g., +CMTI for new SMS)
 */
void sim800_uart_receive_task(void *pvParameters) {
    char buffer[SIM800_UART_BUFFER_SIZE];
    while (1) {
        int len = uart_read_bytes(sim800_uart_num, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(500));
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGD(SIM800_TAG, "[RX] %s", buffer);
            char *cmti = strstr(buffer, "+CMTI: ");
            if (cmti) {
                // Example: +CMTI: "SM",3
                int index = -1;
                char *comma = strchr(cmti, ',');
                if (comma) {
                    index = atoi(comma + 1);
                }
                if (index > 0) {
                    // Read SMS at index
                    char cmd[32];
                    snprintf(cmd, sizeof(cmd), "AT+CMGR=%d", index);
                    sim800_send_cmd(cmd);
                    char sms_resp[SIM800_UART_BUFFER_SIZE];
                    sim800_read_response(sms_resp);
                    // Call handler
                    sim800_sms_receive(sms_resp);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/**
 * @brief Handler for received SMS
 * 
 * @param sms : SMS content
 */
void sim800_sms_receive(const char *sms) {
    ESP_LOGI(SIM800_TAG, "Received SMS: %s", sms);
}


/**
 * @brief Console command to interact with SIM800
 * 
 * @param argc : Argument count
 * @param argv : Argument values
 */
int sim800_console_cmd(int argc, char** argv) {
    /* Send command to SIM800 and print response */
    if (argc < 2) {
        ESP_LOGE(SIM800_TAG, "Usage: sim800-cmd <AT command>");
        return ESP_FAIL;
    }
    char command[128] = {0};
    snprintf(command, sizeof(command), "%s", argv[1]);
    sim800_send_cmd(command);
    char response[SIM800_UART_BUFFER_SIZE];
    sim800_read_response(response);
    printf("Response: %s\n", response);
    return ESP_OK;

}