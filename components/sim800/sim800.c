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

static const char* SIM800_TAG = "SIM800";                       /* Tag for logging */
static uart_port_t sim800_uart_num;                             /* UART port for SIM800 communication */
void (*sim800_sms_handler)(const char *sms) = NULL;             /* SMS handler callback */
static char sim800_response_buffer[SIM800_UART_BUFFER_SIZE];    /* Response buffer for SIM800 communication */


/******************************************************************************
 * Static function declaration
*******************************************************************************/

int sim800_console_cmd(int argc, char** argv);
void sim800_uart_receive_task(void *pvParameters);

/******************************************************************************
 * Function definition
*******************************************************************************/

int sim800_init(uart_port_t uart_num, void* sms_handler) {
    /* return value */
    esp_err_t ret = ESP_OK;

    /* Set SMS handler */
    if (sms_handler != NULL) {
        sim800_sms_handler = sms_handler;
    } else {
        ESP_LOGE(SIM800_TAG, "No SMS handler provided!");
        ret = ESP_FAIL;
    }

    /* Reset module with GPIO */
    /* TODO: implement GPIO logic */
    vTaskDelay(pdMS_TO_TICKS(1000));

    sim800_uart_num = uart_num;
    ESP_LOGI(SIM800_TAG, "SIM800C initialized on UART %d...", uart_num);

    /* Init buffer */
    memset(sim800_response_buffer, 0, SIM800_UART_BUFFER_SIZE);
    
    /* Check AT -> OK 
     * (repeat multiple times, the module will synchronize to baud rate)
     */
    if (ret == ESP_OK) {
        int retries = SIM800_MAX_RETRIES;
        while (retries-- > 0) {
            sim800_send_cmd(SIM800_CMD_CHECK);
            ret = sim800_read_response(sim800_response_buffer, SIM800_UART_BUFFER_SIZE);
            if (ret == ESP_OK) {
                ESP_LOGI(SIM800_TAG, "SIM800C module is responsive.");
                break;
            } else {
                ESP_LOGW(SIM800_TAG, "No valid response, retrying... (%d left)", retries);
            }

            vTaskDelay(pdMS_TO_TICKS(500));
        }

        if (retries <= 0) {
            ESP_LOGE(SIM800_TAG, "No response from SIM800C module after %d retries", SIM800_MAX_RETRIES);
            ret = ESP_FAIL;
        }
    }
    
    /* Check Sim card */
    if (ret == ESP_OK) {
        sim800_send_cmd(SIM800_CMD_INFO_SIM);
        ret = sim800_read_response(sim800_response_buffer, SIM800_UART_BUFFER_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(SIM800_TAG, "No SIM card detected");
            ret = ESP_FAIL;
        }
    }

    /* Check Network registration */
    if (ret == ESP_OK) {
        sim800_send_cmd(SIM800_CMD_INFO_NETWORK);
        ret = sim800_read_response(sim800_response_buffer, SIM800_UART_BUFFER_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(SIM800_TAG, "Not registered in network");
            ret = ESP_FAIL;
        }
    }

    /* Check Network quality */
    if (ret == ESP_OK) {
        uint8_t rssi = 0, ber = 0;
        ret = sim800_get_signal_quality(&rssi, &ber);
        if (ret != ESP_OK) {
            ESP_LOGE(SIM800_TAG, "Not registered in network");
            ret = ESP_FAIL;
        } else {
            ESP_LOGI(SIM800_TAG, "Signal quality: RSSI=%d, BER=%d", rssi, ber);
        }
    }

    /* Set SMS text mode */
    if (ret == ESP_OK) {
        sim800_send_cmd(SIM800_CMD_SMS_TEXT_MODE);
        ret = sim800_read_response(sim800_response_buffer, SIM800_UART_BUFFER_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(SIM800_TAG, "Failed to set SMS text mode");
            ret = ESP_FAIL;
        }
    }

    /* Enable SMS notification */
    if (ret == ESP_OK) {
        sim800_send_cmd(SIM800_CMD_SMS_TX_CONFIG);
        ret = sim800_read_response(sim800_response_buffer, SIM800_UART_BUFFER_SIZE);
        if (ret != ESP_OK) {
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

    /* Start UART receive task */
    #if 0 
    if (ret == ESP_OK) {
        xTaskCreate(sim800_uart_receive_task, "sim800_uart_rx", 4096, NULL, 10, NULL);
    }
    #endif

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


int sim800_read_response(char *buf, size_t len) {
    /* Null pointer check */
    if (buf == NULL) {
        ESP_LOGE(SIM800_TAG, "Response buffer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* Clear buffer */
    memset(buf, 0, len);

    /* Read response */
    int bytes_read = uart_read_bytes(sim800_uart_num, (uint8_t *)buf, len - 1, pdMS_TO_TICKS(SIM800_UART_TIMEOUT_MS));
    if (bytes_read > 0) {
        buf[bytes_read] = '\0';
        ESP_LOGD(SIM800_TAG, "<< %s", buf);
    } else if (bytes_read == 0) {
        ESP_LOGW(SIM800_TAG, "No response received or timeout");
        buf[0] = '\0';
        return ESP_OK;
    } else {
        ESP_LOGE(SIM800_TAG, "Error reading response: %i", bytes_read);
        buf[0] = '\0';
        return ESP_FAIL;
    }

    /* check response for OK */
    if (strstr(buf, SIM800_RSP_OK)) {
        return ESP_OK;
    } else {
        ESP_LOGE(SIM800_TAG, "Error response: %s", buf);
        return ESP_FAIL;
    }
}


int sim800_get_signal_quality(uint8_t *rssi, uint8_t *ber) {
    /* Null pointer check */
    if (rssi == NULL || ber == NULL) {
        ESP_LOGE(SIM800_TAG, "RSSI or BER pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    sim800_send_cmd(SIM800_CMD_SIGNAL_TEST);
    esp_err_t ret = sim800_read_response(sim800_response_buffer, SIM800_UART_BUFFER_SIZE);
    if (ret != ESP_OK) {
        return ret;
    }

    char *p = strstr(sim800_response_buffer, "+CSQ:");
    if (p) {
        int rssi_val = 0, ber_val = 0;
        int matched = sscanf(p, "+CSQ: %d,%d", &rssi_val, &ber_val);
        if (matched == 2) {
            *rssi = rssi_val;
            *ber = ber_val;
            return ESP_OK;
        } else {
            ESP_LOGE(SIM800_TAG, "Failed to parse CSQ response");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(SIM800_TAG, "No +CSQ response found");
        return ESP_FAIL;
    }
}


/******************************************************************************
 * Static function definition
*******************************************************************************/

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
                    sim800_read_response(sms_resp, SIM800_UART_BUFFER_SIZE);
                    // Call handler
                    sim800_sms_handler(sms_resp);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
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
    sim800_read_response(sim800_response_buffer, SIM800_UART_BUFFER_SIZE);
    printf("Response:\n%s\n", sim800_response_buffer);
    return ESP_OK;
}