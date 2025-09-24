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

#define SIM800_UART_TIMEOUT_MS      (10)        /* Timeout for UART read for one Byte in milliseconds */
#define SIM800_UART_BUFFER_SIZE     (512)       /* UART response buffer size */
#define SIM800_MAX_RETRIES          (10)        /* Max retries for sending handshake */
#define SIM800_RESPONSE_TIMEOUT_MS  (60*1000)   /* Timeout for UART response */
#define SIM800_LINE_MAX_LEN         (256)       /* Max length of a line */


/******************************************************************************
 * Static variable definition
*******************************************************************************/

static const char* SIM800_TAG = "SIM800";                       /* Tag for logging */
static char sim800_response_buffer[SIM800_UART_BUFFER_SIZE];    /* Response buffer for SIM800 communication */

static sim800_t sim800;                     /* SIM800 module state */
static QueueHandle_t response_queue;        /* Queue for storing responses */
static QueueHandle_t urc_queue;             /* Queue for storing unsolicited notifications */


/******************************************************************************
 * Static function declaration
*******************************************************************************/

int sim800_console_cmd(int argc, char** argv);
void sim800_uart_receive_task(void *pvParameters);
int sim800_get_response(char *buf, size_t len);
int sim800_handle_get_response(char *buf, size_t len);
int sim800_handle_response(void);


/******************************************************************************
 * Function definition
*******************************************************************************/

int sim800_init(uart_port_t uart_num, void* sms_handler) {
    /* return value */
    esp_err_t ret = ESP_OK;

    /* Set SMS handler */
    if (sms_handler != NULL) {
        sim800.sms_handler = sms_handler;
    } else {
        ESP_LOGE(SIM800_TAG, "No SMS handler provided!");
        ret = ESP_FAIL;
    }

    /* Reset module with GPIO */
    /* TODO: implement GPIO logic */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Create queues */
    response_queue = xQueueCreate(5, SIM800_LINE_MAX_LEN);
    urc_queue = xQueueCreate(5, SIM800_LINE_MAX_LEN);

    /* Store UART interface */
    sim800.uart_num = uart_num;
    ESP_LOGI(SIM800_TAG, "SIM800C initialized on UART %d...", sim800.uart_num);

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

    /* Turn off echo */
    if (ret == ESP_OK) {
        sim800_send_cmd(SIM800_CMD_ECHO_OFF);
        ret = sim800_read_response(sim800_response_buffer, SIM800_UART_BUFFER_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(SIM800_TAG, "Failed to turn off echo");
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
    #if 1
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

    uart_write_bytes(sim800.uart_num, cmd, strlen(cmd));
    uart_write_bytes(sim800.uart_num, SIM800_CMD_END, 2);
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
    int bytes_read = uart_read_bytes(sim800.uart_num, (uint8_t *)buf, len, pdMS_TO_TICKS(SIM800_UART_TIMEOUT_MS));
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
 * TODO: needs modification and maybe change to polling mode or free the UART for
 * more time / use mutex for UART (SMS and Signal checks)
 */
void sim800_uart_receive_task(void *pvParameters) {
    char rx_char;
    int line_pos = 0;
    char line[SIM800_LINE_MAX_LEN];
    memset(line, 0, SIM800_LINE_MAX_LEN);

    while (1) {
        int rx_len = uart_read_bytes(sim800.uart_num, &rx_char, 1, pdMS_TO_TICKS(SIM800_UART_TIMEOUT_MS));
        if (rx_len > 0) {
            if (rx_char == '\n') {
                line[line_pos] = '\0';
                if (line_pos > 0) {
                    if (strncmp(line, "+CMTI:", 6) == 0 ||      /* new SMS */
                        strncmp(line, "RING", 4) == 0 ||        /* new Call */
                        strncmp(line, "+CLIP:", 6) == 0) {      /* Caller ID */
                        xQueueSend(urc_queue, line, 0);
                    } else {
                        xQueueSend(response_queue, line, 0);
                    }
                }
                line_pos = 0;
                memset(line, 0, SIM800_LINE_MAX_LEN);
            } else if (rx_char != '\r') {
                if (line_pos < SIM800_LINE_MAX_LEN - 1)
                line[line_pos++] = rx_char;
            }
        }
    }
}


/**
 * @brief Simple check of response 
 * 
 * @return ESP_OK on succes 
 */
int sim800_handle_response(void) {
    return sim800_get_response(NULL, 0);
}

/**
 * @brief Handle and get the response 
 * 
 * @param buf   buffer to store the response
 * @param len   buffer to get the reso
 * 
 * @return ESP_OK on succes 
 */
int sim800_handle_get_response(char *buf, size_t len) {
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    } else {
        return sim800_get_response(buf, len);
    }
}


/**
 * @brief Internal implementation to receive and handle a response
 * 
 * @param buf 
 * @param len 
 * @return int 
 */
int sim800_get_response(char *buf, size_t len) {
    if (buf && len > 0) {
        buf[0] = '\0';
    }

    while(1) {
        char line[SIM800_LINE_MAX_LEN];

        if (xQueueReceive(response_queue, line, pdMS_TO_TICKS(SIM800_RESPONSE_TIMEOUT_MS))) {
            line[SIM800_LINE_MAX_LEN - 1] = '\0';
            ESP_LOGD(SIM800_TAG, "Resp: %s", line);

            if (strstr(line, SIM800_RSP_ERROR)) {
                ESP_LOGE(SIM800_TAG, "Error response: %s", line);
                return ESP_FAIL;

            } else if (strstr(line, SIM800_RSP_CMS_ERROR)) {
                ESP_LOGE(SIM800_TAG, "CMS Error response: %s", line);
                return ESP_FAIL;

            } else if (strstr(line, SIM800_RSP_OK)) {
                return ESP_OK;

            } else if (strstr(line, "+CSQ:")) {
                sscanf(line, "+CSQ: %d,%d", &sim800.rssi, &sim800.ber);
                ESP_LOGI(SIM800_TAG, "Signal quality: RSSI=%d, BER=%d", sim800.rssi, sim800.ber);
                
            } else if (strstr(line, "+CREG:")) {
                int mode;
                int reg_status;
                sscanf(line, "+CREG: %d, %d", &mode, &reg_status);
                sim800.net_status = (sim800_creg_stat_t)reg_status;
                switch (sim800.net_status) {
                    case SIM800_CREG_NOT_REGISTERED:
                        ESP_LOGI(SIM800_TAG, "Not registered in network");
                        break;
                    case SIM800_CREG_REGISTERED_HOME:
                        ESP_LOGI(SIM800_TAG, "Registered in home network");
                        break;
                    case SIM800_CREG_SEARCHING:
                        ESP_LOGI(SIM800_TAG, "Searching network");
                        break;
                    case SIM800_CREG_DENIED:
                        ESP_LOGI(SIM800_TAG, "Network registration denied");
                        break;
                    case SIM800_CREG_UNKOWN:
                        ESP_LOGI(SIM800_TAG, "Unknown network status");
                        break;
                    case SIM800_CREG_REGISTERED_ROAMING:
                        ESP_LOGI(SIM800_TAG, "Registered roaming");
                        break;
                    default:
                        ESP_LOGW(SIM800_TAG, "Unknown registration status: %d", sim800.net_status);
                        break;
                }
            } else {
                ESP_LOGW(SIM800_TAG, "Not handled response: %s", line);
            }

            /* Copy to output buffer */
            if (buf != NULL) {
                strncpy((char *)buf, line, len);
                buf[len - 1] = '\0';
            }
        } else {
            ESP_LOGE(SIM800_TAG, "Response timeout");
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}


/**
 * @brief Console command to interact with SIM800
 * 
 * @param argc : Argument count
 * @param argv : Argument values
 * 
 * @return ESP_OK on success
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
    char line[SIM800_LINE_MAX_LEN];
    if (sim800_get_response(line, SIM800_LINE_MAX_LEN) == ESP_OK) {
        printf("Response:\n%s\n", line);
    }
    return ESP_OK;
}