/******************************************************************************
 * Header Guard
*******************************************************************************/
#ifndef _APP_SIM800_H_
#define _APP_SIM800_H_


/******************************************************************************
 * CPP Guard
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
 * Includes
*******************************************************************************/


/******************************************************************************
 * Macros
*******************************************************************************/

#define SIM800_CMD_END              ("\r\n")        /* AT command ending */
#define SIM800_CMD_CHECK            ("AT")          /* Basic AT command to check communication */
#define SIM800_RSP_OK               ("OK")          /* OK response */
#define SIM800_RSP_ERROR            ("ERROR")       /* ERROR response */
#define SIM800_RSP_CMS_ERROR        ("+CMS ERROR")  /* CMS ERROR response */

#define SIM800_CMD_ECHO_OFF        ("ATE0")         /* Turn off echo */
#define SIM800_CMD_ECHO_ON         ("ATE1")         /* Turn on echo */
#define SIM800_CMD_SIGNAL_TEST     ("AT+CSQ")       /* Signal quality test, value range is 0-31 , 31 is the best */
#define SIM800_CMD_INFO_SIM        ("AT+CCID")      /* Read SIM information to confirm whether the SIM is plugged */
#define SIM800_CMD_INFO_NETWORK    ("AT+CREG?")     /* Check whether it has registered in the network */
#define SIM800_CMD_INFO_OPERATOR   ("AT+COPS?")     /* Checks which network you are connected to */
#define SIM800_CMD_OPERATORS       ("AT+COPS=?")    /* Returns the list of operators present in the network */
#define SIM800_CMD_BATTERY         ("AT+CBC")       /* Returns Li-Po battery status */

#define SIM800_CMD_SMS_TEXT_MODE   ("AT+CMGF=1")                   /* Configure SMS Text Mode */
#define SIM800_CMD_SMS_TX_CONFIG   ("AT+CNMI=1,2,0,0,0")           /* Decides how newly arrived SMS messages should be handled */
#define SIM800_CMD_SMS_TX_START    ("AT+CMGS=\"+4915237605881\"")  /* Config SMS number */
#define SIM800_CMD_SMS_TX_STOP     (0x1A)                          /* Ctrl+z character to end text input and send SMS */


/******************************************************************************
 * Typedefs
*******************************************************************************/

/**
 * @brief Status <stat> of Network Registration CREG
 * 
 */
typedef enum sim800_creg_stat_e {
    SIM800_CREG_NOT_REGISTERED      = 0,
    SIM800_CREG_REGISTERED_HOME     = 1,
    SIM800_CREG_SEARCHING           = 2,
    SIM800_CREG_DENIED              = 3,
    SIM800_CREG_UNKOWN              = 4,
    SIM800_CREG_REGISTERED_ROAMING  = 5
} sim800_creg_stat_t;


/**
 * @brief 
 * 
 */
typedef struct sim800_s {
    uart_port_t uart_num;
    sim800_creg_stat_t net_status;
    int rssi;
    int ber;
    void (*sms_handler)(const char *sms);
} sim800_t;


/******************************************************************************
 * Function declaration
*******************************************************************************/

/**
 * @brief Inits the SIM800 module
 */
int sim800_init(uart_port_t uart_num, void* sms_handler);

/**
 * @brief Sends an AT command to the SIM800C module
 * 
 * @param cmd   : Command string to send
 */
void sim800_send_cmd(const char *cmd);


 /**
  * @brief Reads the response from the SIM800C module
  * 
  * @param buf  : Buffer to store the response
  * @param len  : Length of the buffer
  * 
  * @return ESP_OK on succes, false on error
  */
int sim800_read_response(char *buf, size_t len);


/**
 * @brief Get signal quality from SIM800C module
 * 
 * @param rssi : Pointer to store the RSSI value (0-31)
 * @param ber  : Pointer to store the Bit Error Rate (0-7, 99=not known or not detectable)
 * 
 * @return ESP_OK on succes, false on error
 * 
 */
int sim800_get_signal_quality(uint8_t *rssi, uint8_t *ber);


#ifdef __cplusplus
}
#endif /* End of CPP guard */


#endif /* _APP_SIM800_H_ */