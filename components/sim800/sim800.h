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


/******************************************************************************
 * Function declaration
*******************************************************************************/

/**
 * @brief Inits the SIM800 module
 */
int sim800_init(uart_port_t uart_num);

/**
 * @brief Sends an AT command to the SIM800C module
 * 
 * @param cmd   : Command string to send
 */
void sim800_send_cmd(const char *cmd);


/**
 * @brief Reads the response from the SIM800C module
 * 
 * @param response : Buffer to store the response
 */
void sim800_read_response(char *response);


#ifdef __cplusplus
}
#endif /* End of CPP guard */


#endif /* _APP_SIM800_H_ */