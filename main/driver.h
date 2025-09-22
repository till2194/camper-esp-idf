/******************************************************************************
 * Header Guard
*******************************************************************************/
#ifndef _APP_DRIVER_H_
#define _APP_DRIVER_H_


/******************************************************************************
 * CPP Guard
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
 * Includes
*******************************************************************************/
#include <driver/uart.h>
#include <driver/gpio.h>

/******************************************************************************
 * Macros
*******************************************************************************/

#define DRIVER_UART_NUM             (UART_NUM_2)
#define DRIVER_UART_TX_PIN          (GPIO_NUM_17)
#define DRIVER_UART_RX_PIN          (GPIO_NUM_16)
#define DRIVER_UART_BUFFER_SIZE     (2048)
#define DRIVER_UART_BAUDRATE        (115200)

#define DRIVER_GPO_LED              (GPIO_NUM_2)
#define DRIVER_GPO_HEATER           (GPIO_NUM_23)
#define DRIVER_GPO_LED1             (GPIO_NUM_25)
#define DRIVER_GPO_LED2             (GPIO_NUM_26)

#define DRIVER_GPI_SW1              (GPIO_NUM_19)
#define DRIVER_GPI_SW2              (GPIO_NUM_18)

/******************************************************************************
 * Typedefs
*******************************************************************************/


/******************************************************************************
 * Function declaration
*******************************************************************************/

/**
 * @brief Inits the drivers (I2C, SPI, GPIO, etc.)
 * 
 */
void driver_init(void);


#ifdef __cplusplus
}
#endif /* End of CPP guard */


#endif /* _APP_DRIVER_H_ */