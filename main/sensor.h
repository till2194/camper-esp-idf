/******************************************************************************
 * Header Guard
*******************************************************************************/
#ifndef _APP_SENSOR_H_
#define _APP_SENSOR_H_


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
 * Typedefs
*******************************************************************************/

/**
 * @brief Possible sensors in the system
 */
typedef enum sensor_type_e {
    SENSOR_BME280,
    SENSOR_MPU6050
} sensor_type_t;


/**
 * @brief Struct to store all relevant sensor data
 */
typedef struct sensor_data_s {
    /* BME280 */
    double temperature;     /* Temperature in °C */
    double pressure;        /* Pressure in Pa */
    double humidity;        /* Humidity in %rH */

    /* MPU6050 */
    double acc_x;           /* x Acceleration in g */
    double acc_y;           /* y Acceleration in g */
    double acc_z;           /* z Acceleration in g */
} sensor_data_t;


/******************************************************************************
 * Function declaration
*******************************************************************************/

/**
 * @brief Inits all sensors defined in sensor_list array.
 */
void sensor_init(void);


/**
 * @brief Periodic task to read all sensors defined in sensor_list array.
 * 
 * @param[in] args : Not used
 */
void sensor_task(void *args);


#ifdef __cplusplus
}
#endif /* End of CPP guard */


#endif /* _APP_SENSOR_H_ */