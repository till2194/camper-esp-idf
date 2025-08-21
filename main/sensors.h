#ifndef APP_SENSORS_H
#define APP_SENSORS_H

/**
 * Possible sensors in the system
 */
typedef enum sensor_type_e {
    SENSOR_BME280,
    SENSOR_MPU6050
} sensor_type_t;


/**
 * Struct to store all relevant sensor data
 */
typedef struct sensor_data_s {
    /* BME280 */
    double temperature;     /* Temperature in °C */
    double pressure;        /* Pressure in Pa */
    double humidity;        /* Humidity in %rH */

    /* MPU6050 */
    double acc_x;           /* Acceleration in x in g */
    double acc_y;           /* Acceleration in y in g */
    double acc_z;           /* Acceleration in z in g */
} sensor_data_t;

void sensors_init(void);
void sensors_task(void *args);

#endif /* APP_SENSORS_H */