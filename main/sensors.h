#ifndef APP_SENSORS_H
#define APP_SENSORS_H

typedef enum {
    SENSOR_BME280,
    SENSOR_MPU6050
} sensor_type_t;

void sensors_init(void);
void sensors_task(void *args);

#endif /* APP_SENSORS_H */