#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>

#define MPU6050_ADDR0           (0x68)
#define MPU6050_ADDR1           (0x69)

#define MPU6050_REG_ACCEL_OUT       (0x3B)
#define MPU6050_REG_USER_CTRL       (0x6A)
#define MPU6050_REG_PWR_MGMT_1      (0x6B)
#define MPU6050_REG_PWR_MGMT_2      (0x6C)
#define MPU6050_REG_WHO_AM_I        (0x75)
#define MPU6050_REG_WHO_AM_I_VAL    (0x68)

#define MPU6050_ACC_SENS0           (16384.0) /* LSB/g */
#define MPU6050_ACC_SENS1           (8192.0) /* LSB/g */
#define MPU6050_ACC_SENS2           (4096.0) /* LSB/g */
#define MPU6050_ACC_SENS3           (2048.0) /* LSB/g */

void mpu6050_init(uint16_t address);
void mpu6050_reset(void);
void mpu6050_config(void);
void mpu6050_read_acc(double *acc_x, double *acc_y, double *acc_z);

#endif /* DRV_MPU6050_H */