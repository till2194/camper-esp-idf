#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>

#define MPU6050_ADDR0           (0x68)
#define MPU6050_ADDR1           (0x69)

#define MPU6050_REG_WHOAMI      (117)
#define MPU6050_REG_WHOAMI_VAL  (0x68)


void mpu6050_init(uint16_t address);

#endif /* DRV_MPU6050_H */