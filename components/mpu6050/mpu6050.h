/******************************************************************************
 * Header Guard
*******************************************************************************/
#ifndef _APP_MPU6050_H_
#define _APP_MPU6050_H_


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

#define MPU6050_I2C_ADDR0           (0x68)      /* Primary I2C address */
#define MPU6050_I2C_ADDR1           (0x69)      /* Secondary I2C address */

#define MPU6050_REG_ACCEL_OUT       (0x3B)      /* Accelerometer Measurements */
#define MPU6050_REG_USER_CTRL       (0x6A)      /* User Control */
#define MPU6050_REG_PWR_MGMT_1      (0x6B)      /* Power Management 1 */
#define MPU6050_REG_PWR_MGMT_2      (0x6C)      /* Power Management 2 */
#define MPU6050_REG_WHO_AM_I        (0x75)      /* Who Am I */
#define MPU6050_REG_WHO_AM_I_VAL    (0x68)      /* Who Am I default value */

#define MPU6050_ACC_SENS0           (16384.0)   /* Accelaration sensitivity 0 value in LSB/g */
#define MPU6050_ACC_SENS1           (8192.0)    /* Accelaration sensitivity 1 value in LSB/g */
#define MPU6050_ACC_SENS2           (4096.0)    /* Accelaration sensitivity 2 value in LSB/g */
#define MPU6050_ACC_SENS3           (2048.0)    /* Accelaration sensitivity 3 value in LSB/g */


/******************************************************************************
 * Function declaration
*******************************************************************************/

/**
 * @brief Inits the MPU6050 and check it's ID register. Also resets the
 *        device and set the config register.
 * 
 * 
 * @param[in] address   : I2C address of the device
 */
void mpu6050_init(uint16_t address);


 /**
  * @brief Resets the MPU6050 to restore all registers to default values.
  * 
  * Waits till the device finishes reset routine.
  * 
  */
void mpu6050_reset(void);


/**
 * @brief Wake up device and set to accelerometer only low power mode.
 * 
 * The device will wake up periodically and take a single measurement.
 * 
 */
void mpu6050_config(void);


/**
 * @brief Reads out the latest accelerometer values from the device.
 * 
 * @param[out] acc_x : Pointer to store the x value
 * @param[out] acc_y : Pointer to store the y value 
 * @param[out] acc_z : Pointer to store the z value
 */
void mpu6050_read_acc(double *acc_x, double *acc_y, double *acc_z);


#ifdef __cplusplus
}
#endif /* End of CPP guard */


#endif /* _APP_MPU6050_H_ */