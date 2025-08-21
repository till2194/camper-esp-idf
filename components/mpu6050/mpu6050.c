#include <esp_log.h>
#include <driver/i2c_master.h>

#include "mpu6050.h"

/******************************************************************************
 *                               Macros                                       *
*******************************************************************************/

/* Timeout of the I2C operation */
#define MPU6050_TIMEOUT_MS   (-1)


/******************************************************************************
 *                    Static variable definition                              *
*******************************************************************************/

static const char* MPU6050_TAG = "MPU6050";
static i2c_master_dev_handle_t i2c_dev_handle;


/******************************************************************************
 *                    Static function declaration                              *
*******************************************************************************/

static uint8_t mpu6050_read_reg(uint8_t reg);
static void mpu6050_write_reg(uint8_t reg, uint8_t val);


/******************************************************************************
 *                        Function definition                              *
*******************************************************************************/

/**
 * Inits the MPU6050 and check it's ID register.
 * Also resets the device and set the config register.
 * 
 * @param address   I2C address of the device
 */
void mpu6050_init(uint16_t address) {
    i2c_master_bus_handle_t master_handle;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &master_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_handle, &dev_cfg, &i2c_dev_handle));

    /* Check Who am I register */
    uint8_t val = mpu6050_read_reg(MPU6050_REG_WHO_AM_I);
    ESP_LOGD(MPU6050_TAG, "WHO AM I: %u", val);
    if (val != MPU6050_REG_WHO_AM_I_VAL) {
        ESP_LOGE(MPU6050_TAG, "Register WHO AM I has wrong value 0x%X!", val);
    }

    /* Reset device and all registers */
    mpu6050_reset();

    /* Configures the device */
    mpu6050_config();
}


/**
 * Resets the MPU6050 to restore all registers to default values.
 * Waits till the device finishes reset routine.
 */
void mpu6050_reset(void) {
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80);
    uint8_t val = 0x00;

    /* Compare register and reset bit */
    while ((val & 0x80) != 0) {
        val = mpu6050_read_reg(MPU6050_REG_PWR_MGMT_1);
    }
}


/**
 * Wake up device and set to accelerometer only low power mode.
 * The device will wake up periodically and take a single measurement.
 */
void mpu6050_config(void) {
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x28);
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_2, 0x47);
}


/**
 * Reads out the latest accelerometer values from the device.
 * 
 * @param acc_x Pointer to store the x value
 * @param acc_y Pointer to store the y value
 * @param acc_z Pointer to store the z value
 */
void mpu6050_read_acc(double *acc_x, double *acc_y, double *acc_z) {
    uint8_t reg = MPU6050_REG_ACCEL_OUT;
    uint8_t buf[6];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev_handle, &reg, 1, &buf[0], 6, MPU6050_TIMEOUT_MS));

    /* Translate to 16 bit values */
    int16_t val_raw_x = (int16_t) (buf[0] << 8) | buf[1];
    int16_t val_raw_y = (int16_t) (buf[2] << 8) | buf[3];
    int16_t val_raw_z = (int16_t) (buf[4] << 8) | buf[5];
    
    /* Translate to doubles in g unit */
    *acc_x = (double) val_raw_x / MPU6050_ACC_SENS0;
    *acc_y = (double) val_raw_y / MPU6050_ACC_SENS0;
    *acc_z = (double) val_raw_z / MPU6050_ACC_SENS0;
}


/******************************************************************************
 *                    Static function definition                              *
*******************************************************************************/

/**
 * Reads a 8 bit register from the MPU6050
 * 
 * @param reg Register address to read from
 * 
 * @return Returns the 8 bit value of the register
 */
static uint8_t mpu6050_read_reg(uint8_t reg) {
    uint8_t val;
    ESP_LOGD(MPU6050_TAG, "Reading register 0x%X", reg);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev_handle, &reg, 1, &val, 1, MPU6050_TIMEOUT_MS));
    return val;
}

/**
 * Writes a 8 bit value to a register of the MPU6050
 * 
 * @param reg Register address to write to
 * @param val Value to write to the register
 */
static void mpu6050_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[] = {reg, val};
    ESP_LOGD(MPU6050_TAG, "Writing register 0x%X with value 0x%X", reg, val);
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &buf[0], 2, MPU6050_TIMEOUT_MS));
}