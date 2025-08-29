#include <esp_log.h>
#include <driver/i2c_master.h>

#include "ds3231.h"

/******************************************************************************
 *                               Macros                                       *
*******************************************************************************/

/* Timeout of the I2C operation */
#define DS3231_TIMEOUT_MS   (-1)


/******************************************************************************
 *                    Static variable definition                              *
*******************************************************************************/

static const char* DS3231_TAG = "DS3231";
static i2c_master_dev_handle_t i2c_dev_handle;


/******************************************************************************
 *                    Static function declaration                              *
*******************************************************************************/

static uint8_t ds3231_read_reg(uint8_t reg);
static void ds3231_write_reg(uint8_t reg, uint8_t val);


/******************************************************************************
 *                        Function definition                              *
*******************************************************************************/
void ds3231_init(uint16_t address) {
    
}

/******************************************************************************
 *                    Static function definition                              *
*******************************************************************************/

/**
 * Reads a 8 bit register from the DS3231
 * 
 * @param reg Register address to read from
 * 
 * @return Returns the 8 bit value of the register
 */
static uint8_t ds3231_read_reg(uint8_t reg) {
    uint8_t val;
    ESP_LOGD(DS3231_TAG, "Reading register 0x%X", reg);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev_handle, &reg, 1, &val, 1, DS3231_TIMEOUT_MS));
    return val;
}

/**
 * Writes a 8 bit value to a register of the DS3231
 * 
 * @param reg Register address to write to
 * @param val Value to write to the register
 */
static void ds3231_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[] = {reg, val};
    ESP_LOGD(DS3231_TAG, "Writing register 0x%X with value 0x%X", reg, val);
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &buf[0], 2, DS3231_TIMEOUT_MS));
}