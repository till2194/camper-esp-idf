/******************************************************************************
 * Includes
*******************************************************************************/
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <time.h>

#include "ds3231.h"

/******************************************************************************
 * Macros
*******************************************************************************/

#define DS3231_TIMEOUT_MS   (10)    /* Timeout of the I2C operation */


/******************************************************************************
 * Static variable definition
*******************************************************************************/

static const char* DS3231_TAG = "DS3231";
static i2c_master_dev_handle_t i2c_dev_handle;


/******************************************************************************
 * Static function declaration
*******************************************************************************/

static uint8_t ds3231_read_reg(uint8_t reg);
static void ds3231_write_reg(uint8_t reg, uint8_t val);
uint8_t dec_to_bcd(uint8_t val);
uint8_t bcd_to_dec(uint8_t val);
int ds3231_is_dst(struct tm *t);
void ds3231_print_time(const ds3231_time_t *time);


/******************************************************************************
 * Function definition
*******************************************************************************/

void ds3231_init(uint16_t address) {
    i2c_master_bus_handle_t master_handle;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &master_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_handle, &dev_cfg, &i2c_dev_handle));

    /* Check Status register to verify connection and state */
    uint8_t status = ds3231_read_reg(DS3231_REG_STATUS);
    ESP_LOGI(DS3231_TAG, "Status register: 0x%02X", status);
    if (status & DS3231_REG_STATUS_MASK_EN32KHZ) {
        ESP_LOGI(DS3231_TAG, "32kHz output is enabled. DS3231 communication seems OK.");
    } else {
        ESP_LOGI(DS3231_TAG, "32kHz output is disabled.");
    }

    /* Clear OSF flag if needed */
    if (status & DS3231_REG_STATUS_MASK_OSF) {
        ESP_LOGW(DS3231_TAG, "Oscillator Stop Flag (OSF) is set. Clearing it.");
        
        uint8_t new_status = status & ~(DS3231_REG_STATUS_MASK_OSF);
        ds3231_write_reg(DS3231_REG_STATUS, new_status);

        status = ds3231_read_reg(DS3231_REG_STATUS);
        if (status & DS3231_REG_STATUS_MASK_OSF) {
            ESP_LOGE(DS3231_TAG, "Failed to clear OSF bit. Check DS3231 connection or backup battery.");
        } else {
            ESP_LOGI(DS3231_TAG, "OSF bit cleared. DS3231 communication is OK.");
        }
    } else {
        ESP_LOGI(DS3231_TAG, "Oscillator is running. DS3231 communication seems OK.");
    }

    /* Check Hour register and set 24h mode if needed */
    uint8_t hours = ds3231_read_reg(DS3231_REG_HOURS);
    if (hours & DS3231_REG_HOURS_MASK_12H_FLAG) {
        ESP_LOGI(DS3231_TAG, "Device is in 12h mode. Switching to 24h mode.");
        hours &= ~DS3231_REG_HOURS_MASK_12H_FLAG;
        ds3231_write_reg(DS3231_REG_HOURS, hours);
    } else {
        ESP_LOGI(DS3231_TAG, "Device is in 24h mode.");
    }

    /* Read the current time */
    ds3231_time_t current_time;
    ds3231_read_time(&current_time);

    /* Print the current time */ 
    ds3231_print_time(&current_time);
}


void ds3231_read_time(ds3231_time_t *time) {
    /* Null pointer check */
    if (time == NULL) {
        ESP_LOGE(DS3231_TAG, "Null pointer provided for time structure.");
        return;
    }

    uint8_t reg = DS3231_REG_SECONDS;
    uint8_t buf[7];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev_handle, &reg, 1, buf, 7, DS3231_TIMEOUT_MS));

    time->seconds = bcd_to_dec(buf[0] & DS3231_REG_SECONDS_MASK);
    time->minutes = bcd_to_dec(buf[1] & DS3231_REG_MINUTES_MASK);

    if  (buf[2] & DS3231_REG_HOURS_MASK_12H_FLAG) {
        /* 12h format */
        time->hours = bcd_to_dec(buf[2] & DS3231_REG_HOURS_MASK_12H);
        if (buf[2] & 0x20) {
            /* PM case */
            if (time->hours != 12) {
                time->hours += 12;
            }
        } else {
            /* AM case */
            if (time->hours == 12) {
                time->hours = 0; // Midnight case
            }
        }
    } else {
        /* 24h format */
        time->hours = bcd_to_dec(buf[2] & DS3231_REG_HOURS_MASK_24H);
    }

    time->day   = buf[3] & DS3231_REG_DAY_MASK;
    time->date  = bcd_to_dec(buf[4] & DS3231_REG_DATE_MASK);
    time->month = bcd_to_dec(buf[5] & DS3231_REG_MONTH_MASK);

    if (buf[5] & DS3231_REG_CENTURY_MASK) {
        /* Century bit is set, so we are in 2100-2199 */
        time->year = 2100 + bcd_to_dec(buf[6] & DS3231_REG_YEAR_MASK);
    } else {
        /* Century bit is clear, so we are in 2000-2099 */
        time->year = 2000 + bcd_to_dec(buf[6] & DS3231_REG_YEAR_MASK);
    }
}


void ds3231_write_time(const ds3231_time_t *time) {
    /* Null pointer check */
    if (time == NULL) {
        ESP_LOGE(DS3231_TAG, "Null pointer provided for time structure.");
        return;
    }

    uint8_t buf[8];
    buf[0] = DS3231_REG_SECONDS;

    buf[1] = dec_to_bcd(time->seconds) & DS3231_REG_SECONDS_MASK;
    buf[2] = dec_to_bcd(time->minutes) & DS3231_REG_MINUTES_MASK;
    buf[3] = dec_to_bcd(time->hours) & DS3231_REG_HOURS_MASK_24H;   /* Ensure 24h format */
    buf[4] = time->day & DS3231_REG_DAY_MASK;
    buf[5] = dec_to_bcd(time->date) & DS3231_REG_DATE_MASK;
    buf[6] = dec_to_bcd(time->month) & DS3231_REG_MONTH_MASK;

    if (time->year >= 2100) {
        /* Set century bit for 2100-2199 */
        buf[5] |= DS3231_REG_CENTURY_MASK;
        buf[6] = dec_to_bcd(time->year - 2100) & DS3231_REG_YEAR_MASK;
    } else {
        /* Clear century bit for 2000-2099 */
        buf[5] &= ~DS3231_REG_CENTURY_MASK;
        buf[6] = dec_to_bcd(time->year - 2000) & DS3231_REG_YEAR_MASK;
    }

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &buf[0], 8, DS3231_TIMEOUT_MS));
    ESP_LOGI(DS3231_TAG, "Time updated successfully.");
}


void ds3231_convert_dstime_to_tmtime(const ds3231_time_t *ds_time, struct tm *tm_time) {
    /* Null pointer check */
    if (ds_time == NULL || tm_time == NULL) {
        ESP_LOGE(DS3231_TAG, "Null pointer provided for time structure.");
        return;
    }

    tm_time->tm_sec  = ds_time->seconds;            /* Seconds [0, 60] */
    tm_time->tm_min  = ds_time->minutes;            /* Minutes [0, 59] */
    tm_time->tm_hour = ds_time->hours;              /* Hours [0, 23] */
    tm_time->tm_mday = ds_time->date;               /* Day of the month [1, 31] */
    tm_time->tm_mon  = ds_time->month - 1;          /* Month since January [0, 11] */
    tm_time->tm_year = ds_time->year - 1900;        /* Years since 1900 */
    tm_time->tm_wday = (ds_time->day - 1) % 7;      /* Day of the week [0, 6] (0 = Sunday) */
    tm_time->tm_isdst = ds3231_is_dst(tm_time);     /* Daylight Saving Time flag */
}


void ds3231_convert_tmtime_to_dstime(const struct tm *tm_time, ds3231_time_t *ds_time) {
    /* Null pointer check */
    if (ds_time == NULL || tm_time == NULL) {
        ESP_LOGE(DS3231_TAG, "Null pointer provided for time structure.");
        return;
    }

    ds_time->seconds = tm_time->tm_sec;             /* Seconds 0-59 */
    ds_time->minutes = tm_time->tm_min;             /* Minutes 0-59 */
    ds_time->hours   = tm_time->tm_hour;            /* Hours 0-23 */
    ds_time->date    = tm_time->tm_mday;            /* Date 1-31 */
    ds_time->month   = tm_time->tm_mon + 1;         /* Month 1-12 */
    ds_time->year    = tm_time->tm_year + 1900;     /* Year 2000-2199 */
    ds_time->day     = (tm_time->tm_wday + 1) % 7;  /* Day 1-7 (1 = Monday, 7 = Sunday) */
}


/******************************************************************************
 * Static function definition
*******************************************************************************/

/**
 * @brief Reads a 8 bit register from the DS3231
 * 
 * @param[in] reg   : Register address to read from
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
 * @brief Writes a 8 bit value to a register of the DS3231
 * 
 * @param[in] reg   : Register address to write to
 * @param[in] val   : Value to write to the register
 */
static void ds3231_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[] = {reg, val};
    ESP_LOGD(DS3231_TAG, "Writing register 0x%X with value 0x%X", reg, val);
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, &buf[0], 2, DS3231_TIMEOUT_MS));
}


/**
 * @brief Convert a BCD (Binary-Coded Decimal) value to binary
 * 
 * @param[in] val   : Value to convert
 * 
 * @return Binary representation of the value
 */
uint8_t bcd_to_dec(uint8_t val) {
    return (val & 0x0F) + ((val >> 4) * 10);
}


/**
 * @brief Convert a binary value to BCD (Binary-Coded Decimal)
 * 
 * @param[in] val   : Value to convert
 * 
 * @return BCD representation of the value 
 */
uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}


/**
 * @brief Determines if the given time is in Daylight Saving Time (DST)
 * 
 * @param[in] t : Pointer to a struct tm structure containing the time to check
 * 
 * @retval Returns 1 if the time is in DST
 * @retval Returns 0 if not in DST
 */
int ds3231_is_dst(struct tm *t) {
    /* Null pointer check */
    if (t == NULL) {
        ESP_LOGE(DS3231_TAG, "Null pointer provided for time structure.");
        return 0;
    }

    /* November to February = Wintertime */
    if (t->tm_mon < 2 || t->tm_mon > 9) {
        return 0; 
    }

    /* April to September = Summertime */
    if (t->tm_mon > 2 && t->tm_mon < 9) {
        return 1;
    }

    /* March: Summertime from last Sunday 2:00 */
    if (t->tm_mon == 2) {
        int last_sunday = 31 - ((5 + (t->tm_year + 1900) + ((t->tm_year + 1900)/4)) % 7);
        if (t->tm_mday > last_sunday) {
            return 1;
        }

        if (t->tm_mday == last_sunday && t->tm_hour >= 2){
            return 1;
        }

        return 0;
    }

    /* October: Wintertime from last Sunday 3:00 */
    if (t->tm_mon == 9) {
        int last_sunday = 31 - ((2 + (t->tm_year + 1900) + ((t->tm_year + 1900)/4)) % 7);
        if (t->tm_mday < last_sunday) return 1;
        if (t->tm_mday == last_sunday && t->tm_hour < 3) return 1;
        return 0;
    }

    return 0;
}


/**
 * @brief Prints the time stored in a ds3231_time_t structure to the log
 * 
 * @param[in] time  : Pointer to a ds3231_time_t structure containing the time to print
 */
void ds3231_print_time(const ds3231_time_t *time) {
    if (time == NULL) {
        ESP_LOGE(DS3231_TAG, "Null pointer provided for time structure.");
        return;
    }

    ESP_LOGI(DS3231_TAG, "Time: %02u:%02u:%02u %02u/%02u/%04u (Day of week: %u)",
             time->hours, time->minutes, time->seconds,
             time->date, time->month, time->year,
             time->day);
}