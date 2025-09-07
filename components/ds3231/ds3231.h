/******************************************************************************
 * Header Guard
*******************************************************************************/
#ifndef _APP_DS3231_H_
#define _APP_DS3231_H_


/******************************************************************************
 * CPP Guard
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
 * Includes
*******************************************************************************/
#include <time.h>

/******************************************************************************
 * Macros
*******************************************************************************/

#define DS3231_I2C_ADDR                     (0x68)      /* I2C address */

#define DS3231_REG_SECONDS                  (0x00)      /* Seconds register */
#define DS3231_REG_SECONDS_MASK             (0x7F)      /* Seconds mask */

#define DS3231_REG_MINUTES                  (0x01)      /* Minutes register */
#define DS3231_REG_MINUTES_MASK             (0x7F)      /* Minutes mask */

#define DS3231_REG_HOURS                    (0x02)      /* Hours register */
#define DS3231_REG_HOURS_MASK_12H_FLAG      (1 << 5)    /* Flag to set 12h format */
#define DS3231_REG_HOURS_MASK_12H           (0x1F)      /* Hours mask for 12h format */
#define DS3231_REG_HOURS_MASK_24H           (0x3F)      /* Hours mask for 24h format */

#define DS3231_REG_DAY                      (0x03)      /* Day register */
#define DS3231_REG_DAY_MASK                 (0x07)      /* Day mask */

#define DS3231_REG_DATE                     (0x04)      /* Date register */
#define DS3231_REG_DATE_MASK                (0x3F)      /* Date mask */

#define DS3231_REG_MONTH_CENTURY            (0x05)      /* Month/Century register */
#define DS3231_REG_MONTH_MASK               (0x1F)      /* Month mask */
#define DS3231_REG_CENTURY_MASK             (1 << 7)    /* Century mask */

#define DS3231_REG_YEAR                     (0x06)      /* Year register */
#define DS3231_REG_YEAR_MASK                (0xFF)      /* Year mask */

#define DS3231_REG_CONTROL                  (0x0E)      /* Control register */
#define DS3231_REG_CONTROL_MASK_EOSC        (1 << 7)    /* Enable Oscillator */
#define DS3231_REG_CONTROL_MASK_BBSQW       (1 << 6)    /* Battery Backed Square-Wave Enable */
#define DS3231_REG_CONTROL_MASK_CONV        (1 << 5)    /* Convert  Temperature */
#define DS3231_REG_CONTROL_MASK_RS2         (1 << 4)    /* Rate Select 2 */
#define DS3231_REG_CONTROL_MASK_RS1         (1 << 3)    /* Rate Select 1 */
#define DS3231_REG_CONTROL_MASK_INTCN       (1 << 2)    /* Interrupt Control */
#define DS3231_REG_CONTROL_MASK_A2IE        (1 << 1)    /* Alarm 2 Interrupt Enable */
#define DS3231_REG_CONTROL_MASK_A1IE        (1 << 0)    /* Alarm 1 Interrupt Enable */

#define DS3231_REG_STATUS                   (0x0F)      /* Status register */
#define DS3231_REG_STATUS_MASK_OSF          (1 << 7)    /* Oscillator Stop Flag */
#define DS3231_REG_STATUS_MASK_EN32KHZ      (1 << 3)    /* Enable 32kHz output */
#define DS3231_REG_STATUS_MASK_BSY          (1 << 2)    /* Busy Flag */
#define DS3231_REG_STATUS_MASK_A2F          (1 << 1)    /* Alarm 2 Flag */
#define DS3231_REG_STATUS_MASK_A1F          (1 << 0)    /* Alarm 1 Flag */


/******************************************************************************
 * Typedefs
*******************************************************************************/

typedef struct ds3231_time_s {
    uint8_t seconds;   /* Seconds 0-59 */
    uint8_t minutes;   /* Minutes 0-59 */
    uint8_t hours;     /* Hours 0-23 */
    uint8_t day;       /* Day 1-7 */
    uint8_t date;      /* Date 1-31 */
    uint8_t month;     /* Month 1-12 */
    uint16_t year;     /* Year 2000-2199 */
} ds3231_time_t;

/******************************************************************************
 * Function declaration
*******************************************************************************/

/**
 * @brief Inits the DS3231 RTC
 * 
 * @param[in] address : I2C address of the device
 */
void ds3231_init(uint16_t address);


/**
 * @brief Reads the current time from the DS3231 RTC
 * 
 * @param[out] time : Pointer to a ds3231_time_t structure to store the read time
 */
void ds3231_read_time(ds3231_time_t *time);


/**
 * @brief Writes the provided time to the DS3231 RTC
 * 
 * @param[in] time : Pointer to a ds3231_time_t structure containing the time to write
 */
void ds3231_write_time(const ds3231_time_t *time);


/**
 * @brief Converts the time from ds3231_time_t format to struct tm format
 * 
 * @param[in] ds_time  : Pointer to a ds3231_time_t structure containing the time to convert
 * @param[out] tm_time : Pointer to a struct tm structure to store the converted time
 */
void ds3231_convert_dstime_to_tmtime(const ds3231_time_t *ds_time, struct tm *tm_time);


/**
 * @brief Converts the time from struct tm format to ds3231_time_t format
 * 
 * @param[in] tm_time   : Pointer to a struct tm structure containing the time to convert
 * @param[out] ds_time  : Pointer to a ds3231_time_t structure to store the converted time
 */
void ds3231_convert_tmtime_to_dstime(const struct tm *tm_time, ds3231_time_t *ds_time);


#ifdef __cplusplus
}
#endif /* End of CPP guard */


#endif /* _APP_DS3231_H_ */