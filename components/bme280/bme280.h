#ifndef BME280_H
#define BME280_H

#define BME280_ADDR0    (0x76)
#define BME280_ADDR1    (0x77)

/* Registers */
#define BME280_REG_ID       (0xD0)
#define BME280_REG_ID_VAL   (0x60)

typedef int64_t BME280_S64_t;

void bme280_init(uint16_t address);

#endif /* BME280_H */