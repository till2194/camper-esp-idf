idf_component_register(SRCS "mpu6050.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c
                      )
