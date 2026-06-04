idf_component_register(
  SRCS "navigation.c"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES mpu6050
  PRIV_REQUIRES esp_driver_i2c
  PRIV_REQUIRES fusion_math
)
