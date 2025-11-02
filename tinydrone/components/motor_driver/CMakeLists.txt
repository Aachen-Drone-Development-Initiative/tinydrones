idf_component_register(SRCS "motor_driver.c"
  PRIV_REQUIRES esp_driver_ledc
  PRIV_REQUIRES esp_driver_gpio
  INCLUDE_DIRS "include")
