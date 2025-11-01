idf_component_register(SRCS "main.c"
  INCLUDE_DIRS "."
  
  ## our components
  PRIV_REQUIRES MPU6050
  PRIV_REQUIRES controller_communication

  ## esp-idf components
  PRIV_REQUIRES esp_driver_usb_serial_jtag
  PRIV_REQUIRES esp_driver_i2c
  PRIV_REQUIRES esp_driver_gpio
  # for esp-now
  PRIV_REQUIRES nvs_flash
  PRIV_REQUIRES esp_event
  PRIV_REQUIRES esp_netif
  PRIV_REQUIRES esp_wifi
)

