idf_component_register(SRCS "main.c"
  INCLUDE_DIRS "."
  INCLUDE_DIRS "../../message_interface"
  # for esp-now
  PRIV_REQUIRES nvs_flash
  PRIV_REQUIRES esp_event
  PRIV_REQUIRES esp_netif
  PRIV_REQUIRES esp_wifi
  PRIV_REQUIRES esp_driver_usb_serial_jtag
  # PRIV_REQUIRES esp_gdbstub
)
