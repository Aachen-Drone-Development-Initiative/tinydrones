idf_component_register(SRCS "espnow_messaging.c"
  INCLUDE_DIRS "include"
  INCLUDE_DIRS "../../../message_interface"

  # our components
  PRIV_REQUIRES logging

  # esp-idf components
  PRIV_REQUIRES esp_wifi
  PRIV_REQUIRES esp_ringbuf)
