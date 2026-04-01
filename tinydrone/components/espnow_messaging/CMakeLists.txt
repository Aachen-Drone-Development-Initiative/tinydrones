idf_component_register(SRCS "espnow_messaging.c"
  INCLUDE_DIRS "include"
  INCLUDE_DIRS "../../../message_interface"
  INCLUDE_DIRS "../logging"

  # our components

  # esp-idf components
  PRIV_REQUIRES esp_wifi
  PRIV_REQUIRES esp_ringbuf
  PRIV_REQUIRES esp_timer)
