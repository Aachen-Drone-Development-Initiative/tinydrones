idf_component_register(SRCS "remote_comm.c"
  INCLUDE_DIRS "include"
  INCLUDE_DIRS "../../../message_interface"

  # our components

  # esp-idf components
  PRIV_REQUIRES esp_wifi
  PRIV_REQUIRES esp_ringbuf
  PRIV_REQUIRES esp_timer)
