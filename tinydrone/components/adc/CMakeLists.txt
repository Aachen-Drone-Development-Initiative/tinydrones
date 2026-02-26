idf_component_register(SRCS "adc.c"
  INCLUDE_DIRS "include"
  REQUIRES esp_adc
  PRIV_REQUIRES control
  PRIV_REQUIRES esp_netif)