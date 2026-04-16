idf_component_register(SRCS "mqtt_setup.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES nvs_flash
    PRIV_REQUIRES esp_event
    PRIV_REQUIRES esp_netif
    PRIV_REQUIRES esp_wifi
    PRIV_REQUIRES mqtt
)
