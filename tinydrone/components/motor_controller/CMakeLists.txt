idf_component_register(SRCS "motor_controller.c"
                    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio
                    INCLUDE_DIRS "include")
