idf_component_register(SRCS "main.cpp" "motors.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc freertos)
