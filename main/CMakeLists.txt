idf_component_register(SRCS "main.cpp" "components/motor/motors.cpp"
                       INCLUDE_DIRS "." "components/motor/include"
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_ledc freertos)
