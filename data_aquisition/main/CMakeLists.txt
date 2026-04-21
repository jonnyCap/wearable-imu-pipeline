idf_component_register(SRCS "main.c" "imu_driver.c"
                       PRIV_REQUIRES spi_flash esp_wifi esp_event nvs_flash lwip esp_driver_gpio esp_driver_i2c
                       INCLUDE_DIRS "")
