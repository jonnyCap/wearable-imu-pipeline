idf_component_register(SRCS "main.c"
                       "imu_driver.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES driver esp_timer esp_lcd
                       INCLUDE_DIRS "")
