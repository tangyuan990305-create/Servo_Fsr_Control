idf_component_register(
    SRCS "fsr_sensor.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_adc freertos
)
