idf_component_register(
    SRCS
        "servo_driver.c"
        "drv_oled.c"
        "servo_task.c"
        "uart_receiver.c"
        "at_parser.c"
        "system_commands.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_hw_support nvs_flash
)
