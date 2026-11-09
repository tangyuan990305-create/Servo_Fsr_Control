idf_component_register(
    SRCS
        "servo_driver.c"
        "drv_oled.c"
        "servo_task.c"
        "fsr_monitor.c"
        "uart_receiver.c"
        "at_parser.c"
        "system_commands.c"
        #2026.10.09 新增：压力控制模块
        "press_control.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_hw_support nvs_flash fsr_sensor
)
