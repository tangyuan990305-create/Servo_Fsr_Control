#ifndef SYSTEM_COMMANDS_H
#define SYSTEM_COMMANDS_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t system_commands_init(void);
bool system_command_handle(const char *line);

#endif
