#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

void enable_nvs_service(void)
{
    nvs_flash_init();
}
