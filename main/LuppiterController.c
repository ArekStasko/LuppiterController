#include <stdio.h>
#include "Nvs_Service.h"
#include "Communication_Service.h"
#include "Barometric_Pressure_Service.h"

void app_main(void)
{
	enable_nvs_service();
	init_barometric_pressure_service();
    enable_service();
}
