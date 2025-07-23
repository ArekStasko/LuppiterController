idf_component_register(SRCS "LuppiterController.c" "./services/CommunicationService/Communication_Service.c" "./services/NvsService/Nvs_Service.c"
                    INCLUDE_DIRS "." "./services/CommunicationService" "./services/NvsService"
                    REQUIRES nvs_flash driver esp_wifi)