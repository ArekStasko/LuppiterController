#include <stdio.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LUPPITER-C-CONTROLLER";
uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void on_data_recv(const uint8_t *mac, const uint8_t *data, int len)
{
    printf("%.*s\n", len, data);
}

void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    printf("Data sent\n");
}

void send_task(void *pvParameter)
{
    while (1)
    {
        const char *msg = "Hello World!";
        esp_err_t result = esp_now_send(broadcast_address, (uint8_t *)msg, strlen(msg));
        if (result != ESP_OK) {
            printf("esp_now_send failed: %s\n", esp_err_to_name(result));
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void init_esp_now()
{
  esp_now_init();
  esp_now_register_send_cb(on_data_sent);
  esp_now_register_recv_cb(on_data_recv);

  esp_now_peer_info_t peer = {
    .channel = 0,
    .ifidx = ESP_IF_WIFI_STA,
    .encrypt = false,
  };

  memcpy(peer.peer_addr, broadcast_address, 6);
  esp_now_add_peer(&peer);

  xTaskCreate(send_task, "send_task", 2048, NULL, 5, NULL);
}

void enable_service()
{
	esp_netif_init();
    esp_event_loop_create_default();
    esp_wifi_init(&(wifi_init_config_t)WIFI_INIT_CONFIG_DEFAULT());
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_start();
    init_esp_now();
}