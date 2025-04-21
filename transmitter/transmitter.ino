#include <esp_now.h>
#include <WiFi.h>
#include "cmdWiFi.h"

uint8_t broadcastAddress[] = {0xd8, 0x3b, 0xda, 0xa3, 0x7b, 0xfc};
esp_now_peer_info_t peerInfo;

// ========== CALLBACK POUR L'ENVOI ==========
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // Optionnel
}

// ========== TACHE 1 : Communication WiFi ==========
void TaskWiFi(void *pvParameters)
{
    cmdWiFi_init();
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Erreur d'initialisation ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Impossible d'ajouter le peer ESP-NOW");
        return;
    }

    // Création des deux tâches
    xTaskCreatePinnedToCore(
        TaskWiFi,    // fonction
        "WiFi Task", // nom
        4096,        // stack size
        NULL,        // params
        1,           // priorité
        NULL,        // handle
        1            // core 1
    );
}

void loop()
{
    esp_now_send(broadcastAddress, (uint8_t *)&currentCmd, sizeof(currentCmd));
    delay(80);
}
