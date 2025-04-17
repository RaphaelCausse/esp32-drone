#include <esp_now.h>
#include <WiFi.h>

#include "cmdWiFi.h"

// RECEIVER MAC Address
//carte 2 (test)
uint8_t broadcastAddress[] = {0xd8, 0x3b, 0xda, 0xa3, 0x71, 0xec};
//carte 3 (drone)
//uint8_t broadcastAddress[] = {0xd8, 0x3b, 0xda, 0xa3, 0x7b, 0xfc};

esp_now_peer_info_t peerInfo;

// callback appelee quand des donnees sont envoyees
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  Serial.begin(115200);
 
  //Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // recupere le status des packets transmis
  esp_now_register_send_cb(OnDataSent);
  
  // registre peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Ajout peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  cmdWiFi_init();
}
 
void loop() {  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &currentCmd, sizeof(currentCmd));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
   
  delay(10);
}