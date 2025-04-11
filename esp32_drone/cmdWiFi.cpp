#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <LittleFS.h>

const char* ssid = "ESP32-Drone";
const char* password = "12345678";

WiFiUDP udp;
AsyncWebServer server(80);

void cmdWiFi_init() {
  // Init LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed, formatting...");
    LittleFS.format(); //Formate la partition (supprime les fichiers actuels)
    if (!LittleFS.begin()) {
      Serial.println("Échec du format même après tentative de formatage");
      return;
    }
  }

  // Lancer le point d'accès
  WiFi.softAP(ssid, password);
  Serial.println("WiFi en AP lancé");

  // Serveur de fichiers
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Réception de commande
  server.on("/send", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("msg")) {
      String msg = request->getParam("msg")->value();
      Serial.println("Commande reçue : " + msg);

      // Si tu veux envoyer en UDP vers un module moteur :
      udp.beginPacket("192.168.4.2", 8888); // IP cible
      udp.print(msg);
      udp.endPacket();

      // OU : analyser le msg ici et agir sur les moteurs
    }
    request->send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("Serveur HTTP lancé !");
}