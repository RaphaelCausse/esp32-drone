#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "cmdWiFi.h"

const char* ssid = "ESP32-Drone";
const char* password = "12345678";

WiFiUDP udp;
AsyncWebServer server(80);

struct_command currentCmd;
struct_message currentMsg;

void parseMessage(String msg, struct_message &msg_parsed) {
  StaticJsonDocument<256> doc;

  DeserializationError error = deserializeJson(doc, msg);
  if (error) {
    Serial.print(F("Erreur de parsing JSON : "));
    Serial.println(error.f_str());
    return;
  }

  JsonObject direction = doc["direction"];

  msg_parsed.left_x = direction["yaw"];
  msg_parsed.left_y = direction["throttle"];
  msg_parsed.right_x = direction["roll"];
  msg_parsed.right_y = direction["pitch"];
  strlcpy(msg_parsed.disarmedArmed, doc["disarmedArmed"], sizeof(msg_parsed.disarmedArmed));
  strlcpy(msg_parsed.mode, doc["mode"], sizeof(msg_parsed.mode));

  // Sécurité : s'assurer que les chaînes sont null-terminated
  msg_parsed.disarmedArmed[sizeof(msg_parsed.disarmedArmed) - 1] = '\0';
  msg_parsed.mode[sizeof(msg_parsed.mode) - 1] = '\0';
}

void convert_msg_to_cmd(struct_message &msg_parsed, struct_command &cmd){
  if (strcmp(msg_parsed.mode, "auto-landing") == 0) {
    cmd.switch_mode = MIN;
  } else if (strcmp(msg_parsed.mode, "neutral") == 0) {
    cmd.switch_mode = MIDDLE;
  } else if (strcmp(msg_parsed.mode, "auto-take-off") == 0) {
    cmd.switch_mode = MAX;
  }
  
  if (strcmp(msg_parsed.disarmedArmed, "disarmed") == 0) {
    cmd.channel1 = MIDDLE;
    cmd.channel2 = MIDDLE;
    cmd.channel3 = MIN;
    cmd.channel4 = MIDDLE;
    cmd.switch_disarmed_armed = MIN;
    return;
  } else {
    cmd.switch_disarmed_armed = MAX;
  }

  cmd.channel1 = msg_parsed.right_x;
  cmd.channel2 = msg_parsed.right_y;
  cmd.channel3 = msg_parsed.left_y;
  cmd.channel4 = msg_parsed.left_x;
}

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

      // Parser la commande
      parseMessage(msg, currentMsg);

      Serial.println(String(currentMsg.left_x));
      Serial.println(String(currentMsg.left_y));
      Serial.println(String(currentMsg.right_x));
      Serial.println(String(currentMsg.right_y));
      Serial.println(currentMsg.disarmedArmed);
      Serial.println(currentMsg.mode);

      //Convertion du message recu en commande a envoyer avec ESP-NOW
      convert_msg_to_cmd(currentMsg, currentCmd);

      Serial.println(String(currentCmd.channel1));
      Serial.println(String(currentCmd.channel2));
      Serial.println(String(currentCmd.channel3));
      Serial.println(String(currentCmd.channel4));
      Serial.println(String(currentCmd.switch_disarmed_armed));
      Serial.println(String(currentCmd.switch_mode));

    }
    request->send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("Serveur HTTP lancé !");
}