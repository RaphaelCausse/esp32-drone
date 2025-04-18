#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "cmdWiFi.h"

const char *ssid = "ESP32-Drone";
const char *password = "12345678";

WiFiUDP udp;
AsyncWebServer server(80);

struct_command currentCmd;
struct_message currentMsg;

void parseMessage(String msg, struct_message &msg_parsed)
{
  StaticJsonDocument<256> doc;

  DeserializationError error = deserializeJson(doc, msg);
  if (error)
  {
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

void convert_msg_to_cmd(struct_message &msg_parsed, struct_command &cmd)
{
  if (strcmp(msg_parsed.mode, "auto-landing") == 0)
  {
    cmd.switch_mode = MIN;
  }
  else if (strcmp(msg_parsed.mode, "neutral") == 0)
  {
    cmd.switch_mode = MIDDLE;
  }
  else if (strcmp(msg_parsed.mode, "auto-take-off") == 0)
  {
    cmd.switch_mode = MAX;
  }

  if (strcmp(msg_parsed.disarmedArmed, "disarmed") == 0)
  {
    cmd.switch_arm = MIN;
  }
  else if (strcmp(msg_parsed.disarmedArmed, "armed") == 0)
  {
    cmd.switch_arm = MAX;
  }

  cmd.roll = msg_parsed.right_x;
  cmd.pitch = msg_parsed.right_y;
  cmd.throttle = msg_parsed.left_y;
  cmd.yaw = msg_parsed.left_x;
}

void cmdWiFi_init()
{
  // Init LittleFS
  if (!LittleFS.begin())
  {
    Serial.println("LittleFS mount failed, formatting...");
    LittleFS.format(); // Formate la partition (supprime les fichiers actuels)
    if (!LittleFS.begin())
    {
      Serial.println("Échec du format même après tentative de formatage");
      return;
    }
  }

  // Lancer le point d'accès
  WiFi.softAP(ssid, password);
  Serial.println("WiFi en AP lancé");

  // Affiche l'adresse IP pour accéder à la page web
  Serial.print("Connectez-vous à : ");
  Serial.println(WiFi.softAPSSID());

  // Serveur de fichiers
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Réception de commande
  server.on("/send", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (request->hasParam("msg")) {
      String msg = request->getParam("msg")->value();
      // Serial.println("===== Commande reçue : " + msg + "=====");

      // Parser la commande
      parseMessage(msg, currentMsg);

      //Convertion du message recu en commande a envoyer avec ESP-NOW
      convert_msg_to_cmd(currentMsg, currentCmd);

      Serial.print("roll: ");
      Serial.print(String(currentCmd.roll));
      Serial.print("\t| pitch: ");
      Serial.print(String(currentCmd.pitch));
      Serial.print("\t| throttle: ");
      Serial.print(String(currentCmd.throttle));
      Serial.print("\t| yaw: ");
      Serial.print(String(currentCmd.yaw));
      
      Serial.print("\t| switch_arm: ");
      if (currentCmd.switch_arm == MIN)
        Serial.print("disarmed");
      else if (currentCmd.switch_arm == MAX)
        Serial.print("armed");

      Serial.print("\t| switch_mode: ");
      if (currentCmd.switch_mode == MIN)
        Serial.println("auto_landing");
      else if (currentCmd.switch_mode == MIDDLE)
        Serial.println("neutral");
      else if (currentCmd.switch_mode == MAX)
        Serial.println("auto_takeoff");
    }
    request->send(200, "text/plain", "OK"); });

  server.begin();
  Serial.println("Serveur HTTP lancé !");
  Serial.print("Controle du drone : http://");
  Serial.println(WiFi.softAPIP());
}