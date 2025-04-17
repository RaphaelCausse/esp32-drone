#ifndef CMD_WIFI_H
#define CMD_WIFI_H

#define MIN (1000)
#define MIDDLE (1500)
#define MAX (2000)

struct struct_command {
  uint16_t channel1; //roll
  uint16_t channel2; //pitch
  uint16_t channel3; //throttle
  uint16_t channel4; //yaw
  uint16_t switch_disarmed_armed;
  uint16_t switch_mode;
};

struct struct_message {
  uint16_t left_x;
  uint16_t left_y;
  uint16_t right_x;
  uint16_t right_y;
  char disarmedArmed[32];
  char mode[32];
};

extern struct_command currentCmd;
extern struct_message currentMsg;

void cmdWiFi_init();

#endif