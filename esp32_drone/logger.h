#ifndef ESP32_DRONE_LOGGER_H
#define ESP32_DRONE_LOGGER_H

#include <Arduino.h>

class Logger
{
public:
  void begin(unsigned long baudRate = 115200);
  void info(const char* tag, const char* message, ...);
  void warning(const char* tag, const char* message, ...);
  void error(const char* tag, const char* message, ...);

protected:
  void log(const char* tag, const char* level, const char* format, va_list args);

private:
  const size_t BUFFER_SIZE = 256;
};

extern Logger logger;

#endif /* ESP32_DRONE_LOGGER_H */