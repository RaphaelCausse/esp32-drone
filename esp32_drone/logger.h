#ifndef ESP32_DRONE_LOGGER_H
#define ESP32_DRONE_LOGGER_H

#include <Arduino.h>

class Logger
{
public:
    static void begin(unsigned long baudRate = 115200);

    static void info(const char* tag, const char* message, ...);
    static void warning(const char* tag, const char* message, ...);
    static void error(const char* tag, const char* message, ...);

private:
    static void log(const char* tag, const char* level, const char* format, va_list args);

    static const size_t BUFFER_SIZE = 256;
};

#endif /* ESP32_DRONE_LOGGER_H */