#ifndef ESP32_DRONE_LOGGER_H
#define ESP32_DRONE_LOGGER_H

#include <Arduino.h>

#define LOGGER_BAUDRATE (115200)
#define LOGGER_BUFFER_SIZE (128)

class Logger
{
public:
    void begin(unsigned long baudRate = LOGGER_BAUDRATE);
    void debug(const char *tag, const char *message, ...);
    void info(const char *tag, const char *message, ...);
    void warning(const char *tag, const char *message, ...);
    void error(const char *tag, const char *message, ...);

protected:
    void log(const char *tag, const char *level, const char *format, va_list args);
};

extern Logger logger;

#endif /* ESP32_DRONE_LOGGER_H */
