#ifndef ESP32_DRONE_LOGGER_H
#define ESP32_DRONE_LOGGER_H

#include <Arduino.h>

class Logger
{
public:
    void begin(uint32_t baudRate = LOGGER_BAUDRATE);
    void debug(const char *tag, const char *message, ...);
    void info(const char *tag, const char *message, ...);
    void warning(const char *tag, const char *message, ...);
    void error(const char *tag, const char *message, ...);

private:
    void log(const char *tag, const char *level, const char *format, va_list args);

private:
    static constexpr uint32_t LOGGER_BAUDRATE = 115200;
    static constexpr uint32_t LOGGER_BUFFER_SIZE = 128;
};

extern Logger logger;

#endif /* ESP32_DRONE_LOGGER_H */
