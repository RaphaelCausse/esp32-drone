#include <stdarg.h>
#include "logger.h"

void Logger::begin(unsigned long baudRate)
{
  Serial.begin(baudRate);
  while (!Serial)
  {
    delay(10);
  }
}

void Logger::info(const char* tag, const char* message, ...)
{
  va_list args;
  va_start(args, message);
  log(tag, "INFO", message, args);
  va_end(args);
}

void Logger::warning(const char* tag, const char* message, ...)
{
  va_list args;
  va_start(args, message);
  log(tag, "WARN", message, args);
  va_end(args);
}

void Logger::error(const char* tag, const char* message, ...)
{
  va_list args;
  va_start(args, message);
  log(tag, "ERR", message, args);
  va_end(args);
}

void Logger::log(const char* tag, const char* level, const char* format, va_list args)
{
  Serial.print("[");
  Serial.print(tag);
  Serial.print("] ");
  Serial.print(level);
  Serial.print(": ");
  char buffer[BUFFER_SIZE];
  vsnprintf(buffer, sizeof(buffer), format, args);
  Serial.println(buffer);
}