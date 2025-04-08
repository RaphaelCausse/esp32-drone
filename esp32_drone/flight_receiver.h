#ifndef ESP32_DRONE_FLIGHT_RECEIVER_H
#define ESP32_DRONE_FLIGHT_RECEIVER_H

#include <Arduino.h>
// #include <IBusBM.h> // Compilation errors
#include <HardwareSerial.h>

#define IBUS_UART_RX (6) // GPIO pin for UART RX
#define IBUS_UART_TX (7) // GPIO pin for UART TX

class FlightReceiver
{
public:
    FlightReceiver();
    ~FlightReceiver();

    void init(int uart_rx = IBUS_UART_RX, int uart_tx = IBUS_UART_TX);
    void update();

private:
    // IBusBM m_ibus;
};

#endif /* ESP32_DRONE_FLIGHT_RECEIVER_H */
