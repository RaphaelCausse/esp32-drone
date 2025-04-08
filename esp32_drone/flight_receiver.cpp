#include "flight_receiver.h"

static const char *TAG = "FLIGHT_RCVR";

HardwareSerial IBUS_SERIAL(2);

FlightReceiver::FlightReceiver() {}

FlightReceiver::~FlightReceiver() {}

void FlightReceiver::init(int uart_rx, int uart_tx)
{
    IBUS_SERIAL.begin(115200, SERIAL_8N1, uart_rx, uart_tx);
    // m_ibus.begin(IBUS_SERIAL);
}

void FlightReceiver::update()
{
}
