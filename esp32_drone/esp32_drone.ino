#include "drone.h"
#include "logger.h"

Drone drone;

void setup()
{
    logger.begin();
    drone.init();
}

void loop()
{
    drone.update();
}
