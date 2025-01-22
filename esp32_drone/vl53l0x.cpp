#include "vl53l0x.h"
#include "logger.h"

static const char *TAG = "vl53l0x";

bool vl53l0x_init(void)
{
  logger.info(TAG, "Initializing vl53l0x device");
  return true;
}