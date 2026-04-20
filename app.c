#include "app.h"
#include "i2c_simulatedsensor.h"

void app_init(void)
{
  i2c_simulatedsensor_init();
}

void app_process_action(void)
{
  // Counter advances on each 0xAC trigger from master — nothing to do here
}
