#include "PressureSensorManager.h"

uint16_t initial_pressure = 0;
uint16_t last_pressure = 0;

void ReadInitialPressure()
{
#ifndef DEBUG_NO_HARDWARE
  sensor.read();
  initial_pressure = (uint16_t)sensor.pressure();
#else
  initial_pressure = 1500;
#endif
}

bool ValidatePressure()
{
  bool valid_pressure = true;

  if (last_pressure == 0)
    last_pressure = initial_pressure;

  if (initial_pressure < 300)
    valid_pressure = false;
  if (initial_pressure > 6000)
    valid_pressure = false;

  int32_t diff = (int32_t)initial_pressure - (int32_t)last_pressure;
  if (abs(diff) > 2000)
    valid_pressure = false;

  return valid_pressure;
}

