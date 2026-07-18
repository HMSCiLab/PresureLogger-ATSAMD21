#include "Utilities.h"

/******************************************************************************/
void update_pointer()
{
  uint8_t buffer[2];
  buffer[0] = (uint8_t)(current_address >> 8);
  buffer[1] = (uint8_t)(current_address & 0xFF);

  // 1. Write to Primary Slot using multi-byte write
#ifndef DEBUG_NO_HARDWARE
  fram.write(FRAM_POINTER_ADDR, buffer, 2);
#endif

  delayMicroseconds(100); // Tiny pause to let the I2C bus rest

  // 2. Write to Mirror Slot using multi-byte write
#ifndef DEBUG_NO_HARDWARE
  fram.write(0x7FFE, buffer, 2);
#endif
}

