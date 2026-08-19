#include "DeploymentManager.h"
#include "Utilities.h"

/******************************************************************************/
void handle_active_deployment(uint16_t initial_pressure)
{
    // Start a new deployment header
    start_new_deployment(initial_pressure);

    const uint16_t START_THRESHOLD = 2000;  // start logging
    const uint16_t STOP_THRESHOLD  = 1100;  // stop logging (hysteresis)

    while (true)
    {
        Watchdog.reset();

        sensor.read();
        uint16_t pressure_value = (uint16_t)sensor.pressure();

        // Exit condition with hysteresis
        if (pressure_value <= STOP_THRESHOLD)
        {
            return;   // deployment finished
        }

        // Log sample
        save_pressure(pressure_value);

        // Keep RX quiet during sleep
        pinMode(RX_pin, INPUT_PULLDOWN);

        // Sleep 10 seconds between samples
        go_to_sleep(10);

        delay(50); // I2C settle
    }
}

/******************************************************/
void save_pressure(uint16_t pressure_value)
{
  // Ensure room for 2 bytes
  if (current_address + 2 > FRAM_POINTER_ADDR)
  {
    current_address = 2;
    fram_wrapped = true;
  }

  // Sanitize
  if (pressure_value == 0xFFFF || pressure_value == 0x0003)
  {
    pressure_value = 0xFFFE;
  }

  // Write pressure sample
  write_16(current_address, pressure_value);
  current_address = current_address + 2;

  last_valid_address = current_address; // <-- REQUIRED FIX

  update_pointer();
}

/***********************************************************************/
void start_new_deployment(uint16_t initial_pressure)
{
  if (current_address + 8 > FRAM_POINTER_ADDR)
  {
    current_address = 2;
    fram_wrapped = true;
  }

  // 1. Write everything sequentially
  write_16(current_address, 0xFFFF);

  uint32_t epoch = rtc.getEpoch();
  write_32(current_address + 2, epoch); // Use offsets!

  if (initial_pressure == 0xFFFF || initial_pressure == 0x0003)
  {
    initial_pressure = 0xFFFE;
  }
  write_16(current_address + 6, initial_pressure);

  // 2. Advance your RAM pointer by the total 8 bytes
  current_address += 8;
  last_valid_address = current_address;

  // 3. Commit it to physical FRAM ONCE
  update_pointer();
}

/****************************************************************************/
void write_16(uint16_t address, uint16_t value)
{
  uint8_t buffer[2];
  buffer[0] = (uint8_t)(value >> 8);
  buffer[1] = (uint8_t)(value & 0xFF);

  fram.write(address, buffer, 2);
}

/****************************************************************************/
void write_32(uint16_t address, uint32_t value)
{
  uint8_t buffer[4];
  buffer[0] = (uint8_t)(value >> 24);
  buffer[1] = (uint8_t)(value >> 16);
  buffer[2] = (uint8_t)(value >> 8);
  buffer[3] = (uint8_t)(value & 0xFF);

  // library's multi-byte write is better for battery than 4 single writes
  fram.write(address, buffer, 4);
}

