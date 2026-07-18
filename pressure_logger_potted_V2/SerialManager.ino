#include "SerialManager.h"
#include "Utilities.h"


UartState uartState = UartState::CLEAN_COMM; // Start here
uint32_t listen_start = millis();

//
// State machine to cycle through the UART protocol as directed
// by a connected client.
//
// Returns true if the session reached CLEAN_COMM, false otherwise
bool handle_uart()
{
  switch (uartState)
  {
    case UartState::CLEAN_COMM:
      SERIAL_LOG("CLEAN_COMM state");
      handle_uart_clean_comm();
      break;
    case UartState::DETECT_CLIENT:
      //SERIAL_LOG("DETECT_CLIENT state");
      handle_uart_detect_client();
      break;
    case UartState::CONFIRM_CLIENT:
      SERIAL_LOG("CONFIRM_CLIENT state");
      handle_uart_confirm_client();
      break;
    case UartState::COMMAND_MODE:
      SERIAL_LOG("COMMAND_MODE state");
      handle_uart_command_mode();
      break;
    case UartState::CLEANUP_COMM:
      SERIAL_LOG("CLEANUP_COMM state");
      handle_uart_cleanup_comm();
      break;
  }

  return (uartState == UartState::CLEAN_COMM);
}

void handle_uart_clean_comm()
{
  pinMode(TX_pin, OUTPUT);
  pinMode(RX_pin, INPUT);

  Serial1.begin(9600);
  delay(50);

  // Clear startup noise
  //while (Serial1.available() > 0)
  //{
  //  Serial1.read();
  //}

  listen_start = millis();

  SERIAL_LOG("CLEAN_COMM -> DETECT_CLIENT");
  uartState = UartState::DETECT_CLIENT;
}

void handle_uart_detect_client()
{
  if (millis() - listen_start >= 2000)
    uartState = UartState::CLEANUP_COMM;
    return;

  while (Serial1.available() > 0)
  {
    cmd = Serial1.read();
    SERIAL_LOG("Received: " + String(cmd));

    if (cmd == 'P')
    {
      Serial1.write('H');
      uartState = UartState::CONFIRM_CLIENT;
      listen_start = millis();
      break;
    }
  }
}

void handle_uart_confirm_client()
{
  if (millis() - listen_start >= 2000)
    uartState = UartState::CLEANUP_COMM;
    return;

  if (Serial1.available() > 0)
  {
    cmd = Serial1.read(); // 

    if (cmd == 'C')
    {
      get_battery_voltage();
      uartState = UartState::COMMAND_MODE;
      listen_start = millis();
    }
  }
}

void handle_uart_command_mode()
{
  if (millis() - listen_start >= 10000)
    uartState = UartState::CLEAN_COMM;
    return;

    if (Serial1.available() > 0)
    {
      listen_start = millis(); // reset timeout
      cmd = Serial1.read();

      if (cmd == 'D')
      {
        dump_data();
      }
      else if (cmd == 'A')
      {
        Serial1.write('L');
      }
      else if (cmd == 'W')
      {
        current_address = 2;
        last_valid_address = 2;
        fram_wrapped = false;
        update_pointer();
        Serial1.print('E');
      }
      else if (cmd == 'X')
      {
        uartState = UartState::CLEANUP_COMM;
      }
    }
}

void handle_uart_cleanup_comm()
{
  delay(1);

  Serial1.end();
  pinMode(TX_pin, INPUT);
  pinMode(RX_pin, INPUT);
  uartState = UartState::CLEAN_COMM;
}


/***********************************************************************************/
void get_battery_voltage()
{
  // Wake ADC and wait for it to wake up
  // 1. Configure settings FIRST (ADC is disabled in setup)
  analogReadResolution(12);
  analogReference(AR_INTERNAL2V23);
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(63);
  // 2. Enable the ADC
  ADC->CTRLA.bit.ENABLE = 1;
  // 3. Wait for the ENABLE bit to synchronize
  while (ADC->STATUS.bit.SYNCBUSY)
    ;
  delay(5);
  analogRead(measure_battery); // Sacrificial read to clear internal ADC charge
  // Take a measurement
  uint16_t adc_value = analogRead(measure_battery);
  uint32_t battery_voltage = (uint32_t)adc_value * 907UL; // 0.000907 * 1000
  battery_voltage = battery_voltage / 1000UL;             // Now battery_voltage is in millivolts
  // Disable ADC
  ADC->CTRLA.bit.ENABLE = 0;
  while (ADC->STATUS.bit.SYNCBUSY)
    ;

  // Send battery_voltage;
  Serial1.print(battery_voltage);
  Serial1.write('V');
}


/********************************************************************/
void dump_data()
{
  Watchdog.reset();

  // Clear UART RX buffer
  uint32_t t0 = millis();
  while (millis() - t0 < 20)
  {
    while (Serial1.available() > 0)
    {
      Serial1.read();
    }
  }

  // Compute total valid bytes
  uint16_t total_bytes = 0;

  if (!fram_wrapped)
  {
    if (last_valid_address > 2)
    {
      total_bytes = last_valid_address - 2;
    }
    else
    {
      total_bytes = 0;
    }
  }
  else
  {
    // FIX #1: inclusive range (2 → FRAM_DATA_END)
    uint16_t before_wrap = (FRAM_DATA_END - 2) + 1;
    uint16_t after_wrap = last_valid_address - 2;
    total_bytes = before_wrap + after_wrap;
  }

  // Send STX
  Serial1.write(0x02);

  // Send size
  uint8_t size_hi = (uint8_t)(total_bytes >> 8);
  uint8_t size_lo = (uint8_t)(total_bytes & 0xFF);
  Serial1.write(size_hi);
  Serial1.write(size_lo);

  // Stream FRAM data
  uint16_t read_ptr = 2;
  uint16_t bytes_remaining = total_bytes;

  uint8_t buffer[64];
  uint16_t crc = 0xFFFF;

  while (bytes_remaining > 0)
  {
    Watchdog.reset();

    uint8_t chunk;

    if (bytes_remaining >= 64)
    {
      chunk = 64;
    }
    else
    {
      chunk = (uint8_t)bytes_remaining;
    }

    // Wrap only when exceeding FRAM_DATA_END
    if (read_ptr + chunk > FRAM_DATA_END)
    {
      uint16_t first = FRAM_DATA_END - read_ptr + 1;

      fram.read(read_ptr, buffer, first);
      Serial1.write(buffer, first);

      for (uint16_t i = 0; i < first; i++)
      {
        crc = crc16_update(crc, buffer[i]);
      }

      bytes_remaining -= first;
      read_ptr = 2;
    }
    else
    {
      fram.read(read_ptr, buffer, chunk);
      Serial1.write(buffer, chunk);

      for (uint16_t i = 0; i < chunk; i++)
      {
        crc = crc16_update(crc, buffer[i]);
      }

      read_ptr += chunk;

      if (read_ptr > FRAM_DATA_END)
      {
        read_ptr = 2;
      }

      bytes_remaining -= chunk;
    }
  }

  // Send ETX
  Serial1.write(0x03);

  // Send CRC
  uint8_t crc_hi = (uint8_t)(crc >> 8);
  uint8_t crc_lo = (uint8_t)(crc & 0xFF);
  Serial1.write(crc_hi);
  Serial1.write(crc_lo);

  Serial1.flush();
  delay(100);

  cmd = 0;
}


/******************************************************/
uint16_t crc16_update(uint16_t crc, uint8_t data)
{
  crc = crc ^ ((uint16_t)data << 8);

  for (uint8_t i = 0; i < 8; i++)
  {
    if ((crc & 0x8000) != 0)
    {
      crc = (crc << 1) ^ 0x1021;
    }
    else
    {
      crc = crc << 1;
    }
  }

  return crc;
}



//******************************** DEPRECATED, REMOVE *****************************/

bool handle_uart_session()
{
  // 1. INITIALIZE HARDWARE
  pinMode(TX_pin, OUTPUT);
  pinMode(RX_pin, INPUT);

  Serial1.begin(9600);
  delay(50);

  // Clear startup noise
  while (Serial1.available() > 0)
  {
    Serial1.read();
  }

  // 2. STAGE 1: DETECTION ('P' -> 'H')
  uint32_t listen_start = millis();
  bool pc_detected = false;

  while (millis() - listen_start < 200)
  {
    if (Serial1.available() > 0)
    {
      cmd = Serial1.read();

      if (cmd == 'P')
      {
        Serial1.write('H');
        pc_detected = true;
        break;
      }
    }
  }

  // 3. STAGE 2: CONFIRMATION ('H' -> 'C')
  bool session_confirmed = false;

  if (pc_detected)
  {
    uint32_t wait_conf = millis();

    while (millis() - wait_conf < 2000)
    {
      if (Serial1.available() > 0)
      {
        cmd = Serial1.read(); // 

        if (cmd == 'C')
        {
          session_confirmed = true;
          get_battery_voltage();
          break;
        }
      }
    }
  }

  // --- NEW: Track whether we actually reached Stage 3 ---
  bool in_command_mode = false;

  // 4. STAGE 3: COMMAND LOOP (AWAKE UNTIL DISCONNECT)
  if (session_confirmed)
  {
    in_command_mode = true;

    // This loop keeps the XIAO 100% awake until 'X' is received.
    // No pinMode toggling here prevents corruption of incoming 'D' commands.
    uint32_t last_activity = millis();

    while (true)
    {
      Watchdog.reset();

      // --- INACTIVITY TIMEOUT ---
      if (millis() - last_activity > 10000)
      {
        break; // PC disconnected or app closed
      }

      if (Serial1.available() > 0)
      {
        last_activity = millis(); // reset timeout
        cmd = Serial1.read();

        if (cmd == 'D')
        {
          dump_data();
        }
        else if (cmd == 'A')
        {
          Serial1.write('L');
        }
        else if (cmd == 'W')
        {
          current_address = 2;
          last_valid_address = 2;
          fram_wrapped = false;
          update_pointer();
          Serial1.print('E');
        }
        else if (cmd == 'X')
        {
          break; // disconnect
        }

        delay(1);
      }
    }
  }
  // 5. CLEANUP
  Serial1.end();
  pinMode(TX_pin, INPUT);
  pinMode(RX_pin, INPUT);

  // --- RETURN ONLY IF WE REACHED STAGE 3 ---
  return in_command_mode;
}


