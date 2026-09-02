#include "SerialManager.h"

void handle_uart_session_clean_comm();
bool handle_uart_sessaion_debounce();
void handle_uart_session_detect_client();
void handle_uart_session_confirm_client();
void handle_uart_session_command_mode();
void handle_uart_session_command_mode_exit();
void handle_uart_session_cleanup_comm();

void fram_erase();
void get_battery_voltage();
void dump_data();
uint16_t crc16_update(uint16_t crc, uint8_t data);

UartSessionState uartSessionState = UartSessionState::CLEAN_COMM; // Start here
uint32_t listen_start = millis();
uint32_t handshake_timeout_start = millis();
uint32_t disconnect_grace_start = 0;

const uint32_t fram_size = 32768;


//
// State machine to cycle through the UART protocol as directed
// by a connected client.
//
// Returns true if the session reached CLEAN_COMM, false otherwise
bool handle_uart_session()
{
  switch (uartSessionState)
  {
    case UartSessionState::CLEAN_COMM:
      SERIAL_LOG("CLEAN_COMM state");
      handle_uart_session_clean_comm();
      break;
    case UartSessionState::DETECT_CLIENT:
      //SERIAL_LOG("DETECT_CLIENT state");
      if (handle_uart_sessaion_debounce())
        handle_uart_session_detect_client();
      break;
    case UartSessionState::CONFIRM_CLIENT:
      SERIAL_LOG("CONFIRM_CLIENT state");
      if (handle_uart_sessaion_debounce())
        handle_uart_session_confirm_client();
      break;
    case UartSessionState::COMMAND_MODE:
      SERIAL_LOG("COMMAND_MODE state");
      if (handle_uart_sessaion_debounce())
        handle_uart_session_command_mode();
      break;
    case UartSessionState::COMMAND_MODE_EXIT:
      SERIAL_LOG("COMMAND_MODE_EXIT state");
      if (handle_uart_sessaion_debounce())
        handle_uart_session_command_mode_exit();
      break;
    case UartSessionState::CLEANUP_COMM:
      SERIAL_LOG("CLEANUP_COMM state");
      if (handle_uart_sessaion_debounce())
        handle_uart_session_cleanup_comm();
      break;
  }

  return (uartSessionState == UartSessionState::CLEAN_COMM);
}

void handle_uart_session_clean_comm()
{
  // Setup Hardware Serial1 port
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM1_CORE | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);
  
  SERCOM1->USART.CTRLA.bit.SWRST = 1;
  while (SERCOM1->USART.CTRLA.bit.SWRST || SERCOM1->USART.SYNCBUSY.bit.SWRST);
  
  pinMode(TX_pin, OUTPUT);
  pinMode(RX_pin, INPUT);
  Serial1.begin(115200);
  while (SERCOM1->USART.SYNCBUSY.bit.ENABLE);
  delay(50);
  
  while (Serial1.available() > 0) 
  { 
    Serial1.read(); 
  }

  listen_start = millis();
  handshake_timeout_start = millis();
  disconnect_grace_start = 0;

  SERIAL_LOG("CLEAN_COMM -> DETECT_CLIENT");
  uartSessionState = UartSessionState::DETECT_CLIENT;
}

bool handle_uart_sessaion_debounce()
{
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);

  // 1-Second cable glitch timmer for docking/debouncing
  if (digitalRead(RX_pin) == LOW) 
  {
    if (disconnect_grace_start == 0) 
    {
      disconnect_grace_start = millis();
    }
    if (millis() - disconnect_grace_start > 1000) 
    {
      uartSessionState = UartSessionState::CLEANUP_COMM;
      return false;
    }
  } 
  else 
  {
    disconnect_grace_start = 0; 
  }

  return (disconnect_grace_start == 0);
}

void handle_uart_session_detect_client()
{
  if (millis() - handshake_timeout_start >= 2000)
  {
    uartSessionState = UartSessionState::CLEANUP_COMM;
    return;
  }

  if (Serial1.available() > 0) 
  {
    char cmd = Serial1.read();
    // 1. Handshake PC -> 'P'
    if (cmd == 'P') 
    {
      Serial1.write('H'); // Arduino -> 'H'
      Serial1.flush();
      handshake_timeout_start = millis(); // Start 5s limit to see a 'C'
      uartSessionState = UartSessionState::CONFIRM_CLIENT;
    }
  }
  delay(1);
}

void handle_uart_session_confirm_client()
{
  if (millis() - handshake_timeout_start >= 5000)
  {
    uartSessionState = UartSessionState::CLEANUP_COMM;
    return;
  }

  if (Serial1.available() > 0) 
  {
    char cmd = Serial1.read(); // 

    // 2. Confirmed handshake PC -> 'C'
    if (cmd == 'C')
    {
      get_battery_voltage();
      handshake_timeout_start = millis();
      uartSessionState = UartSessionState::COMMAND_MODE;
    }
  }
  delay(1);
}

void handle_uart_session_command_mode()
{
  if (Serial1.available() > 0) 
  {
    char cmd = Serial1.read(); // 

    switch (cmd) 
    {
      // 3. Request Data dump PC -> 'D'
      case 'D': 
        dump_data();
        delay(5);
        Serial1.flush();
        break;

      // 4. Acknolage received data PC -> 'A'
      case 'A': 
        Serial1.write('L'); // Arduino -> 'L' waiting for 'X' or 'W'
        Serial1.flush();
        uartSessionState = UartSessionState::COMMAND_MODE_EXIT;
        break;

      default: 
        uartSessionState = UartSessionState::CLEANUP_COMM;
        break;
    }
  }
  delay(1);
}

void handle_uart_session_command_mode_exit()
{
  if (Serial1.available() > 0) 
  {
    char cmd = Serial1.read(); // 

    switch (cmd) 
    {
      // 5. Request FRAM erase PC -> 'W'
      case 'W': 
        fram_erase();
        Serial1.flush();
        break;

      // 6. Request disconnect of UART PC -> 'X'
      case 'X': 
      default: 
        uartSessionState = UartSessionState::CLEANUP_COMM;
        break;
    }
  }
  delay(1);
}

void handle_uart_session_cleanup_comm()
{
  // Shut down Serial port
  Serial1.flush(); 
  Serial1.end(); 
  
  pinMode(TX_pin, INPUT_PULLDOWN);
  pinMode(RX_pin, INPUT);

  uartSessionState = UartSessionState::CLEAN_COMM;
}

/****************************************************************************/
void fram_erase() 
{ 
    // 1. Execute full chip wipe (32KB @ 400kHz)
    for (uint32_t i = 0; i < fram_size; i++) 
    { 
        fram.write(i, 0x00); 
        
        // Kick watchdog every 4096 bytes
        if (i % 4096 == 0) 
        { 
            WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY; 
            while (WDT->STATUS.bit.SYNCBUSY); 
        } 
    } 

    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY; 
    while (WDT->STATUS.bit.SYNCBUSY); 

    // 2. Reset pointers instantly (FRAM writes are inherently reliable)
    current_address = 2; 
    last_valid_address = 2; 
    fram_wrapped = false; 
    update_pointer(); 

    // 3. Send Success Token immediately 
    Serial1.write('E'); 
    Serial1.flush();    
}

/***********************************************************************************/
void get_battery_voltage()
{
  // Wake ADC and wait for it to wake up
  // 1. Configure settings FIRST (ADC is disabled in setup)
  analogReadResolution(12);// 12 bit resolution
  analogReference(AR_INTERNAL2V23);// 2.23V referance
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(63); // 63 samples

  // 2. Enable the ADC
  ADC->CTRLA.bit.ENABLE = 1;

  // 3. Wait for the ENABLE bit to synchronize
  while (ADC->STATUS.bit.SYNCBUSY);
  delay(5);
  analogRead(measure_battery); // Sacrificial read to clear internal ADC charge

  // 4. Take a measurement
  uint16_t adc_value = analogRead(measure_battery);
  uint32_t battery_voltage = (uint32_t)adc_value * 907UL; // 0.000907 * 1000
  battery_voltage = battery_voltage / 1000UL;             // Now battery_voltage is in millivolts

  // 5. Disable ADC
  ADC->CTRLA.bit.ENABLE = 0;
  while (ADC->STATUS.bit.SYNCBUSY);

  // 6. Send battery_voltage;
  Serial1.print(battery_voltage);
  Serial1.write('V');
}

/********************************************************************/
void dump_data()
{
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY);

    // 1. Clear UART RX buffer
    uint32_t t0 = millis();
    while (millis() - t0 < 20) 
    {
        while (Serial1.available() > 0) 
        {
            Serial1.read();
        }
    }

    // 2. Compute total valid bytes
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
        // 3. inclusive range (2 → FRAM_DATA_END)
        uint16_t before_wrap = (FRAM_DATA_END - 2) + 1;
        uint16_t after_wrap = last_valid_address - 2;
        total_bytes = before_wrap + after_wrap;
    }

    // 4. Send Multi-byte STX (0xAA 0x55) ---
    Serial1.write(0xAA);
    Serial1.write(0x55);

    // 5. Send size
    uint8_t size_hi = (uint8_t)(total_bytes >> 8);
    uint8_t size_lo = (uint8_t)(total_bytes & 0xFF);
    Serial1.write(size_hi);
    Serial1.write(size_lo);

    // 6. Stream FRAM data
    uint16_t read_ptr = 2;
    uint16_t bytes_remaining = total_bytes;
    uint8_t buffer[64];
    uint16_t crc = 0xFFFF;

    while (bytes_remaining > 0) 
    {
        WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
        while (WDT->STATUS.bit.SYNCBUSY);

        uint8_t chunk;
        if (bytes_remaining >= 64) 
        {
            chunk = 64;
        } 
        else 
        {
            chunk = (uint8_t)bytes_remaining;
        }

        // 7. Wrap only when exceeding FRAM_DATA_END
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

    // 8. Send Multi-byte ETX (0x55 0xAA) ---
    Serial1.write(0x55);
    Serial1.write(0xAA);

    // 9.Send CRC
    uint8_t crc_hi = (uint8_t)(crc >> 8);
    uint8_t crc_lo = (uint8_t)(crc & 0xFF);
    Serial1.write(crc_hi);
    Serial1.write(crc_lo);

    Serial1.flush();
    delay(100);
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

/******************************************************************************/
void update_pointer()
{
  uint8_t buffer[2];
  buffer[0] = (uint8_t)(current_address >> 8);
  buffer[1] = (uint8_t)(current_address & 0xFF);

  // 1. Write to Primary Slot using multi-byte write
  fram.write(FRAM_POINTER_ADDR, buffer, 2);

  delay(1); // Tiny pause to let the I2C bus rest

  // 2. Write to Mirror Slot using multi-byte write
  fram.write(0x7FFE, buffer, 2);
}
