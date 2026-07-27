/* On the SAMD21, if you set a pin to OUTPUT first, it defaults to LOW
245 milliseconds is the wdt drift. For a timeout of 16384 it is 1.5%
Gr=SDA pin4  W=SCL pin5  Bu=RX pin7  V=TX pin6  ADC pin10 R1 = 1M  R2 = 1.5M
Current logic
'P' PC -> Arduino "Connect"
'H'	Arduino -> PC "Handshake"
'C'	PC -> Arduino	connected "Send the voltage"
'V' arduino -> PC "Sent the voltage"
'B' arduino -> PC "Backdoor" future
'T' arduino -> PC "Time set" furture
'D'	PC -> Arduino	"Send the data"
'A' PC -> Arduino "Processed the data"
'L' Arduino -> PC "Waiting for 'W' to wipe data or 'X' to disconnect"
'W'	Wipe	PC -> Arduino	"Erase the memory."
'E' Arduino -> PC "Memory wiped" waiting for 'X'
'X'	eXit	PC -> Arduino	"Close session and go to sleep."
Data Flow	0x02	STX	Arduino → PC	"Start of Text" (Binary start)
0x03	ETX	Arduino → PC	"End of Text" (Binary end)
**********RTC time set*************
For true UTC/GMT unix time an offset must be added to the compile time
PST (UTC-8) the offset is 8 hours * 3600 = +28800)
PDT (UTC-7) the offset is 7 hours 8 3600 = +25200 
Make sure to change it in setup*/

#include <Adafruit_FRAM_I2C.h>
#include <Adafruit_SleepyDog.h>
#include <MS5837.h>
#include <RTCZero.h>
#include <Wire.h>
#include<time.h>

// Define states
enum DeviceState
{
  CHECK_COMM,
  READ_SENSOR,
  SLEEP_MODE,
  ACTIVE_DEPLOYMENT
};

DeviceState currentState = CHECK_COMM; // Start here

// Global Objects
MS5837 sensor;
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
RTCZero rtc;

// Handshake Constants
char cmd = ' ';
const byte TX_pin = 6;           // serial TX pin
const byte RX_pin = 7;           // serial RX and "is-wet" test pin
const byte service_pin = 8;      // jumper for USB. wont be in production code
const byte measure_battery = 10; // ADC input from voltage divider for battery test

// Runtime Variables
uint32_t sleepDuration = 5; // Default transition sleep
uint16_t last_pressure = 0;
uint16_t current_address; // keeps tract of where deployments end
uint16_t last_valid_address = 2;
const uint16_t FRAM_DATA_END = 0x7FFB;     // last usable data byte
const uint16_t FRAM_POINTER_ADDR = 0x7FFC; // start of pointer region
bool fram_wrapped = false;                 // true once we've wrapped at least once

// Function Prototypes
void update_pointer();
void recover_fram_address();
void handle_active_deployment(uint16_t initial_pressure);
void start_new_deployment(uint16_t initial_pressure);
void save_pressure(uint16_t pressure);
void write_32(uint16_t address, uint32_t value);
void write_16(uint16_t address, uint16_t value);
void dump_data();
void go_to_sleep(uint32_t seconds);
void get_battery_voltage();
void inject_test_data();
bool handle_uart_session();
uint16_t crc16_update(uint16_t crc, uint8_t data);

void setup() 
{
    delay(500); // Settling delay 

    // Set ALL 10 GPIO pins to INPUT_PULLUP to prevent floating current
    for (int i = 0; i <= 10; i++) 
    {
        // Skip pins used for I2C (4, 5) and Serial (6, 7)
        if (i != 4 && i != 5 && i != 6 && i != 7) 
        {
            pinMode(i, INPUT_PULLUP);
        }
    }

    // Kill the On-board LEDs (D11, D12, D13) HIGH = OFF
    digitalWrite(11, HIGH); // TX LED
    pinMode(11, OUTPUT);
    digitalWrite(12, HIGH); // RX LED
    pinMode(12, OUTPUT);
    digitalWrite(13, HIGH); // User LED
    pinMode(13, OUTPUT);

    pinMode(measure_battery, INPUT); 
    pinMode(service_pin, INPUT_PULLUP);
    delay(100);

    if (digitalRead(service_pin) == LOW) 
    {
        USBDevice.attach();
        delay(5000);
    }
    else 
    {
        // Turn off USB and ADC to save power
        USBDevice.detach();
        ADC->CTRLA.bit.ENABLE = 0;
        
        while (ADC->STATUS.bit.SYNCBUSY)
        {
            // Do nothing, wait for ADC to stop safely
        }
    }

    // 1. Initialize core communication buses first
    Wire.begin();
    Wire.setClock(400000); // Fast I2C reduces CPU "on time"
    delay(10);

    // 2. Initialize the RTC library to establish baseline settings
    rtc.begin(); 

    // 3. SAFE BITWISE OVERRIDES: Keep external crystal active during sleep
    SYSCTRL->XOSC32K.reg |= SYSCTRL_XOSC32K_RUNSTDBY;
    SYSCTRL->XOSC32K.reg &= ~SYSCTRL_XOSC32K_ONDEMAND;

    while (!SYSCTRL->PCLKSR.bit.XOSC32KRDY) 
    {
        // Do nothing, wait for crystal to stabilize
    }

    // 4. Route XOSC32K into GCLK2 and force it to run during sleep
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(2); 
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_RUNSTDBY;

    while (GCLK->STATUS.bit.SYNCBUSY) 
    {
        // Do nothing, wait for synchronization
    }

    // 5. Connect GCLK2 to the RTC peripheral
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(RTC_GCLK_ID) | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;

    while (GCLK->STATUS.bit.SYNCBUSY) 
    {
        // Do nothing, wait for final sync
    }

    // 6. Automatically sync internal time to code compilation time
    struct tm tm_compile;
    char month_name[4]; 

    sscanf(__DATE__, "%3s %d %d", month_name, &tm_compile.tm_mday, &tm_compile.tm_year);
    sscanf(__TIME__, "%d:%d:%d", &tm_compile.tm_hour, &tm_compile.tm_min, &tm_compile.tm_sec);

    const char month_lookup[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    tm_compile.tm_mon = (strstr(month_lookup, month_name) - month_lookup) / 3;
    tm_compile.tm_year -= 1900;
    tm_compile.tm_isdst = -1; 

    uint32_t compile_time_local = mktime(&tm_compile);
    uint32_t offset = 25200; // UTC-7 PDT 
    uint32_t compile_time_utc = compile_time_local + offset; 

    if (rtc.getEpoch() < compile_time_utc) 
    {
        rtc.setEpoch(compile_time_utc);
    }

    // 7. Initialize FRAM and recover write pointer
    if (fram.begin(0x50)) 
    {
        recover_fram_address(); 
    }

    // 8. Initialize the pressure sensor
    sensor.setModel(MS5837::MS5837_30BA); 
    sensor.setFluidDensity(1029);         
    sensor.init();                        
    delay(1000);

    // 9. DISABLE THE BROWN-OUT DETECTOR FOR MAX SLEEP POWER SAVINGS
    SYSCTRL->BOD33.bit.ENABLE = 0;

    while (SYSCTRL->PCLKSR.bit.BOD33RDY) 
    {
        // Do nothing, wait for BOD33 to safely turn off completely
    }

    // 10. Enable system Watchdog safety right before loop execution begins
    Watchdog.enable(16384);
    
  // --- TEMPORARY DESK TESTING ONLY ---
  // This will overwrite your memory with fake data every time it boots.
  // Comment this out before your real deployment
  inject_test_data();
}
/*********************************************************************************/

void loop()
{
  Watchdog.reset();

  // --- Read sensor once per wake cycle ---
  sensor.read();
  uint16_t initial_pressure = (uint16_t)sensor.pressure();
  // --- UART wake detection 
  pinMode(RX_pin, INPUT_PULLDOWN);
  delayMicroseconds(300);

  if (digitalRead(RX_pin) == HIGH)
  {
    // Now switch to UART mode
    pinMode(TX_pin, OUTPUT);
    pinMode(RX_pin, INPUT);

    Serial1.begin(9600);
    handle_uart_session();
    Serial1.end();

    pinMode(TX_pin, INPUT);
    pinMode(RX_pin, INPUT);
    return;
  }

  // --- Validate pressure reading ---
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

  if (!valid_pressure)
  {
    sleepDuration = 5;
    go_to_sleep(sleepDuration);
    return;
  }

  last_pressure = initial_pressure;

  // --- State Machine ---
  switch (currentState)
  {
  case READ_SENSOR:
  {
    if (initial_pressure >= 2000)
    {
      handle_active_deployment(initial_pressure);
      sleepDuration = 10;
    }
    else
    {
      sleepDuration = 5;
    }

    currentState = SLEEP_MODE;
    break;
  }

  case SLEEP_MODE:
  {
    pinMode(RX_pin, INPUT);
    pinMode(TX_pin, INPUT);

    go_to_sleep(sleepDuration);
    currentState = READ_SENSOR;
    break;
  }

  default:
    currentState = READ_SENSOR;
    break;
  }
}

//************************************************************************************************
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

/*******************************************************/
void go_to_sleep(uint32_t seconds)
{
  // 1. Calculate the exact second to wake up
  uint32_t alarm_time = rtc.getEpoch() + seconds;
  rtc.setAlarmEpoch(alarm_time);

  // 2. Tell the RTC hardware to watch for this specific time
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  // 3. Enter deepest sleep mode (~15-40uA)
  // The CPU stops here until the RTC alarm triggers
  rtc.standbyMode();
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
/******************************************************************************/
void update_pointer()
{
  uint8_t buffer[2];
  buffer[0] = (uint8_t)(current_address >> 8);
  buffer[1] = (uint8_t)(current_address & 0xFF);

  // 1. Write to Primary Slot using multi-byte write
  fram.write(FRAM_POINTER_ADDR, buffer, 2);

  delayMicroseconds(100); // Tiny pause to let the I2C bus rest

  // 2. Write to Mirror Slot using multi-byte write
  fram.write(0x7FFE, buffer, 2);
}

/*************************************************************************************/

void recover_fram_address()
{
  // 1. Safe 2-byte read buffer
  uint8_t buffer[2];

  // 2. Safely grab Primary Pointer
  fram.read(FRAM_POINTER_ADDR, buffer, 2);
  uint16_t primary = ((uint16_t)buffer[0] << 8) | buffer[1];

  // 3. Safely grab Mirror Pointer
  fram.read(0x7FFE, buffer, 2);
  uint16_t mirror = ((uint16_t)buffer[0] << 8) | buffer[1];

  // Helper lambda to validate a pointer
  auto is_valid = [&](uint16_t p)
  {
    if (p < 2 || p > FRAM_DATA_END) return false;
    if (p & 0x0001) return false; // Must be even (samples are 2 bytes)
    if (p >= 0x7FFC) return false; // Must not point into pointer storage region
    if (p == 0x0000 || p == 0xFFFF) return false;
    return true;
  };

  bool primary_ok = is_valid(primary);
  bool mirror_ok = is_valid(mirror);

  if (primary_ok && mirror_ok)
  {
    // Both valid → choose the larger one (monotonic pointer)
    current_address = (primary >= mirror) ? primary : mirror;
    if (primary != mirror)
      update_pointer();
  }
  else if (primary_ok)
  {
    current_address = primary;
    update_pointer();
  }
  else if (mirror_ok)
  {
    current_address = mirror;
    update_pointer();
  }
  else
  {
    current_address = 2;
    update_pointer();
  }
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

/************This is for testing only*****************/
void inject_test_data()
{
  // 1. Reset write pointer to start of data region
  current_address = 2;
  fram_wrapped = false;
  last_valid_address = 2;

  // 2. Start a deployment with a safe initial pressure
  uint16_t safe_initial = 1025; // already guaranteed != 0x0003
  start_new_deployment(safe_initial);

  // 3. Write 20 fake samples (pressure ramp)
  for (int i = 0; i < 20; i++)
  {
    uint16_t fake_pressure = 2000 + (i * 100);
    save_pressure(fake_pressure); // also guaranteed != 0x0003
  }

  // 4. Update last_valid_address to the final write location
  last_valid_address = current_address;
}
