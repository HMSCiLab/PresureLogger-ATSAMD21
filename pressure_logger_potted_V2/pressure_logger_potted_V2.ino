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
#include "SerialManager.h"
#include "DeploymentManager.h"
#include "PressureSensorManager.h"
#include "Utilities.h"

// Uncomment this line to add debugging prints.
#define DEBUG_LOGGING
#ifdef DEBUG_LOGGING
#define SERIAL_LOG(x) Serial.println(x)
#else
#define SERIAL_LOG(x)
#endif

// Uncomment this line to exclude code that requires hardware.
#define DEBUG_NO_HARDWARE

// Define states
enum DeviceState
{
  CHECK_COMM,
  READ_SENSOR,
  SLEEP_MODE,
  ACTIVE_DEPLOYMENT
};

DeviceState currentState = READ_SENSOR; // Start here

// Global Objects
MS5837 sensor;
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
RTCZero rtc;

// Handshake Constants
char cmd = ' ';
// Redefine pins for feather.
//const byte TX_pin = 6;           // serial TX pin
//const byte RX_pin = 7;           // serial RX and "is-wet" test pin
const byte TX_pin = 1;           // serial TX pin
const byte RX_pin = 0;           // serial RX and "is-wet" test pin
const byte service_pin = 8;      // jumper for USB. wont be in production code
const byte measure_battery = 10; // ADC input from voltage divider for battery test

// Runtime Variables
uint32_t sleepDuration = 5; // Default transition sleep
uint16_t current_address; // keeps tract of where deployments end
uint16_t last_valid_address = 2;
const uint16_t FRAM_DATA_END = 0x7FFB;     // last usable data byte
const uint16_t FRAM_POINTER_ADDR = 0x7FFC; // start of pointer region
bool fram_wrapped = false;                 // true once we've wrapped at least once

// Function Prototypes
void SetupIOPins();
void SetupUsbDevice();
void SetupRtc();
void recover_fram_address();
void go_to_sleep(uint32_t seconds);
void inject_test_data();

void setup() 
{
#ifdef DEBUG_LOGGING
  Serial.begin(115200);
  while(!Serial);
#endif

  SERIAL_LOG("SetupIOPins()");
  SetupIOPins();
  SERIAL_LOG("SetupUsbDevice()");
  SetupUsbDevice();
  SERIAL_LOG("SetupRtc()");
  SetupRtc();

  // 1. Initialize core communication buses first
  Wire.begin();
  Wire.setClock(400000); // Fast I2C reduces CPU "on time"
  delay(10);

  // 7. Initialize FRAM and recover write pointer
  if (fram.begin(0x50)) 
  {
      recover_fram_address(); 
  }

  // 8. Initialize the pressure sensor
#ifndef DEBUG_NO_HARDWARE
  sensor.setModel(MS5837::MS5837_30BA); 
  sensor.setFluidDensity(1029);         
  sensor.init();                        
  delay(1000);
#endif

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
  SERIAL_LOG("Setup function done");
}
/*********************************************************************************/
void loop()
{
  Watchdog.reset();

  switch (currentState)
  {
    case DeviceState::CHECK_COMM:
      if (handle_uart())
        currentState = DeviceState::READ_SENSOR; // Resume normal operation after UART session
      break;

    case DeviceState::READ_SENSOR:
      SERIAL_LOG("READ_SENSOR state");
      ReadInitialPressure();
      if (initial_pressure >= 2000)
      {
        handle_active_deployment(initial_pressure);
        sleepDuration = 10;
      }
      else
      {
        sleepDuration = 5;
      }

      currentState = DeviceState::SLEEP_MODE;
      break;

    case DeviceState::SLEEP_MODE:
      SERIAL_LOG("SLEEP_MODE state");
      pinMode(RX_pin, INPUT);
      pinMode(TX_pin, INPUT);

      go_to_sleep(sleepDuration);
      currentState = DeviceState::READ_SENSOR;
      break;

    case DeviceState::ACTIVE_DEPLOYMENT:
      SERIAL_LOG("ACTIVE_DEPLOYMENT state");
      // --- Read sensor once per wake cycle ---
      ReadInitialPressure();
      if (!ValidatePressure())
      {
        sleepDuration = 5;
        go_to_sleep(sleepDuration);
      }
      else
      {
        currentState = DeviceState::READ_SENSOR;
      }
      break;

    default:
      SERIAL_LOG("DEFAULT state");
      currentState = DeviceState::READ_SENSOR;
      break;
  }

  // --- UART wake detection 
  if (currentState != DeviceState::CHECK_COMM)
  {
    pinMode(RX_pin, INPUT_PULLDOWN);
    delayMicroseconds(300);

    if (digitalRead(RX_pin) == HIGH)
    {
      SERIAL_LOG("UART wake detected");
      // Force UART session on next loop iteration
      uartState = UartState::CLEAN_COMM;
      currentState = DeviceState::CHECK_COMM;
    }
  }
}

#if 0
void loop_old()
{
  Watchdog.reset();

  // --- Read sensor once per wake cycle ---
  ReadInitialPressure();

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

  if (!ValidatePressure())
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
#endif

//************************************************************************************************

/*******************************************************/
void SetupIOPins()
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
}

/*******************************************************/
void SetupUsbDevice()
{
#ifndef DEBUG_NO_HARDWARE
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
#endif
}

/*******************************************************/
void SetupRtc()
{
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
}

/*******************************************************/
void go_to_sleep(uint32_t seconds)
{
#ifndef DEBUG_NO_HARDWARE
  // 1. Calculate the exact second to wake up
  uint32_t alarm_time = rtc.getEpoch() + seconds;
  rtc.setAlarmEpoch(alarm_time);

  // 2. Tell the RTC hardware to watch for this specific time
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  // 3. Enter deepest sleep mode (~15-40uA)
  // The CPU stops here until the RTC alarm triggers
  rtc.standbyMode();
#else
  delay(seconds * 1000);
#endif
}

/*************************************************************************************/

void recover_fram_address()
{
  // 1. Safe 2-byte read buffer
  uint8_t buffer[2];

  // 2. Safely grab Primary Pointer
#ifndef DEBUG_NO_HARDWARE
  fram.read(FRAM_POINTER_ADDR, buffer, 2);
#else
  buffer[0] = FRAM_POINTER_ADDR >> 8;
  buffer[1] = FRAM_POINTER_ADDR & 0xff;
#endif
  uint16_t primary = ((uint16_t)buffer[0] << 8) | buffer[1];

  // 3. Safely grab Mirror Pointer
#ifndef DEBUG_NO_HARDWARE
  fram.read(0x7FFE, buffer, 2);
#else
  buffer[0] = 0x7FFE >> 8;
  buffer[1] = 0x7FFE & 0xff;
#endif
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
