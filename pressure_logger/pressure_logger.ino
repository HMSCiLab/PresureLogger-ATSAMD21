/* Communication logic
'P' PC -> Arduino "Connect"
'H'	Arduino -> PC "Handshake"
'C'	PC -> Arduino	connected "Send the voltage"
'V' arduino -> PC "Sent the voltage"
'D'	PC -> Arduino	"Send the data"
'A' PC -> Arduino "Processed the data"
'L' Arduino -> PC "Waiting for 'W' to wipe data or 'X' to disconnect"
'W'	Wipe	PC -> Arduino	"Erase the memory."
'E' Arduino -> PC "Memory wiped" waiting for 'X'
'X'	exit	PC -> Arduino	"Close session and go to sleep."
['B' PC -> Arudino "Backdoor" Furture]
['T' PC -> Arduino "Time syncronize" Furture]
**********Data Flow**************************************
0xAA 0x55	STX	Arduino → PC	"Start of Text" (Binary start)
0x55 0xAA	ETX	Arduino → PC	"End of Text" (Binary end)
STX > Data > ETX > CRC
**********RTC time set*************************************************
For true UTC/GMT unix time an offset must be added to the compile time
Eastern Time (ET): Standard is UTC−05:00; Daylight is UTC−04:00
Central Time (CT): Standard is UTC−06:00; Daylight is UTC−05:00
Mountain Time (MT): Standard is UTC−07:00; Daylight is UTC−06:00
Pacific Time (PT): Standard is UTC−08:00; Daylight is UTC−07:00
Alaska Time (AKT): Standard is UTC−09:00; Daylight is UTC−08:00
Hawaii-Aleutian Time (HAT): Standard is UTC−10:00
Set UTC_OFFSET_HOURS = "Your timezone" 
************Pin connections********************************************
SDA pin 4
SCL pin 5
RX pin 7 external 6.8k pulldown resistor
TX pin 6
ADC pin 10 voltage divider 1M & 1.5M ~= 2.23V
************Optional*******************************************
To lower power consumption even more the main clock can be reduced 
The default value is 48MHz
Update the system clock variable to reflect your new speed (e.g., 12 MHz)
SystemCoreClock = 12000000; 
Recalculate and reload the SysTick timer so millis() stays accurate
SysTick_Config(SystemCoreClock / 1000);  */

#include <Adafruit_FRAM_I2C.h>
#include <MS5837.h>
#include <Wire.h>
#include <time.h>

// Operating states states
enum DeviceState
{
  CHECK_UART,
  READ_SENSOR,
  SHELF_MODE,
};
DeviceState currentState = CHECK_UART; // Start here

// Global Objects
MS5837 sensor;
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();

// Handshake Constants
const byte TX_pin = 6;                    // serial TX pin
const byte RX_pin = 7;                    // serial RX pin
const byte measure_battery = 10;          // ADC input from voltage divider for battery test

// Runtime Variables
const uint8_t UTC_OFFSET_HOURS = 7;         // * besure to set your timezone* PDT 07/10/2026
uint32_t sleep_duration;                    // Sleep duration = standby or measurement
uint32_t standby_seconds = 5;               // The amount of time (in seconds) the logger sleeps when not deployed
uint32_t measurement_interval = 10;         // The amount of time (in seconds) between pressure measurements
uint32_t fram_size = 32768;
uint16_t last_pressure = 0;
const uint16_t START_THRESHOLD = 2000;
const uint16_t STOP_THRESHOLD = 1100;
uint16_t current_address;                   // keeps tract of where deployments end
uint16_t last_valid_address = 2;
const uint16_t FRAM_DATA_END = 0x7FFB;      // last usable data byte
const uint16_t FRAM_POINTER_ADDR = 0x7FFC;  // start of pointer region
bool fram_wrapped = false;                  // true once we've wrapped at least once
volatile bool alarmFired = false;

// Function Prototypes
void RTC_Handler();
void fram_erase();
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
    delay(5000);// 5 second delay to help upload code. leave 1000 before upload of final code
    // --- Unexposed pin configuration ---
    pinMode(0, INPUT_PULLUP);
    pinMode(1, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
    pinMode(measure_battery, INPUT);

     // --- Exposed pin configuration ---
    pinMode(RX_pin, INPUT);               //has 6.8k externak pulldown
    pinMode(TX_pin, INPUT_PULLDOWN);

    // --- LEDS Off ---
    // --- The power LED has been physicaly removed ---
    digitalWrite(11, HIGH);
    pinMode(11, OUTPUT);
    digitalWrite(12, HIGH);
    pinMode(12, OUTPUT);
    digitalWrite(13, HIGH);
    pinMode(13, OUTPUT);

    // 1. Enable external 32.768 kHz crystal
    SYSCTRL->XOSC32K.reg =
        SYSCTRL_XOSC32K_STARTUP(0x6u) |
        SYSCTRL_XOSC32K_XTALEN |
        SYSCTRL_XOSC32K_EN32K |
        SYSCTRL_XOSC32K_ENABLE |
        SYSCTRL_XOSC32K_RUNSTDBY;

    while (!SYSCTRL->PCLKSR.bit.XOSC32KRDY);

    // 2. Route crystal to GCLK2 (divide by 32 → 1024 Hz)
    GCLK->GENDIV.reg =
        GCLK_GENDIV_ID(2) |
        GCLK_GENDIV_DIV(4);

    GCLK->GENCTRL.reg =
        GCLK_GENCTRL_ID(2) |
        GCLK_GENCTRL_SRC_XOSC32K |
        GCLK_GENCTRL_DIVSEL |
        GCLK_GENCTRL_GENEN |
        GCLK_GENCTRL_RUNSTDBY;

    while (GCLK->STATUS.bit.SYNCBUSY);

    // 3. Connect GCLK2 to RTC
    GCLK->CLKCTRL.reg =
        GCLK_CLKCTRL_ID_RTC |
        GCLK_CLKCTRL_GEN_GCLK2 |
        GCLK_CLKCTRL_CLKEN;

    while (GCLK->STATUS.bit.SYNCBUSY);

    // 4. Reset RTC
    RTC->MODE2.CTRL.reg = RTC_MODE2_CTRL_SWRST;

    while (RTC->MODE2.STATUS.bit.SYNCBUSY);

    while (RTC->MODE2.CTRL.bit.SWRST);

    // 5. Configure RTC in MODE2 (calendar) with 1024 prescaler
    RTC->MODE2.CTRL.reg =
        RTC_MODE2_CTRL_MODE_CLOCK |
        RTC_MODE2_CTRL_PRESCALER_DIV1024;

    while (RTC->MODE2.STATUS.bit.SYNCBUSY);

   // 6. Initialize the RTC calendar
    char month_str[4];
    int day;
    int year;
    int hour;
    int minute;
    int second;

    sscanf(__DATE__, "%3s %d %d", month_str, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);

    const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";

    int month = (strstr(months, month_str) - months) / 3 + 1;

    // 7. Convert compiler local time to UTC 
    hour += UTC_OFFSET_HOURS;

    while (hour >= 24)
    {
        hour -= 24;
        day++;

        int days_in_month;

        switch (month)
        {
            case 1:
            case 3:
            case 5:
            case 7:
            case 8:
            case 10:
            case 12:
                days_in_month = 31;
                break;

            case 4:
            case 6:
            case 9:
            case 11:
                days_in_month = 30;
                break;

            case 2:
                if ((year % 4 == 0 && year % 100 != 0) ||
                    (year % 400 == 0))
                {
                    days_in_month = 29;
                }
                else
                {
                    days_in_month = 28;
                }
                break;

            default:
                days_in_month = 31;
                break;
        }

        if (day > days_in_month)
        {
            day = 1;
            month++;

            if (month > 12)
            {
                month = 1;
                year++;
            }
        }
    }

  uint8_t yy = year - 2000;

  RTC->MODE2.CLOCK.reg =
  RTC_MODE2_CLOCK_YEAR(yy) |
  RTC_MODE2_CLOCK_MONTH(month) |
  RTC_MODE2_CLOCK_DAY(day) |
  RTC_MODE2_CLOCK_HOUR(hour) |
  RTC_MODE2_CLOCK_MINUTE(minute) |
  RTC_MODE2_CLOCK_SECOND(second);

  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  // 8. Set the Alarm Mask Type
  RTC->MODE2.Mode2Alarm[0].MASK.reg = RTC_MODE2_MASK_SEL_YYMMDDHHMMSS;
  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  // 9. Enable Alarm 0 hardware interrupt 
  //    Clear any pending ALARM0 interrupt
  RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0;

  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  // 10. Enable ALARM0 interrupt in the RTC
  RTC->MODE2.INTENSET.reg = RTC_MODE2_INTENSET_ALARM0;

  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  // 11. Enable the interrupt in the NVIC
  NVIC_ClearPendingIRQ(RTC_IRQn);
  NVIC_EnableIRQ(RTC_IRQn);

  // 12. Enable RTC calander
  RTC->MODE2.CTRL.bit.ENABLE = 1;
  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  // 13. Initialize the Wire bus 
  Wire.begin();
  Wire.setClock(400000);
  delay(50);

  // 14. Initialize the FRAM
  if (fram.begin(0x50))
  {
    recover_fram_address();
  }

  // 15. Initialize sensor
  sensor.setModel(MS5837::MS5837_30BA); // using 300 meter sensor
  sensor.setFluidDensity(1029);         // Set for saltwater density
  sensor.init();                        // wait for pressure sensor
  delay(1000);
    
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);

  // 16. Keep the Brown-Out Detector active during standby.
  SYSCTRL->BOD33.bit.RUNSTDBY = 1;
  while (!SYSCTRL->PCLKSR.bit.BOD33RDY);

  // 17. WATCHDOG TIMER ACTIVATION (Hardened Deep Standby Safety Profile)
  //     Watchdog peripheral is turned off before configuring
  WDT->CTRL.bit.ENABLE = 0;
  while (WDT->STATUS.bit.SYNCBUSY);

  // 18. Configure the timeout window to ~16 seconds (16384 cycles) FIRST!
  WDT->CONFIG.bit.PER = 0xB;
  while (WDT->STATUS.bit.SYNCBUSY);

  // 19. Turn the Watchdog peripheral ON using an atomic bitwise enable mask
  WDT->CTRL.reg |= WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);

  // 20. Clear the timer counter cleanly 
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);

  // 21. Disable USB and ADC to consurve power consumption
  USBDevice.detach();
  ADC->CTRLA.bit.ENABLE = 0;

  while (ADC->STATUS.bit.SYNCBUSY);
  sleep_duration = standby_seconds;

  // --- TEMPORARY DESK TESTING ONLY ---
  // This will overwrite the memory with fake data every time it boots.
  // Delete this for production
  inject_test_data();
}
/************************************************************************************************************/

void loop()
{

  // WATCHDOG RESET "pet the dog"
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);

  switch (currentState)
  {   //start of switch

    /*------------------------------------------------------------------------*/
    case CHECK_UART: 
    {
      if (digitalRead(RX_pin) == LOW)
      {
          currentState = READ_SENSOR;
          break;
      }

      delay(50);

      if (digitalRead(RX_pin) != HIGH)
      {
          currentState = READ_SENSOR;
          break;
      }

      handle_uart_session();
      sleep_duration = standby_seconds;
      currentState = SHELF_MODE;
    }

    /*-------------------------------------------------------------------------*/
    case READ_SENSOR: // Read the pressure sensor
    {
      sensor.read();
      uint16_t initial_pressure = (uint16_t)sensor.pressure();

      // Make sure sensor reading is not a wild glitch
      bool valid_pressure = true;
      if (last_pressure == 0)
      {
          last_pressure = initial_pressure;
      }

      // At a normal descent rate these values are reasonable
      if (initial_pressure < 300 || initial_pressure > 6000)
      {
          valid_pressure = false;
      }

      int32_t diff = (int32_t)initial_pressure - (int32_t)last_pressure;
      if (abs(diff) > 2000)
      {
          valid_pressure = false;
      }

      if (!valid_pressure)
      {
          sleep_duration = standby_seconds;
          currentState = SHELF_MODE;
          break;
      }

      last_pressure = initial_pressure;

      if (initial_pressure >= START_THRESHOLD)
      {
          start_new_deployment(initial_pressure);
          handle_active_deployment();
      }       
      sleep_duration = standby_seconds;
      currentState = SHELF_MODE;
      break;
   }

    /*--------------------------------------------------------------------*/
   case SHELF_MODE:
   {
      go_to_sleep(sleep_duration);
      currentState = CHECK_UART;
      break;
   } // end of case  
  } //end of switch
} //end of loop

/*****************************************************************************/
void go_to_sleep(uint32_t seconds)
{
   RTC->MODE2.READREQ.reg =
            RTC_READREQ_RREQ |
            RTC_READREQ_ADDR(0x10);
   while (RTC->MODE2.STATUS.bit.SYNCBUSY);


  // Read current seconds
  uint8_t current_sec = RTC->MODE2.CLOCK.bit.SECOND;

  uint8_t target_sec = current_sec + seconds;

  if (target_sec >= 60)
  {
      target_sec = target_sec - 60;
  }

  // Prevent immediate wake if rollover caused equality
  if (target_sec == current_sec)
  {
      target_sec = target_sec + 1;

      if (target_sec >= 60)
      {
          target_sec = target_sec - 60;
      }
  }

  // PROGRAM ALARM 
  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  RTC->MODE2.Mode2Alarm[0].ALARM.reg = RTC_MODE2_ALARM_SECOND(target_sec);
  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  // Set mask for seconds only
  RTC->MODE2.Mode2Alarm[0].MASK.reg = RTC_MODE2_MASK_SEL(1);
  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  // Clear and enable interrupt
  RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0;
  NVIC_ClearPendingIRQ(RTC_IRQn);
  RTC->MODE2.INTENSET.reg = RTC_MODE2_INTENSET_ALARM0;
  NVIC_EnableIRQ(RTC_IRQn);

  // Set for deep sleep
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  /*******Blinks LED for sleep durration***********
  ****For testing only. Delete before uploading*****/
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
      digitalWrite(LED_BUILTIN, HIGH);

  __DSB();
  __ISB();
  __WFI();
}

/***************************************************************************************/
bool handle_uart_session() 
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

    bool session_active = true; 
    bool is_session_unlocked = false;
    bool reached_stage_3 = false;
    
    //char cmd = 0;
    uint32_t handshake_timeout_start = millis();
    uint32_t disconnect_grace_start = 0;

    //Switch case

    while (session_active) 
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
                session_active = false; 
            }
        } 
        else 
        {
            disconnect_grace_start = 0; 
        }

        // Get Charactor command
        if (disconnect_grace_start == 0) 
        {
            if (Serial1.available() > 0) 
            {
                char cmd = Serial1.read();
                switch (cmd) 
                {
                    
                    // 1. Handshake PC -> 'P'
                    case 'P': 
                    {
                        Serial1.write('H'); // Arduino -> 'H'
                        Serial1.flush();
                        handshake_timeout_start = millis(); // Start 5s limit to see a 'C'
                        break;
                    }

                    // 2. Confirmed handshake PC -> 'C'
                    case 'C': 
                    {
                        // If PC sends 'C' within 5 seconds of 'P', unlock the device
                        if (millis() - handshake_timeout_start < 5000) 
                        {
                            get_battery_voltage();
                            is_session_unlocked = true; 
                            reached_stage_3 = true;     
                        }
                        break;
                    }

                    // 3. Request Data dump PC -> 'D'
                    case 'D': 
                    {
                        if (is_session_unlocked) 
                        {
                            dump_data();
                            delay(5);
                            Serial1.flush();
                        }
                        break;
                    }

                    // 4. Acknolage received data PC -> 'A'
                    case 'A': 
                    {
                        if (is_session_unlocked) 
                        {
                            Serial1.write('L'); // Arduino -> 'L' waiting for 'X' or 'W'
                            Serial1.flush();
                        }
                        break;
                    }

                    // 5. Request FRAM erase PC -> 'W'
                    case 'W': 
                    {
                        if (is_session_unlocked) 
                        {
                            fram_erase();
                            Serial1.flush();
                        }
                        break;
                    }

                    // 6. Request disconnect of UART PC -> 'X'
                    case 'X': 
                    {
                        session_active = false;
                        break;
                    }

                    default: 
                    {
                        break;
                    }
                }
                cmd = 0;
                delay(1);
            }
        }
    } 

    // Shut down Serial port
    Serial1.flush(); 
    Serial1.end(); 
    
    pinMode(TX_pin, INPUT_PULLDOWN);
    pinMode(RX_pin, INPUT);
    
    return reached_stage_3;
}

/******************************************************************************/
void handle_active_deployment(void)
{
  while (true)
  {
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY);

    // Read the pressure
    sensor.read();
    uint16_t pressure_value = (uint16_t)sensor.pressure();

    // Exit condition with hysteresis
    if (pressure_value <= STOP_THRESHOLD)
    {
      return; // deployment finished
    }

    // Log sample to FRAM
    save_pressure(pressure_value);

    // Sleep 10 seconds between samples
    go_to_sleep(measurement_interval);

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

  // 2. You have to request a read before you can read the register
  RTC->MODE2.READREQ.reg =
  RTC_READREQ_RREQ |
  RTC_READREQ_ADDR(0x10);
  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  //read RTC time value register
  uint32_t rtc_reg_value = RTC->MODE2.CLOCK.reg; 
  while (RTC->MODE2.STATUS.bit.SYNCBUSY);

  // 3. Convert to UNIX time
  struct tm t;

  // 1. Extract and unpack fields using masks and right-shifts
  uint8_t second =  rtc_reg_value & 0x3F;          // Bits 5:0
  uint8_t minute = (rtc_reg_value >> 6)  & 0x3F;   // Bits 11:6
  uint8_t hour   = (rtc_reg_value >> 12) & 0x1F;   // Bits 16:12
  uint8_t day    = (rtc_reg_value >> 17) & 0x1F;   // Bits 21:17
  uint8_t month  = (rtc_reg_value >> 22) & 0x0F;   // Bits 25:22
  uint8_t year   = (rtc_reg_value >> 26) & 0x3F;   // Bits 31:26

  // 4. Map the fields to standard struct tm requirements
  t.tm_sec  = second;                              // 0 to 59
  t.tm_min  = minute;                              // 0 to 59
  t.tm_hour = hour;                                // 0 to 23
  t.tm_mday = day;                                 // 1 to 31
  t.tm_mon  = month - 1;                           // time.h expects 0 (Jan) to 11 (Dec)
  
  // 5. Convert current year to a 1900-offset (e.g., 2026 - 1900 = 126)
  int full_year = 2000 + year;
  t.tm_year = full_year - 1900;                    // Years since 1900

  // 6. Set daylight saving flag to -1 so mktime tries to determine it automatically, 
  // or 0 if your hardware clock tracks strict UTC/Standard time.
  t.tm_isdst = 0; 

  // 7. Convert the populated struct tm into Unix epoch time (seconds since 1970)
  time_t unix_timestamp = mktime(&t);

  write_32(current_address + 2, unix_timestamp); 

  if (initial_pressure == 0xFFFF)
  {
    initial_pressure = 0xFFFE;
  }
  write_16(current_address + 6, initial_pressure);

  // 8. Advance RAM pointer by a total of 8 bytes
  current_address += 8;
  last_valid_address = current_address;

  // 9. Write to FRAM
  update_pointer();
}

/******************************************************/
void save_pressure(uint16_t pressure_value)
{
  // 1. Ensure room for 2 bytes
  if (current_address + 2 > FRAM_POINTER_ADDR)
  {
    current_address = 2;
    fram_wrapped = true;
  }

  // 2. Sanitize
  if (pressure_value == 0xFFFF)
  {
    pressure_value = 0xFFFE;
  }

  // 3. Write pressure sample
  write_16(current_address, pressure_value);

  current_address += 2;

  if (current_address > (FRAM_POINTER_ADDR - 2))
  {
    current_address = 2;
    fram_wrapped = true;
  }

  last_valid_address = current_address;

  update_pointer();
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

  // Write multipal bytes
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

  delay(1); // Tiny pause to let the I2C bus rest

  // 2. Write to Mirror Slot using multi-byte write
  fram.write(0x7FFE, buffer, 2);
}

/*************************************************************************************/

void recover_fram_address()
{
  uint8_t buffer[2];

  // 1. Safely grab Primary Pointer
  fram.read(FRAM_POINTER_ADDR, buffer, 2);
  uint16_t primary = ((uint16_t)buffer[0] << 8) | buffer[1];

  // 2. Safely grab Mirror Pointer
  fram.read(0x7FFE, buffer, 2);
  uint16_t mirror = ((uint16_t)buffer[0] << 8) | buffer[1];

  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);

  // Data region limits
  constexpr uint16_t DATA_START = 0x0002;
  constexpr uint16_t MAX_DATA_ADDR = FRAM_POINTER_ADDR - 2; // 0x7FFA

  auto is_valid = [&](uint16_t p)
  {
    // Must stay strictly inside data region, never into pointer storage
    if (p < DATA_START || p > MAX_DATA_ADDR)
    {
      return false;
    }

    // 2-byte alignment
    if (p & 0x0001)
    {
      // Severe corruption fallback
      return false;
    }

    // Individual sentinel checks
    if (p == 0x0000 || p == 0xFFFF)
    {
      return false;
    }

    return true;
  };

  // Check if new FRAM or erased FRAM
  if ((primary == 0x0000 && mirror == 0x0000) || (primary == 0xFFFF && mirror == 0xFFFF))
  {
    current_address = DATA_START;
    update_pointer();
    return;
  }

  bool primary_ok = is_valid(primary);
  bool mirror_ok = is_valid(mirror);

  if (primary_ok && mirror_ok)
  {
    // Both valid → choose the newer one
    if (primary >= mirror)
    {
      current_address = primary;
    }
    else
    {
      current_address = mirror;
    }

    if (primary != mirror)
    {
      update_pointer(); // self-heal disagreement
    }
  }
  else if (primary_ok)
  {
    current_address = primary;
    update_pointer(); // heal mirror
  }
  else if (mirror_ok)
  {
    current_address = mirror;
    update_pointer(); // heal primary
  }
  else
  {
    // Severe corruption fallback
    current_address = DATA_START;
    update_pointer();
  }
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

/******************************************************************/
// Global Hardware Interrupt Routine 
  void RTC_Handler(void) 
  { 
  if (RTC->MODE2.INTFLAG.bit.ALARM0) 
  { 
    RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0; // Clear interrupt flag
    alarmFired = true; 
  } 
}

/************This is for testing only*****************/
/**************Delete before uploading****************/
void inject_test_data()
{
  // 1. Reset write pointer to start of data region
  current_address = 2;
  fram_wrapped = false;
  last_valid_address = 2;

  // 2. Start a deployment 
  uint16_t safe_initial = 1025; 
  start_new_deployment(safe_initial);

  // 3. Write 20 fake samples (pressure ramp)
  for (int i = 0; i < 20; i++)
  {
    uint16_t fake_pressure = 2000 + (i * 100);
    save_pressure(fake_pressure); 
  }

  // 4. Update last_valid_address to the final write location
  last_valid_address = current_address;
}
