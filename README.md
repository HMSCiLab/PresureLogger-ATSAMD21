
 
# Ocean Pressure Logger

## 1. Description

The Ocean Pressure Logger is a battery-powered, potted, disposable data logger designed to record underwater pressure during repeated deployments.
The unit is permanently sealed. The battery is not replaceable. When the battery is exhausted, the unit is discarded.

2. Design Goals
       
Lowest possible power consumption
Reliable data collection
Accurate timestamps
Data integrity
Electrolysis prevention
Simple communication interface (UART)

3. Hardware

Controller:
Seeed Studio
Seeedunio XIAO M0 (SAMD21)

Memory:
Adafruit
I�C 32k FRAM

Sensor:
TE Connectivity Measurement Specialties
MS583730BA01-50

UART/USB Bridge:  
`Sparkfun FTDI RS232`


4. Operating States 

Shelf
The device will wake from deep sleep every 5 seconds and check whether it is connect to the UART/USB bridge or in the water

UART
While connected to the UART/USB bridge it will stay awake and respond to commands sent by a UI application

Deployment
When in the water the device will wake from deep sleep and log pressure measurements every 10 seconds



5. BOOT 

Executed only once after power-up
Responsibilities: 

Configure hardware 
Initialize RTC 
Set the RTC time to compile time and adjust for UTC
Initialize FRAM 
Initialize sensor 
Configure watchdog 
Configure low-power peripherals 
Enter CHECK_UART

6. CHECK UART

Purpose: 
Determine whether the UART/USB bridge is attached
Detection: RX HIGH means UART bridge connected 
If connected handle_uart_session() is entered
The processor remains awake until RX LOW indicates cable removal. 
If not connected proceed to READ_SENSOR

7. READ SENSOR 

Purpose:
Read one pressure sample
If above deployment threshold ACTIVE_DEPLOYMENT 
Otherwise SLEEP 

8. ACTIVE DEPLOYMENT 

At the beginning of each deployment record a timestamp start_new_deployment()

Purpose:
Every wake:
Read pressure 
Validate reading 
Store sample 
Sleep for deployment interval 

9. SLEEP 
Enter lowest practical power state. 
Wake source: 
RTC only. 
Wake interval: 
Normally 5 seconds 
Deployment interval 10 seconds





10. Communications UART/USB bridge

Purpose:
Download FRAM 
Erase FRAM 
Future:
Sync time
Back door

11. Current Logic

'P' PC -> Arduino "Connect"
'H' Arduino -> PC "Handshake"
'C' PC -> Arduino connected "Send the voltage"
'V' arduino -> PC "Sent the voltage"
'D' PC -> Arduino "Request send the data"
'A' PC -> Arduino "Processed the data"
'L' Arduino -> PC "Waiting for 'W' to erase data or 'X' to disconnect"
'W' Wipe �PC -> Arduino "Erase the memory."
'E' Arduino -> PC "Memory erased" waiting for 'X'
'X' PC -> Arduino "Close the port and go to sleep"
�T� PC -> Arduino  �Sync RTC time� furture
'B' PC -> Arduino "Backdoor" future

12. Data Flow

0xAA 0x55 �STX Arduino -> PC �"Start of Text" (Binary start)
0x55 0xAA �ETX Arduino -> PC �"End of Text" (Binary end)
CRC Arduino -> PC

13. RTC Set Time

For true UTC/GMT unix time an offset must be added to the compile time
Eastern Time (ET): Standard is UTC?05:00; Daylight is UTC?04:00
Central Time (CT): Standard is UTC?06:00; Daylight is UTC?05:00
Mountain Time (MT): Standard is UTC?07:00; Daylight is UTC?06:00
Pacific Time (PT): Standard is UTC?08:00; Daylight is UTC?07:00
Alaska Time (AKT): Standard is UTC?09:00; Daylight is UTC?08:00
Hawaii-Aleutian Time (HAT): Standard is UTC?10:00



14. Variables

UTC_OFFSET_HOURS = 7;	***Besure to set your timezone*** e.g. PDT 07/10/2026 = 7
standby_seconds = 5;		The amount of time (in seconds) the logger sleeps when not deployed
measurement_interval = 10;	The amount of time (in seconds) between pressure measurements
fram_size = 32768;			Change if a different size FRAM is used
START_THRESHOLD = 2000;	Changes when logging begins
STOP_THRESHOLD = 1100;	Recommend, don�t change. The highest recorded high pressure in history was 					1080mb. Any pressure above that is assumed to be under water.                          

15. Connections

SDA pin 4
SCL pin 5 �
RX pin 7 �
TX pin 6 �
ADC pin 10    R1 = 1M �R2 = 1.5M
