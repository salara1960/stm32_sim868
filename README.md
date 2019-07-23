#################################################################################################
#
# stm32f4 + sim868 (gsm,gps) + ssd1306(i2c&spi) + bmp280(i2c) + bh1750(i2c)
#
#################################################################################################


## The composition of the working equipment:

```
* stm32f4 (STM32F407GDISC1 board) - microcontroller board
* ssd1306 - OLED display 0.96 "128x64 (interfaces I2C, SPI)
* bmp280 - atmospheric pressure and air temperature sensor (I2C interface).
* bh1750 - light sensor (I2C interface).
* sim868 - card with SIM868 module (SIMCOM).
```


# Development Tools:

```
* STM32CubeMX - graphic package for creating projects (in C language) under the microcontrollers of the STM32 family
  (https://www.st.com/en/development-tools/stm32cubemx.html).
* System Workbench for STM32 - software development environment for microcontrollers of the STM32 family (IDE)
  (https://www.st.com/en/development-tools/sw4stm32.html).
```


# Functional:

* The device uses freeRTOS software:
  - StartAtTask - task, working with the command port of the SIM868 module.
  - STartSensTask - task, working with sensors bmp280, bh1750.
  - StartDefTask - task that performs the function of main and works with GPS data (NMEA) of the SIM868 module.
  - mailQueue - the queue to transfer data from bmp280 and bh1750 from the StartSensTask task to the StartDefTask task,
     to display data on the ssd1306 display and to output to the serial port (USART3).
  - binSemHandle - binary semaphore on USART3 port access (by appointment).
* The device initializes some microcontroller interfaces:
  - GPIO : four LEDs are used: PD12..PD15, one pin (PA3) on/off sim868, pin PA2 - VIO
  - I2C1 : master mode with a frequency of 400KHz (the bus serves ssd1306, bmp280, bh1750).
  - USART3 : parameters of port 115200 8N1 - a port for logging and transmitting AT commands to a SIM868 module
     if a computer is connected.
  - UART4 : parameters of port 9600 8N1 - the AT port of the SIM868 module commands.
  - TIM2 : Timer 250 ms. and 1 second, implemented in the callback function.
  - RTC : real time clock, can be set using the command :DATE=epoch_time
  - SPI3 : serves OLED SSD1306: RST (PC11), DC (PB6), CS (PB4), SCK (PB3), MOSI (PC12).
* The system timer (TIM1) counts milliseconds from the start of operation of the device.
* Data reception on all serial ports (USART3, UART4) is performed in the callback function of the interrupt handler.
  Received data is transferred to the corresponding tasks through structural queues.
* Every 30 seconds (or on an event) data from sensors bmp280 and bh1750 is read, atmospheric recalculation is performed
     pressure in mmHg and temperatures in degrees Celsius, the data obtained are issued in USART3, for example:

```
AT
OK
AT+CMEE=0
OK
AT+GMR
Revision:1418B05SIM868M32_BT
OK
AT+GSN
868183030452648
OK
AT+CNMI=1,2,0,1,0
OK
AT+SCLASS0=0;+CMGF=0
OK
AT+CGNSPWR=1
OK
AT+CGNSPWR?
+CGNSPWR: 1
OK
AT+CREG?
+CREG: 0,1
OK
AT+CSQ
+CSQ: 15,0
OK

:CON
AT+CIPSTART="TCP","aaa.bbb.ccc.ddd",9192
OK
000.00:00:09 | +++ CONNECTED +++
CONNECT OK

AT+CGNSINF
OK
000.00:00:11 | +CGNSINF: 1,0,19800106000706.000,,,,0.00,0.0,0,,,,,,0,0,,,,,
000.00:00:11 | BMP280: mmHg=761.61, DegC=26.25; BH1750: Lx=55.00
AT+CIPSEND=512
>{
        "InfSeqNum": 1,
        "MsgType": "+CGNSINF",
        "DevID": "868183030452648",
        "DevName": "STM32_SIM868",
        "SimNumber": "+79062100000",
        "DevTime": 11,
        "FreeMem": 18232,
        "UTC": "06.01.1980 00:07:06.000",
        "Status": "Invaid",
        "Latitude": 0.000000,
        "Longitude": 0.000000,
        "Altitude": 0,
        "Speed": 0.00,
        "Dir": 0.00,
        "HDOP": 0.00,
        "PDOP": 0.00,
        "VDOP": 0.00,
        "SatGPSV": 0,
        "SatGNSSU": 0,
        "SatGLONASSV": 0,
        "dBHz": 0,
        "Rssi": -83,
        "Press": 761.61,
        "Temp": 26.25,
        "Lux": 55.00
}

SEND OK
{"PackNumber":1,"InfSeqNum":1}

:DIS
AT+CIPCLOSE
000.00:01:16 | --- DISCONNECTED ---
CLOSE OK

```

  Some of this data (operating time, server address and connection status, atmospheric pressure, air temperature and light intensity) is displayed on the ssd1306 display.

* Via usart3, you can send commands to the SIM868 module, for example:

```
AT+GMR
Revision:1418B05SIM868M32_BT
OK
```

* The functional project in the process of replenishment.
