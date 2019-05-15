###########################################################################################
#
# stm32f4 + sim868 (gsm, gps, bluetooth) + ssd1306(i2c) + bmp280(i2c) + bh1750(i2c)
#
###########################################################################################


## Состав рабочего оборудования:

```
* stm32f4 (stm32f4Disc1 board) - плата микроконтроллера
* ssd1306 - OLED дисплей 0.96" 128x64 (интерфейс I2C)
* bmp280 - датчик атмосферного давления и температуры воздуха (интерфейс I2C).
* bh1750 - датчик освещенности (интерфейс I2C).
* sim868 - плата с модулем SIM868 (SIMCOM).
```


# Средства разработки:

```
* STM32CubeMX - графический пакет для создание проектов (на языке Си) под микроконтроллеры семейства STM32
  (https://www.st.com/en/development-tools/stm32cubemx.html).
* System Workbench for STM32 - IDE среда разработки ПО для микроконтроллеров семейства STM32
  (https://www.st.com/en/development-tools/sw4stm32.html).
``


# Функционал:

```
* Устройство использует программные средства freeRTOS :
  - StartGpsTask - нитка (задача), работающая с данными GPS (NMEA) модуля SIM868.
  - StartAtTask - нитка (задача), работающая портом команд модуля SIM868.
  - STartSensTask - нитка (задача), работающая с датчиками bmp280, bh1750.
  - StartDefTask - нитка (задача), выполняющая функцию main.
  - mailQueue - очередь для передачи данных от bmp280 и bh1750 из задачи StartSensTask в задачу StartDefTask,
    для вывода данных на дисплей ssd1306 и для вывода в последовательный порт (USART3).
  - binSemHandle - бинарный семафор на доступ к порту usart3 (по записи).
* Устройство инициализирует некоторые интерфейсы микроконтроллера :
  - GPIO : подключены два сетодиода : PD12..PD15.
  - I2C1 : режим мастера с частотой 400Кгц (шина ослуживает ssd1306, bmp280, bh1750).
  - USART3 : параметры порта 115200 8N1 - порт для логов и передачи AT команд модулю SIM868, если подключен комп.
  - UART4 : параметры порта 9600 8N1 - порт AT команд модуля SIM868
  - UART5 : параметры порта 38400 8N1 - порт для приема данных GPS (NMEA) от модуля SIM868
  - TIM2 : таймер-счетчик временных интервалов в 1 секунду, реализован в callback-функции.
* Системный таймер (TIM1) считает миллисекунды от начала работы устройства.
* Прием данных по всем последовательным портам (USART3, UART4, UART5) выполняется в callback-функции обработчика прерывания.
  Принятые данные передаются в соответствующие задачи (нитки) через структурные очереди.
* Каждые 10 секунд считываются данные с датчиков bmp280 и bh1750, выполняется пересчет атмосферного
  давления в мм ртутного столба и температуры в градусы Цельсия, полученные данные выдаются
  в USART3, например :
000.01:02:57 | BMP280: Press=763.45 mmHg, Temp=21.82 DegC; BH1750: Lux=21.66 lx
000.01:03:07 | BMP280: Press=763.46 mmHg, Temp=21.83 DegC; BH1750: Lux=20.83 lx
000.01:03:17 | BMP280: Press=763.47 mmHg, Temp=21.84 DegC; BH1750: Lux=20.83 lx
  Эти же данные (время работы, напряжение питания, атмосферное давление, температура воздуха и освещенность)
отображаются на дисплей ssd1306.
* Через usart5 можно отправлять команды на модуль SIM868, например :
AT+GMR
Revision:1418B03SIM868M32_BT
OK
* Порт usrt5 предназначен для приема данных GPS (NMEA) от модуля SIM868, например :
000.01:00:46 | $GNRMC,010025.869,V,,,,,0.00,0.00,060180,,,N*5D
        UTC=06.01.80 01:00:25.869 'Invaid data'
        Latitude=0.000000^ (North)
        Longitude=0.000000^ (East)
        Speed=0.00 km/h
        Dir=0.00^

* Функционал проекта в процессе пополнения.
```
