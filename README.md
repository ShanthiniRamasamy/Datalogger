# Datalogger

Using AHT10 sensor and Capacitive Soil Moisture sensor the data like temperature, humidity and moisture values are taken and stored in SPI Flash module and using SIM7000 GSM Module we publish the data in a MQTT Server at a particular time in regular intervals when the arduino is awake using RTC.Arduino sleep and wake up mode is also used here and protocols like UART,SPI,I2C,MQTT,RPC are infered and used where from the cloud these data can be used for data analytics.

## Components

* Arduino Nano
* FTDI UART module
* AHT10 temperature and humidity sensor
* Capacitive Soil Moisture Sensor
* SPI Flash Module
* SIM7000 GSM module
* DS3231 RTC module
* Jumpers and USB cable for connection

## Arduino Libraries 

* ArduinoJson - for the Json format of the data
* [Adafruit_fona(Botletics)](https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code) - for the configuration of SIM7000
* SerialFlash - for the configuration of SPI Flash module
* Adafruit_AHT10 - for the configuration of AHT10 sensor
* [DS3232RTC]( https://github.com/JChristensen/DS3232RTC) - for the configuration of RTC
