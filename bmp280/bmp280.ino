
#include <Wire.h>
#include "SPI.h" 
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP280.h"


Adafruit_BMP280 bmp; 



float pressure;    
float temperature;  
float altimeter;    

void setup() {
  bmp.begin();    
    Serial.begin(9600); 
    Serial.println("Adafruit BMP280 test:");
}

void loop() {
  pressure = bmp.readPressure()/100;
  temperature = bmp.readTemperature();
  altimeter = bmp.readAltitude (847); 

  
  Serial.print(F("Pressure: "));
    Serial.print(pressure);
    Serial.print(" Pa");
    Serial.print("\t");
    Serial.print(("Temp: "));
    Serial.print(temperature);
    Serial.print(" oC");
  Serial.print("\t");
    Serial.print("Altimeter: ");
    Serial.print(altimeter); 
    Serial.println(" m");
    
    delay(2000); 
}
