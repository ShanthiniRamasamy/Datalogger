#include <SerialFlash.h>
#include <SPI.h>
#include <Wire.h>
#include "SPI.h" //Why? Because library supports SPI and I2C connection
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP280.h"

Adafruit_BMP280 bmp; 
SerialFlashFile file;

const byte PIN_FLASH_CS = 6; 

char filename[16] ;

struct oneRecordType {
  uint32_t recordNumber;
  float temperature;
  float pressure;
  float altimeter;
  float moisture;
} oneRecord;

float pressure;    //To store the barometric pressure (Pa)
float temperature;  //To store the temperature (oC)
float altimeter;
float moisture;

const unsigned long eventInterval = 60000;
unsigned long previousTime = 0;

void setup() {
  bmp.begin();
  Serial.begin(115200);
  Serial.println(F("\n\n SPI Flash Data Logger "));

  if (!SerialFlash.begin( PIN_FLASH_CS )) {
    Serial.println(F("SPI Flash not detected."));
    while (1);
  }
  SerialFlash.createErasable( "data.txt", 16 );
  
  file = SerialFlash.open("data.txt");
  if ( !SerialFlash.open( "data.txt" )) {
    Serial.println(F("Flash could not open"));
  }
  else {
    Serial.println(F("Flash opened"));
}
  if ( !SerialFlash.exists( "data.txt" )) {
    Serial.println(F("Flash doesn't appear to hold a file system"));
  }
  else {
    Serial.println(F("Files currently in flash:"));
}
SerialFlash.opendir();
    while (1) {
      uint32_t filesize = 16;
      if (SerialFlash.readdir(filename, sizeof(filename),filesize)) {
        Serial.print(filename);
        Serial.print(F(" "));
        Serial.print(filesize);
        Serial.print(F(" bytes"));
        Serial.println();
      }
      else {
        break; // no more files
      }
    }
}
void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime <= eventInterval) {
     
      oneRecord.recordNumber++;

      oneRecord.temperature = bmp.readTemperature();
      oneRecord.pressure = bmp.readPressure() / 100;
      oneRecord.altimeter = bmp.readAltitude (847);
      oneRecord.moisture = analogRead(A0);

      // write the record to the flash chip
      file.write( (uint8_t *)&oneRecord.temperature, sizeof(oneRecord.temperature) );
      file.write( (uint8_t *)&oneRecord.pressure, sizeof(oneRecord.pressure) );
      file.write( (uint8_t *)&oneRecord.altimeter, sizeof(oneRecord.altimeter) );
      file.write( (uint8_t *)&oneRecord.moisture, sizeof(oneRecord.moisture) );
  
  //Print values to serial monitor:
  Serial.print(F("Pressure: "));
    Serial.print(oneRecord.pressure);
    Serial.print(" Pa");
    Serial.print("\t");
    Serial.print(("Temp: "));
    Serial.print(oneRecord.temperature);
    Serial.print(" oC");
  Serial.print("\t");
    Serial.print("Altimeter: ");
    Serial.print(oneRecord.altimeter); // this should be adjusted to your local forcase
    Serial.print(" m");
    Serial.print("\t");
    Serial.print("Moisture:");
    Serial.println(oneRecord.moisture);
    
    
    delay(3000); //Update every 3 sec
  }
}

    
