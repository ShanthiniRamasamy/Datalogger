#include <SerialFlash.h>
#include <SPI.h>

#define FILE_SIZE_512K 524288L

SerialFlashFile file;

const byte PIN_FLASH_CS = 13; 

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n\n SPI Flash Data Logger "));

  if (!SerialFlash.begin( PIN_FLASH_CS )) {
    Serial.println(F("SPI Flash not detected."));
    while (1);
  }

  SerialFlash.create(datalogger, size);

  if ( !SerialFlash.exists( "datalogger" )) {
    Serial.println(F("Flash doesn't appear to hold a file system"));
  }
  else {
    Serial.println(F("Files currently in flash:"));
  }
}
    
