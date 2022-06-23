#include <ArduinoJson.h>
#include "Adafruit_FONA.h" // https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
#include <Wire.h>
#include "SPI.h" 
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP280.h"
#include <SerialFlash.h>
#include <SPI.h>


/************************* PIN DEFINITIONS *********************************/
// For botletics SIM7000 shield
#define FONA_PWRKEY 6
#define FONA_RST 7
//#define FONA_DTR 8 // Connect with solder jumper
//#define FONA_RI 9 // Need to enable via AT commands
#define FONA_TX 8// Microcontroller RX
#define FONA_RX 9// Microcontroller TX
//#define T_ALERT 12 // Connect with solder jumper

//#define LED 13 // Just for testing if needed!

#define samplingRate 3 // The time we want to delay after each post (in seconds)

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);



SoftwareSerial *fonaSerial = &fonaSS;


//Adafruit_FONA fona = Adafruit_FONA(99);
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

Adafruit_BMP280 bmp;
SerialFlashFile file;

/************************* MQTT PARAMETERS *********************************/
#define MQTT_SERVER      "broker.mqttdashboard.com"
#define MQTT_PORT        1883
#define MQTT_USERNAME    "shanmqtt"
#define MQTT_PASSWORD    "Godisgreat@2001"
//const char *protocol ="MQTT/TCP";
//const char *clientID ="482fd4aa-8649-4036-8099-8e870398a38b";
//const char *username =" ";
//const char *password =" ";
//const char *topic="ttqmtest";
//const char *message="hello";
// Set topic names to publish and subscribe to
#define TEMP_TOPIC       "temper"
//#define MOIST_TOPIC      "moisture"
//#define PRESS_TOPIC      "pressure"
//#define ALTI_TOPIC      "altitude"
//#define SUB1_TOPIC       "moisture"
#define SUB2_TOPIC       "temper"
//#define SUB3_TOPIC       "pressure"
//#define SUB4_TOPIC       "altitude"// Subscribe topic name

// How many transmission failures in a row we're OK with before reset
uint8_t txfailures = 0;

/****************************** OTHER STUFF ***************************************/
// For temperature sensor
//#include <Wire.h>
//#include "Adafruit_MCP9808.h"

// Create the MCP9808 temperature sensor object
//Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
char imei[16] = {0}; // Use this for device ID
uint8_t type;
//uint16_t battLevel = 0; // Battery level (percentage)
//float latitude, longitude, speed_kph, heading, altitude, second;
//uint16_t year;
//uint8_t month, day, hour, minute;
uint8_t counter = 0;
//char PIN[5] = "1234"; // SIM card PIN
const byte PIN_FLASH_CS = 10; 

char filename[16] ;
//
struct oneRecordType {
  uint32_t recordNumber;
  char temperature;
  char pressure;
  char altimeter;
  char moisture;
} oneRecord;

 float pressure;    
 float temperature;  
 float altimeter;
 float moisture;

  StaticJsonDocument<80> doc;

 JsonObject object;

 char Data[150];

const unsigned long eventInterval = 60000;
unsigned long previousTime = 0;


// NOTE: Keep the buffer sizes as small as possible, espeially on
// Arduino Uno which doesn't have much computing power to handle
// large buffers. On Arduino Mega you shouldn't have to worry much.
//char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
//     headBuff[12], altBuff[12], moistBuff[12], battBuff[12],
//  char moistureBuff[6],temperatureBuff[6],pressureBuff[6],altimeterBuff[6];

void setup() {
  Serial.begin(9600);
   bmp.begin(); 
//  initBaud();
  Serial.println(F("*** SIM7000 MQTT Example ***"));

   object = doc.to<JsonObject>();

//  #ifdef LED
//    pinMode(LED, OUTPUT);
//    digitalWrite(LED, LOW);
//  #endif
  
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state

  fona.powerOn(FONA_PWRKEY); // Power on the module
  moduleSetup(); // Establishes first-time serial comm and prints IMEI



  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

 
  fona.setNetworkSettings(F("hologram")); 

  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Connected to cell network!"));
  Serial.println("no problem");

  // Disable data just to make sure it was actually off so that we can turn it on
  fona.openWirelessConnection(false);

  // Open wireless connection if not already activated
  while (!fona.wirelessConnStatus()) {
    Serial.println("No problem");
//    fona.openWirelessConnection(true);
    if (!fona.openWirelessConnection(true)) {
      Serial.println("no problem!");
      Serial.println(F("Failed to enable connection, retrying..."));
      delay(2000); // Retry every 2s
    
//    while (!fona.wirelessConnStatus()) {
    Serial.println("Enabled data!");
  }
   
  else {
    Serial.println(F("Data already enabled!"));
  }
  }
   if (! fona.MQTT_connectionStatus()) {
    // Set up MQTT parameters (see MQTT app note for explanation of parameter values)
    fona.MQTT_setParameter("URL", MQTT_SERVER, MQTT_PORT);
    // Set up MQTT username and password if necessary
    fona.MQTT_setParameter("USERNAME", MQTT_USERNAME);
    fona.MQTT_setParameter("PASSWORD", MQTT_PASSWORD);
//    fona.MQTTsetParameter("KEEPTIME", 30); // Time to connect to server, 60s by default
    
    Serial.println(F("Connecting to MQTT broker..."));
    if (! fona.MQTT_connect(true)) {
      Serial.println(F("Failed to connect to broker!"));
    }
  }
  else {
    Serial.println(F("Already connected to MQTT server!"));
  }

  
  if (!SerialFlash.begin( PIN_FLASH_CS )) {
    Serial.println(F("SPI Flash not detected."));
    while (1);
  }
  SerialFlash.createErasable( "dummy1.txt", 16 );
  
  file = SerialFlash.open("dummy1.txt");
  if ( !SerialFlash.open( "dummy1.txt" )) {
    Serial.println(F("Flash could not open"));
  }
  else {
    Serial.println(F("Flash opened"));
}
  if ( !SerialFlash.exists( "dummy1.txt" )) {
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
  // Connect to cell network and verify connection
  // If unsuccessful, keep retrying every 2s until a connection is made
//  while (!netStatus()) {
//    Serial.println(F("Failed to connect to cell network, retrying..."));
//    delay(2000); // Retry every 2s
//  }
//  Serial.println(F("Connected to cell network!"));
//  Serial.println("no problem");
//
//  // Disable data just to make sure it was actually off so that we can turn it on
//  fona.openWirelessConnection(false);
//
//  // Open wireless connection if not already activated
//  while (!fona.wirelessConnStatus()) {
//    Serial.println("No problem");
////    fona.openWirelessConnection(true);
//    if (!fona.openWirelessConnection(true)) {
//      Serial.println("no problem!");
//      Serial.println(F("Failed to enable connection, retrying..."));
//      delay(2000); // Retry every 2s
//    
////    while (!fona.wirelessConnStatus()) {
//    Serial.println("Enabled data!");
//  }
//   
//  else {
//    Serial.println(F("Data already enabled!"));
//  }
//  }
 
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

//      Serial.print(F("Pressure: "));
//    Serial.print(oneRecord.pressure);
//    Serial.print(" Pa");
//    Serial.print("\t");
//    Serial.print(("Temp: "));
//    Serial.print(oneRecord.temperature);
//    Serial.print(" oC");
//  Serial.print("\t");
//    Serial.print("Altimeter: ");
//    Serial.print(oneRecord.altimeter); // this should be adjusted to your local forcase
//    Serial.print(" m");
//    Serial.print("\t");
//    Serial.print("Moisture:");
//    Serial.println(oneRecord.moisture);

// pressure = bmp.readPressure()/100;
//  temperature = bmp.readTemperature();
//  altimeter = bmp.readAltitude (847);
//  moisture = analogRead(A0);

//   object["temp"] = temperature;
//object["press"] = pressure;
//object["alti"] = altimeter;
//object["moist"] = moisture;
//serializeJson(doc, Data);
//serializeJson(doc, Serial);
//Serial.println();

  doc["temp"]=oneRecord.temperature;
  doc["press"]=oneRecord.pressure;
  doc["alti"]=oneRecord.altimeter;
  doc["moist"]=oneRecord.moisture;

  char jsonStr[60];
  serializeJson(doc,jsonStr);
//  Serial.print(jsonStr);
  Serial.println();

//  Serial.print("Pressure: ");
//    Serial.print(pressure);
//    Serial.print("Temp: ");
//    Serial.print(temperature);
//    Serial.print("Altimeter: ");
//    Serial.print(altimeter); 
//    Serial.print("MOIST:");
//    Serial.print(moisture);


  
//moisture = analogRead(A0);
//Serial.print("MOIST:");Serial.print(moist);

 
//  Serial.println(F("---------------------"));

  // Format the floating point numbers
//  dtostrf(latitude, 1, 6, latBuff); // float_val, min_width, digits_after_decimal, char_buffer
//  dtostrf(longitude, 1, 6, longBuff);
//  dtostrf(temperature, 1, 0,temperatureBuff );
//  dtostrf(pressure, 1, 1, pressureBuff);
//  dtostrf(altimeter, 1, 1, altimeterBuff);
//  dtostrf(moisture, 1, 1, moistureBuff);
//  dtostrf(battLevel, 1, 0, battBuff);

  // Construct a combined, comma-separated location array
//  sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff); // This could look like "10,33.123456,-85.123456,120.5"


  
//   If not already connected, connect to MQTT
//  if (! fona.MQTT_connectionStatus()) {
//    // Set up MQTT parameters (see MQTT app note for explanation of parameter values)
//    fona.MQTT_setParameter("URL", MQTT_SERVER, MQTT_PORT);
//    // Set up MQTT username and password if necessary
//    fona.MQTT_setParameter("USERNAME", MQTT_USERNAME);
//    fona.MQTT_setParameter("PASSWORD", MQTT_PASSWORD);
////    fona.MQTTsetParameter("KEEPTIME", 30); // Time to connect to server, 60s by default
//    
//    Serial.println(F("Connecting to MQTT broker..."));
//    if (! fona.MQTT_connect(true)) {
//      Serial.println(F("Failed to connect to broker!"));
//    }
//  }
//  else {
//    Serial.println(F("Already connected to MQTT server!"));
//  }
//  fona.MQTT_subscribe(SUB1_TOPIC, 1);
  fona.MQTT_subscribe(SUB2_TOPIC, 1);
//  fona.MQTT_subscribe(SUB3_TOPIC, 1);
//  fona.MQTT_subscribe(SUB4_TOPIC, 1);
  // Now publish all the GPS and temperature data to their respective topics!
  // Parameters for MQTT_publish: Topic, message (0-512 bytes), message length, QoS (0-2), retain (0-1)
  if (!fona.MQTT_publish(TEMP_TOPIC, jsonStr, strlen(jsonStr), 2, 0)) Serial.println(F("Failed to publish!")); // Send GPS location
//  if (!fona.MQTT_publish(MOIST_TOPIC, moistureBuff, strlen(moistureBuff), 2, 0)) Serial.println(F("Failed to publish!")); // Send temperature
//  if (!fona.MQTT_publish(PRESS_TOPIC, pressureBuff, strlen(pressureBuff), 2, 0)) Serial.println(F("Failed to publish!")); // Send battery level
//  if (!fona.MQTT_publish(ALTI_TOPIC, altimeterBuff, strlen(altimeterBuff), 2, 0)) Serial.println(F("Failed to publish!"));
//  fona.MQTT_subscribe(SUB_TOPIC, 1); // Topic name, QoS
  
  // Unsubscribe to topics if wanted:
//  fona.MQTT_unsubscribe(SUB_TOPIC);

  // Enable MQTT data format to hex
//  fona.MQTT_dataFormatHex(true); // Input "false" to reverse

  // Disconnect from MQTT
//  fona.MQTT_connect(false);

  // Delay until next post
  Serial.print(F("Waiting for ")); Serial.print(samplingRate); Serial.println(F(" seconds\r\n"));
  delay(samplingRate * 1000UL); // Delay
}
}



void moduleSetup() {
  // SIM7000 takes about 3s to turn on and SIM7500 takes about 15s
  // Press Arduino reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset

  // Software serial:
  fonaSS.begin(115200); // Default SIM7000 shield baud rate

  Serial.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    while (1); // Don't proceed if it couldn't find the device
  }

 

  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM800L:
      Serial.println(F("SIM800L")); break;
    case SIM800H:
      Serial.println(F("SIM800H")); break;
    case SIM808_V1:
      Serial.println(F("SIM808 (v1)")); break;
    case SIM808_V2:
      Serial.println(F("SIM808 (v2)")); break;
    case SIM5320A:
      Serial.println(F("SIM5320A (American)")); break;
    case SIM5320E:
      Serial.println(F("SIM5320E (European)")); break;
    case SIM7000:
      Serial.println(F("SIM7000")); break;
    case SIM7070:
      Serial.println(F("SIM7070")); break;
    case SIM7500:
      Serial.println(F("SIM7500")); break;
    case SIM7600:
      Serial.println(F("SIM7600")); break;
    default:
      Serial.println(F("???")); break;
  }
  
  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
}



bool netStatus() {
  int n = fona.getNetworkStatus();
  
  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) return false;
  else return true;
}
