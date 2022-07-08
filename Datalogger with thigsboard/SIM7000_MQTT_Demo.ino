#include <ArduinoJson.h>
#include "Adafruit_FONA.h" // https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
#include <Wire.h>
#include "SPI.h" 
#include <Adafruit_Sensor.h>
#include <SerialFlash.h>
#include <SPI.h>
#include <Adafruit_AHT10.h>
#include <avr/sleep.h>
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>      // https://github.com/janelia-arduino/Streaming
#include <TimeLib.h>
/************************* PIN DEFINITIONS *********************************/
// For botletics SIM7000 shield
#define FONA_PWRKEY 6
#define FONA_RST 7
//#define FONA_DTR 8 // Connect with solder jumper
//#define FONA_RI 9 // Need to enable via AT commands
#define FONA_TX 8// Microcontroller RX
#define FONA_RX 9// Microcontroller TX
//#define T_ALERT 12 // Connect with solder jumper

#define LED 13 // Just for testing if needed!

#define samplingRate 3 // The time we want to delay after each post (in seconds)

// We default to using software serial.
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

SoftwareSerial *fonaSerial = &fonaSS;



Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
Adafruit_AHT10 aht;
SerialFlashFile file;

/************************* MQTT PARAMETERS *********************************/
//If you are using thingsboard MQTT broker 
#define TOKEN            "1JI9SLw1j3job0yBbbwq" //device access token
#define THINGSBOARD_SERVER  "demo.thingsboard.io"
#define THINGSBOARD_PORT    1883

//If you are using any other MQTT broker
//#define MQTT_SERVER      "broker.mqttdashboard.com"
//#define MQTT_PORT        1883
//#define MQTT_USERNAME    "shanmqtt"
//#define MQTT_PASSWORD    "Godisgreat@2001"

// Set topic names to publish and subscribe to
//#define TEMP_TOPIC       "temper"
//#define SUB2_TOPIC       "temper"

// How many transmission failures in a row we're OK with before reset
//uint8_t txfailures = 0;

/****************************** OTHER STUFF ***************************************/

uint8_t type;

//Serial Flash Chip Select pin
const byte PIN_FLASH_CS = 10; 

char filename[16] ;

struct oneRecordType {
  uint32_t recordNumber;
  char temperature;
  char humidity;
  char moisture;
  char time;
} oneRecord;

    
 float temperature;  
 float humidity;
 float moisture;
 
//Create a Json object
  StaticJsonDocument<80> doc;
JsonObject object;

//How long you want the event to happen (Here it's 2 min)
const unsigned long eventInterval = 120000;
unsigned long previousTime = 0;

constexpr uint32_t wakeInterval {2};    // wake interval in minutes
constexpr uint32_t ledInterval {10};    // seconds to leave the LED on after an alarm
static_assert(ledInterval < wakeInterval*60, "ledInterval must be less than wakeInterval!");
constexpr uint8_t rtcIntPin {2};        // RTC interrupt. Pin 2 for INT0, Pin 3 for INT1.
constexpr uint8_t  rtcPower {5};        // RTC power is supplied by the MCU
volatile bool rtcAlarm {false};         // ISR flag
DS3232RTC myRTC;

char replybuffer[255];

void setup() {
  Serial.begin(9600);
  Serial.println(F("*** SIM7000 MQTT Example ***"));
  Wire.begin();
  if (! aht.begin()) {
    Serial.println(F("Could not find AHT10? Check wiring"));
  }
  Serial.println(F("AHT10 found"));

   object = doc.to<JsonObject>();
   
  moduleSetup(); // Establishes first-time serial comm and prints IMEI

  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

 
  fona.setNetworkSettings(F("hologram")); //(give your APN in " ")

  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Connected to cell network!"));

  // Disable data just to make sure it was actually off so that we can turn it on
  fona.openWirelessConnection(false);

  // Open wireless connection if not already activated
  while (!fona.wirelessConnStatus()) {
    if (!fona.openWirelessConnection(true)) {
      Serial.println(F("Failed to enable connection, retrying..."));
      delay(2000); // Retry every 2s
  }
   
  else {
    Serial.println(F("Data already enabled!"));
  }
  }

   if (! fona.MQTT_connectionStatus()) {
    // Set up MQTT parameters (see MQTT app note for explanation of parameter values)
    fona.MQTT_setParameter("URL", THINGSBOARD_SERVER, THINGSBOARD_PORT);//(for MQTT change to MQTT_SERVER , MQTT _PORT)
    // Set up MQTT username and password if necessary
    fona.MQTT_setParameter("USERNAME", TOKEN);
//    fona.MQTT_setParameter("PASSWORD", MQTT_PASSWORD);
//    fona.MQTTsetParameter("KEEPTIME", 30); // Time to connect to server, 60s by default
//    
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
  if ( SerialFlash.open( "dummy1.txt" )) {
    Serial.println(F("Flash opened"));
  }
  Serial << F( "\n" __FILE__ "\n" __DATE__ " " __TIME__ "\n" );
  myRTC.begin();
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(rtcPower, OUTPUT);
    digitalWrite(rtcPower, HIGH);

    // enable pullup on interrupt pin (RTC SQW pin is open drain)
    pinMode(rtcIntPin, INPUT_PULLUP);

    // set up the interrupt handler
    attachInterrupt(digitalPinToInterrupt(rtcIntPin), rtcInterrupt, FALLING);

    myRTC.begin();
    Serial << F("Sketch start ") << F("\n" __DATE__ " " __TIME__ "\n" );
    Serial << F("Wake interval: ") << wakeInterval << F(" minutes.\n");
    Serial << F("Using interrupt INT") << digitalPinToInterrupt(rtcIntPin)<<endl ;
    Serial << F("RTC interrupt pin: ") << rtcIntPin <<endl;
    Serial << F("RTC power pin: ") << rtcPower <<endl;

    // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
    myRTC.setAlarm(DS3232RTC::ALM1_MATCH_DATE, 0, 0, 0, 1);
    myRTC.setAlarm(DS3232RTC::ALM2_MATCH_DATE, 0, 0, 0, 1);
    myRTC.alarm(DS3232RTC::ALARM_1);
    myRTC.alarm(DS3232RTC::ALARM_2);
    myRTC.alarmInterrupt(DS3232RTC::ALARM_1, false);
    myRTC.alarmInterrupt(DS3232RTC::ALARM_2, false);
    myRTC.squareWave(DS3232RTC::SQWAVE_NONE);  

}
 
void loop() {
  enum states_t { wait, alarm, ledOn, setAlarm };
    static states_t state { setAlarm };
    static uint32_t msLED;
    
    switch (state)
    {
        case wait:
            Serial << F("Going to sleep at ");printTime(myRTC.get());Serial.flush();
            digitalWrite(rtcPower, LOW);
            pinMode(rtcPower, INPUT);       // high-z, turn off power to the rtc
            pinMode(SDA, INPUT);            // high-z for the i2c bus
            pinMode(SCL, INPUT);
            goToSleep();
            pinMode(rtcPower, OUTPUT);
            digitalWrite(rtcPower, HIGH);
            delay(20);                      // a little wake-up time for the rtc & i2c
            if (rtcAlarm) { state = alarm; }
            break;

        case alarm:
            state = ledOn;
            Serial << F("Alarm at ");printTime(myRTC.get());
            myRTC.alarm(DS3232RTC::ALARM_1);    // clear alarm flag
            myRTC.alarmInterrupt(DS3232RTC::ALARM_1, false);
            noInterrupts();
            rtcAlarm = false;
            interrupts();
            msLED = millis();
            digitalWrite(LED_BUILTIN, HIGH);
            break;
            
         case ledOn:
            if (millis() - msLED >= ledInterval * 1000) {
                state = setAlarm;
                digitalWrite(LED_BUILTIN, LOW);
    
          unsigned long currentTime = millis();
          if (currentTime - previousTime <= eventInterval) {
            sensors_event_t humidity, temp;
      aht.getEvent(&humidity, &temp);
          
      oneRecord.recordNumber++;
      
      oneRecord.time = time_t();
      oneRecord.temperature = temp.temperature;
      oneRecord.humidity = humidity.relative_humidity;
      oneRecord.moisture = analogRead(A0);
      // write the record to the flash chip
      file.write( (uint8_t *)&oneRecord.time, sizeof(oneRecord.time) );
      file.write( (uint8_t *)&oneRecord.temperature, sizeof(oneRecord.temperature) );
      file.write( (uint8_t *)&oneRecord.humidity, sizeof(oneRecord.humidity) );
      file.write( (uint8_t *)&oneRecord.moisture, sizeof(oneRecord.moisture) );


  doc["time"]=oneRecord.time;
  doc["temp"]=oneRecord.temperature;
  doc["humi"]=oneRecord.humidity;
  doc["moist"]=oneRecord.moisture;

  char jsonStr[60];
  serializeJson(doc,jsonStr);
//  Serial.print(jsonStr);
  Serial.println();


  fona.MQTT_publish( "v1/devices/me/telemetry", jsonStr ,strlen(jsonStr),0,0);

  fona.MQTT_subscribe("v1/devices/me/rpc/request/+",0);

//   If not already connected and publish/subscribe to MQTT
//  
//  fona.MQTT_subscribe(SUB2_TOPIC, 1); // topic name , QOS

  // Parameters for MQTT_publish: Topic, message (0-512 bytes), message length, QoS (0-2), retain (0-1)
//  if (!fona.MQTT_publish(TEMP_TOPIC, jsonStr, strlen(jsonStr), 2, 0)) Serial.println(F("Failed to publish!")); 

  // Disconnect from MQTT
//  fona.MQTT_connect(false);
  
  
    // The rest of the time, read anything coming over via UART from the SIM7000
    // If it's from an MQTT subscribed topic message, parse it
    uint8_t i = 0;
    if (fona.available()) {
      while (fona.available()) {
        replybuffer[i] = fona.read();
        i++;
      }

      Serial.print(replybuffer); // DEBUG
      delay(100); // Make sure it prints and also allow other stuff to run properly

      // We got an MQTT message! Parse the topic and message
      // Format: +SMSUB: "topic_name","message"
      if (strstr(replybuffer, "+SMSUB:") != NULL) {
        Serial.println(F("*** Received MQTT message! ***"));
        
        char *p = strtok(replybuffer, ",\"");
        char *topic_p = strtok(NULL, ",\"");
        char *message_p = strtok(NULL, ",\"");
        
        Serial.print(F("Topic: ")); Serial.println(topic_p);
        Serial.print(F("Message: ")); Serial.println(message_p);
}
}
//   Delay until next post
 Serial.print(F("Waiting for ")); Serial.print(samplingRate); Serial.println(F(" seconds\r\n"));
  delay(samplingRate * 1000UL);//delay
  }
             
break;

        case setAlarm:
            state = wait;
            time_t utc = myRTC.get();
            time_t wake = wakeInterval * 60;
            time_t alm = utc - utc % wake + wake;
            Serial << F("Setting alarm for "); printTime(alm);
            myRTC.setAlarm(DS3232RTC::ALM1_MATCH_DATE, second(alm), minute(alm), hour(alm), day(alm));
            myRTC.alarmInterrupt(DS3232RTC::ALARM_1, true);
            break;
    }
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
void rtcInterrupt()
{
    rtcAlarm = true;
}

// format and print a time_t value
void printTime(time_t t)
{ 
    char buf[25];
    char m[4];    // temporary storage for month string (DateStrings.cpp uses shared buffer)
    strcpy(m, monthShortStr(month(t)));
    sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d",
        hour(t), minute(t), second(t), dayShortStr(weekday(t)), day(t), m, year(t));
    Serial.println(buf);
}

void goToSleep()
{
    uint8_t adcsra = ADCSRA;        // save the ADC Control and Status Register A
    ADCSRA = 0;                     // disable the ADC
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();                          // stop interrupts to ensure the BOD timed sequence executes as required
    sleep_enable();
    sleep_bod_disable();            // disable brown-out detection while sleeping (20-25µA)
    sei();                          // ensure interrupts enabled so we can wake up again
    sleep_cpu();                    // go to sleep
    sleep_disable();                // wake up here
    ADCSRA = adcsra;                // restore ADCSRA
}
