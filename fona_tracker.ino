
#include "Adafruit_FONA.h"

// standard pins for the shield, adjust as necessary
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

// Initial State defines
#define ISAccessKey "CHANGE_THIS_TO_YOUR_ACCESS_KEY"
String bucketKey = "";
String urlString = "";
char urlArray[1000];

String keyValuesToSend = "";
String keysToSend[8];
String valuesToSend[8];


// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Have a FONA 3G? use this object type instead
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);


void createBucket() {
  uint16_t statuscode;
  int16_t length;
  urlString = "http://insecure-groker.initialstate.com/api/buckets?accessKey="+String(ISAccessKey)+"&bucketKey="+bucketKey;
  Serial.println(urlString);
  
  //char *chararray = malloc(sizeof(char)*urlString.length()+1);
  //urlString.toCharArray(urlArray, urlString.length()+1);
  if (!fona.HTTP_GET_start(F(urlString), &statuscode, (uint16_t *)&length)) {
    if (statuscode > 299 && statuscode < 200) {
      Serial.println("failed to create bucket!");
    }
  }
  while (length > 0) {
    while (fona.available()) {
      char c = fona.read();
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
      loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
      UDR0 = c;
#else
      Serial.write(c);
#endif
      length--;
      if (! length) break;
    }
  }
  fona.HTTP_GET_end();
  urlString = "";
  //fona.HTTP_para(F("URL"), F("http://insecure-groker.initialstate.com/"));
//  while (!fona.HTTP_term()) {
//    Serial.println(F("Still waiting for HTTPTERM"));
//    delay(1000);
//  }
}

void sendDataArrays(String keys[], String values[], int kvpCount) {
  
  keyValuesToSend = "";
  for (int i = 0; i < kvpCount; i++) {
    if (keys[i] != "") {
      keyValuesToSend += "&";
      keyValuesToSend += keys[i];
      keyValuesToSend += "=";
      keyValuesToSend += values[i];
    }
  }
  Serial.print("KVPs to Send: ");
  Serial.println(keyValuesToSend);

  uint16_t statuscode;
  int16_t length;
  urlString = "http://insecure-groker.initialstate.com/api/events?accessKey=";
  urlString += String(ISAccessKey);
  urlString +="&bucketKey=";
  urlString += bucketKey;
  urlString += keyValuesToSend;
  Serial.print(F("URL: "));
  Serial.println(urlString);
  
  //urlString.toCharArray(urlArray, urlString.length()+1);
  if (!fona.HTTP_GET_start(F(urlString), &statuscode, (uint16_t *)&length)) {
    if (statuscode > 299 && statuscode < 200) {
      Serial.println("failed to send data!");
    }
  }
  while (length > 0) {
    while (fona.available()) {
      char c = fona.read();
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
      loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
      UDR0 = c;
#else
      Serial.write(c);
#endif
      length--;
      if (! length) break;
    }
  }

  fona.HTTP_GET_end();
  //fona.HTTP_para(F("URL"), F("http://insecure-groker.initialstate.com/"));
//  while (!fona.HTTP_term()) {
//    Serial.println(F("Still waiting for HTTPTERM"));
//    delay(1000);
//  }
}

void setup() {

  while (! Serial);

  Serial.begin(115200);
  Serial.println(F("Initializing FONA... (May take a few seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("FONA is OK"));

  Serial.println(F("Enabling GPS..."));
  fona.enableGPS(true);
  Serial.println(F("Enabling GPRS..."));
  // Try to enable GPRS
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Waiting to enable GPRS..."));
    delay(1000);
  }

  if (fona.getNetworkStatus() == 1) {
    Serial.println(F("Network status good!"));
    if (fona.enableNetworkTimeSync(true)) {
      Serial.println(F("Network Time Sync Enabled!"));
      if (!fona.enableNTPTimeSync(true, F("pool.ntp.org"))) {
        Serial.println(F("Time is probably wrong??"));
      }
      char timeBuffer[23];
      if (fona.getTime(timeBuffer, 23)) {
        Serial.print(F("Found the time! "));
        Serial.println(timeBuffer);
        String timeToPrint = String(timeBuffer);
        timeToPrint.replace(":", "");
        timeToPrint.replace("/","");
        timeToPrint.replace("\"", "");
        timeToPrint.replace(",", "_");
        timeToPrint = timeToPrint.substring(0, 13);
        bucketKey = "FonaTracker" + timeToPrint;
        Serial.println(bucketKey);
        createBucket();
      }
    }
  }

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  delay(2000);
  digitalWrite(13, LOW);
  
  uint16_t vbat;
  if (fona.getBattVoltage(&vbat)) {
    keysToSend[0] = "battery_level_mV";
    valuesToSend[0] = String(vbat);
    //sendData("battery_level_mV", String(vbat));
  }

  if (fona.getBattPercent(&vbat)) {
    keysToSend[1] = "battery_level_perc";
    valuesToSend[1] = String(vbat);
    //sendData("battery_level_perc", String(vbat));
  }
  

  float latitude, longitude, speed_kph, heading, speed_mph, altitude;

  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);

  if (gps_success) {
    
    Serial.print("GPS lat:");
    Serial.println(latitude, 6);
    Serial.print("GPS long:");
    Serial.println(longitude, 6);
    keysToSend[2] = "gps_loc";
    valuesToSend[2] = String(latitude, 6);
    valuesToSend[2] += ",";
    valuesToSend[2] += String(longitude, 6);
    //sendCoordinates("gps_loc", latitude, longitude);
    Serial.print("GPS speed KPH:");
    Serial.println(speed_kph);
    keysToSend[3] = "speed_kph";
    valuesToSend[3] = String(speed_kph);
    //sendData("speed_kph", String(speed_kph));
    Serial.print("GPS speed MPH:");
    speed_mph = speed_kph * 0.621371192;
    Serial.println(speed_mph);
    keysToSend[4] = "speed_mph";
    valuesToSend[4] = String(speed_mph);
    //sendData("speed_mph", String(speed_mph));
    Serial.print("GPS heading:");
    Serial.println(heading);
    keysToSend[5] = "gps_heading";
    valuesToSend[5] = String(heading);
    //sendData("gps_heading", String(heading));
    Serial.print("GPS altitude:");
    Serial.println(altitude);
    keysToSend[6] = "gps_altitude";
    valuesToSend[6] = String(altitude);
    //sendData("gps_altitude", String(altitude));
  } else {
    Serial.println("Waiting for FONA GPS 3D fix...");
  }

  // Check for network, then GPRS 
  Serial.println(F("Checking for Cell network..."));
  if (fona.getNetworkStatus() == 1) {
    // network & GPRS? Great! Print out the GSM location to compare
    boolean gsmloc_success = fona.getGSMLoc(&latitude, &longitude);

    if (gsmloc_success) {
      Serial.print("GSMLoc lat:");
      Serial.println(latitude, 6);
      Serial.print("GSMLoc long:");
      Serial.println(longitude, 6);
      keysToSend[7] = "gsm_loc";
      valuesToSend[7] = String(latitude, 6);
      valuesToSend[7] += ",";
      valuesToSend[7] += String(longitude, 6);
      //sendCoordinates("gsm_loc", latitude, longitude);
    } else {
      Serial.println("GSM location failed...");
      Serial.println(F("Disabling GPRS"));
      fona.enableGPRS(false);
      Serial.println(F("Enabling GPRS"));
      if (!fona.enableGPRS(true)) {
        Serial.println(F("Failed to turn GPRS on"));  
      }
    }
    
    sendDataArrays(keysToSend, valuesToSend, 8);
  }

  digitalWrite(13, HIGH);
}

