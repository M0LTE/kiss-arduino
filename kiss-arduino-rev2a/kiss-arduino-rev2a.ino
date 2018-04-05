#include <SoftwareSerial.h>
#include <MicroNMEA.h>

// hardware
#define GPS_TX_PIN 11
#define GPS_RX_PIN 12
#define TNC_TX_PIN 2
#define TNC_RX_PIN 3
#define BCN_BTN_PIN 8
#define LED_TX_PIN 9
#define LED_SPARE_PIN 10
#define LED_FIX_PIN 13
#define ONE_WIRE_BUS_PIN 7
#define BATT_VOLT_SENSE_PIN 1

// options
#define AUTO_TX_MIN_INTERVAL_MS 15000
#define BEACON_INTERVAL_MS 60000
#define HEXDEBUG 0

// compatibility
#define _itoa itoa // this helps code compile in Visual C++ as well as Arduino land

// constants
#define KISS_FEND 0xC0
#define KISS_CMD_DATAFRAME0 0x00
#define DELIM_1 0x03
#define DELIM_2 0xF0

const char waitingMsg[] = "Waiting for fix";

char locationComment[25] = "3T Bus";

// global variables
bool bcnDemanded = false;
unsigned long lastTX = 0;
SoftwareSerial kissSerial(TNC_RX_PIN, TNC_TX_PIN);
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
char nmeaBuffer[85];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void setup() {
  kissSerial.begin(38400);
  gpsSerial.begin(9600);
  Serial.begin(9600);
  pinMode(LED_FIX_PIN, OUTPUT);
  pinMode(LED_TX_PIN, OUTPUT);
  pinMode(BCN_BTN_PIN, INPUT);
  pinMode(BATT_VOLT_SENSE_PIN, INPUT);
}

bool fixValid = false;
long latitude_uDeg = -1;
long longitude_uDeg = -1;
int speed_knots = -1;
int course_degs = -1;
long altitude_m = -1;
unsigned long lastChanged = 0;

long lastLatTransmitted_uDeg = -1;
long lastLonTransmitted_uDeg = -1;
int lastSpeedTransmitted_Knots = -1;
int lastCourseTransmitted_Degs = -1;

void handle_gps() {
  // non-blocking
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (nmea.process(c)) {
      if (nmea.isValid()) {
        long newLat = nmea.getLatitude();
        long newLon = nmea.getLongitude();
        long newSpeed = nmea.getSpeed();
        long newCourse = nmea.getCourse();
        long alt = -1;
        bool altValid = nmea.getAltitude(alt);

        if (newLat != latitude_uDeg || newLon != longitude_uDeg || newSpeed / 1000 != speed_knots || newCourse / 1000 != course_degs || (altValid ? altitude_m != alt / 1000 : false)) {
          lastChanged = millis();
          latitude_uDeg = newLat;
          longitude_uDeg = newLon;
          speed_knots = newSpeed / 1000;
          course_degs = newCourse / 1000;
          altitude_m = alt / 1000;

          fixValid = true;

          /*Serial.print(latitude_microDeg / 1000000., 6);
          Serial.print(", ");
          Serial.print(longitude_microDeg / 1000000., 6);
          Serial.print(" @ ");
          Serial.print(altitiude_mm / 1000., 1);
          Serial.println("m ASL");*/
        }
      } else {
        latitude_uDeg = longitude_uDeg = speed_knots = course_degs = altitude_m = -1;
        fixValid = false;
      }
    }
  }
}

bool overMinInterval(){
  if (millis() - lastTX > AUTO_TX_MIN_INTERVAL_MS) {
    return true;
  } else {
    return false;
  } 
}

bool should_transmit() {

  if (bcnDemanded) {
    bcnDemanded = false;
    return true;
  }

  if (!overMinInterval()) {
    return false;
  }

  // TODO: dead reckoning
  if (millis() - lastTX > BEACON_INTERVAL_MS){
    return true;
  }
  
  return false;
}

#define INFO_FIELD_LEN 255
byte infoField[INFO_FIELD_LEN];

char buf[4] = {0,0,0,0};

void processLongitude(){
  // clear buffer
  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = 0;
  }

  // 00101.68W
  long absLongUdeg;
  if (longitude_uDeg < 0) {
    absLongUdeg = longitude_uDeg * -1;
  }
  else {
    absLongUdeg = longitude_uDeg;
  }

  // 1027283
  long lonWholeuDeg = (absLongUdeg / 1000000) * 1000000; // 1000000
  long lonWholeDeg = lonWholeuDeg / 1000000;
  _itoa(lonWholeDeg, buf, 10);
      
  // pad
  if (lonWholeDeg < 10) {
    infoField[10] = '0';
    infoField[11] = '0';
    infoField[12] = buf[0];
  }
  else if (lonWholeDeg < 100) {
    infoField[10] = '0';
    infoField[11] = buf[0];
    infoField[12] = buf[1];
  }
  else {
    infoField[10] = buf[0];
    infoField[11] = buf[1];
    infoField[12] = buf[2];
  }

  long lonFracUDeg = absLongUdeg - lonWholeuDeg; // 27283
  long lonuFrac = lonFracUDeg * 60; // = 1636980
  long preDP = lonuFrac / 1000000; // 1
  long postDP = lonuFrac - preDP * 1000000;
  float postDPRounded_f = postDP / 10000.;
  long postDPRounded = lround(postDPRounded_f); // 64

  // clear buffer
  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = 0;
  }

  // 001 01.68W
  _itoa(preDP, buf, 10);
  if (preDP < 10) {
    infoField[13] = '0';
    infoField[14] = buf[0];
  }
  else {
    infoField[13] = buf[0];
    infoField[14] = buf[1];
  }

  infoField[15] = '.';
  
  _itoa(postDPRounded, buf, 10);
  if (postDPRounded < 10) {
    infoField[16] = '0';
    infoField[17] = buf[0];
  }
  else {
    infoField[16] = buf[0];
    infoField[17] = buf[1];
  }

  if (longitude_uDeg < 0) {
    infoField[18] = 'W';
  }
  else {
    infoField[18] = 'E';
  }

  infoField[19] = '>';
}

void processComment(char msg[], int msgLen) {
  for (int i = 0; i < msgLen; i++) {
    infoField[20 + i] = msg[i];
  }
}

void processLatitude() {
  long abslatitude_uDeg;
  if (latitude_uDeg < 0) {
    abslatitude_uDeg = latitude_uDeg * -1;
  }
  else {
    abslatitude_uDeg = latitude_uDeg;
  }
  
  int latWholeDeg = abslatitude_uDeg / 1000000; // 51
  long lat_whole = latWholeDeg * 1000000;
  long lat_frac = abslatitude_uDeg - lat_whole;
  long latMegaMins = lat_frac * 60;
  float latMinsPart_f = latMegaMins / 10000.;
  long latMinsPart = lround(latMinsPart_f);
  long latWholeMins = latMinsPart / 100;
  long latDecMinsPart = latMinsPart - latWholeMins * 100;

  #if FALSE
  Serial.print("abslatitude_uDeg: "); Serial.println(abslatitude_uDeg);
  Serial.print("latWholeDeg: "); Serial.println(latWholeDeg);
  Serial.print("lat_whole: "); Serial.println(lat_whole);
  Serial.print("lat_frac: "); Serial.println(lat_frac);
  Serial.print("latMegaMins: "); Serial.println(latMegaMins);
  Serial.print("latMinsPart_f: "); Serial.println(latMinsPart_f);
  Serial.print("latMinsPart: "); Serial.println(latMinsPart);
  Serial.print("latWholeMins: "); Serial.println(latWholeMins);
  Serial.print("latDecMinsPart: "); Serial.println(latDecMinsPart);
  #endif
  
  _itoa(latWholeDeg, buf, 10);
  if (latWholeDeg < 10) {
    infoField[1] = '0';
    infoField[2] = buf[0];
  }
  else {
    infoField[1] = buf[0];
    infoField[2] = buf[1];
  }

  _itoa(latWholeMins, buf, 10);
  infoField[3] = buf[0];
  infoField[4] = buf[1];
  infoField[5] = '.';
  _itoa(latDecMinsPart, buf, 10);
  infoField[6] = buf[0];
  infoField[7] = buf[1];
  infoField[8] = latitude_uDeg > 0 ? 'N' : 'S';
}

void build_info_field(char msg[], int msgLen) {
  // e.g. !5126.82N/00101.68W>Arduino testing

  for (int i = 0; i<INFO_FIELD_LEN; i++) {
    infoField[i] = 0;
  }

  if (fixValid){
    infoField[0] = '!';
   
    processLatitude();

    // separator
    infoField[9] = '/';
  
    processLongitude();

    processComment(msg, msgLen);
    
  } else {
    infoField[0] = '>'; // status message
    for (int i=0;i<sizeof(waitingMsg);i++){
      infoField[1+i] = waitingMsg[i];
      if (waitingMsg[i] == 0){
        break;
      }
    }
  }
}

#define ADDRESS_FIELD_LEN 21
byte addressField[ADDRESS_FIELD_LEN] = { 
  // from m0lte-13 to wide1-1, wide2-1
  0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x62,  // to WIDE1-1
  0x9A, 0x60, 0x98, 0xA8, 0x8A, 0x40, 0x7A,  // from M0LTE-13
  0xAE, 0x92, 0x88, 0x8A, 0x64, 0x40, 0x63   // via WIDE2-1 (last)
};

void tncWrite(byte b) {

  kissSerial.write(b);
  
  #if HEXDEBUG
    Serial.print(b, HEX);
    Serial.print(" ");
  #endif
}

void tncSendField(byte field[], int maxlen) {
  for(int i = 0; i < maxlen; i++){

    if (field[i] == 0){
      break;
    }
    
    kissSerial.write(field[i]);
    
    #if HEXDEBUG
      Serial.print(field[i], HEX);
      Serial.print(" ");
    #endif
  }  
}

void transmit() {
  lastTX = millis();
  Serial.println("tx");

  build_info_field(locationComment, sizeof(locationComment));
  
  for (int i = 0; i<INFO_FIELD_LEN; i++) {
    if (infoField[i] == 0) {
      break;
    }
    Serial.write(infoField[i]);
  }
  Serial.println();

  tncWrite(KISS_FEND);
  tncWrite(KISS_CMD_DATAFRAME0);
  tncSendField(addressField, ADDRESS_FIELD_LEN);
  tncWrite(DELIM_1);
  tncWrite(DELIM_2);
  tncSendField(infoField, INFO_FIELD_LEN);
  tncWrite(KISS_FEND);
}

unsigned long resumeReadingButtonAt = 0;

void handle_bcnbtn() {
  if (millis() < resumeReadingButtonAt)
    return;
    
  int buttonState = digitalRead(BCN_BTN_PIN);

  if (buttonState == HIGH) {
    resumeReadingButtonAt = millis() + 1000;
    bcnDemanded = true;
  }
}

void loop() {
  handle_gps();

  handle_bcnbtn();

  if (should_transmit()) {
    transmit();
  }
}
