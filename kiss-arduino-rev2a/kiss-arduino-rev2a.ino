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
#define AUTO_TX_MIN_INTERVAL_MS 15000L
#define BEACON_INTERVAL_MS 300000L
#define HEXDEBUG 0

// 1 deg lat is 111000m
// 1m is 1/111000 degrees
// 1m is 1000000/111000 microdegrees
// 1m is 9 microdegrees
// so 3000 microdegrees is approx 333m
// not that different for longitude
// this is very much ballpark but it should work ok
#define DISTANCE_THRESHOLD_uDEG 3000

// compatibility
#define _itoa itoa // this helps code compile in Visual C++ as well as Arduino land

// constants
#define KISS_FEND 0xC0
#define KISS_CMD_DATAFRAME0 0x00
#define DELIM_1 0x03
#define DELIM_2 0xF0
const char FROM_CALL[] = "M0LTE-3";
const char locationComment[25] = "3T Bus 3";

// See http://www.aprs.org/doc/APRS101.PDF page 104 (Appendix 2: The APRS Symbol Tables)
// U is bus, > is car
const char APRS_PRIMARY_SYMBOL = 'U'; 

#define ADDRESS_FIELD_LEN 21
byte addressField[ADDRESS_FIELD_LEN] = { 
  // from m0lte-13 to wide1-1, wide2-1
  0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x62,  // to WIDE1-1
  //0x9A, 0x60, 0x98, 0xA8, 0x8A, 0x40, 0x7A,  // from M0LTE-13
  0,0,0,0,0,0,0, 
  0xAE, 0x92, 0x88, 0x8A, 0x64, 0x40, 0x63   // via WIDE2-1 (last)
};

// global variables
bool bcnDemanded = false;
unsigned long lastLocationTX = 0;
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
  setFromCall(FROM_CALL);
}

bool fixValid = false;
long latitude_uDeg = -1;
long longitude_uDeg = -1;
int speed_knots = -1;
int course_degs = -1;
long altitude_m = -1;
unsigned long gpsPosLastChangedAt = 0;

long lastLatTransmitted_uDeg = -1;
long lastLonTransmitted_uDeg = -1;

bool shouldTransmitGotFixMessage = false;
bool shouldTransmitLostFixMessage = false;

void setFromCall(char call[]) {
  byte from[7];
  for (int i=0; i<7; i++) {
    from[i]=0;
  }
  
  setCallsign(from, call, false);

  addressField[7] = from[0];
  addressField[8] = from[1];
  addressField[9] = from[2];
  addressField[10] = from[3];
  addressField[11] = from[4];
  addressField[12] = from[5];
  addressField[13] = from[6];
}

void setCallsign(char target[], char callsignAndSsid[], bool isLastCall) {
  
  char callsign[6];
  callsign[0] = callsign[1] = callsign[2] = callsign[3] = callsign[4] = callsign[5] = ' ';
  
  char ssidArr[3];
  ssidArr[0] = ssidArr[1] = ssidArr[2] = 0;
  
  bool inSsid = false;
  
  // loop through the callsign, does it contain an SSID
  for (int i=0; i < 10; i++) {
    char c = callsignAndSsid[i];

    if (c == 0){
      break;
    }

    if (!inSsid){
      if (c == '-') {
        inSsid = true;
      }else {
        callsign[i] = c;
      }
    } else {
      if (ssidArr[0] == 0) {
        ssidArr[0] = c;
      } else {
        ssidArr[1] = c;
      }
    }
  }
  
  byte ssid;
  if (ssidArr[0] == 0 && ssidArr[1] == 0) {
    ssid = 0;
  }
  else{
    ssid = atoi(ssidArr);
    // convert ssidArr to integer
    //ssid = ssidStr.toInt();
  }

  // at this point, callsign is a six character ASCII string, padded with whitespace if necessary, and ssid is an integer.

  byte callBytes[6];
  // left shift all the callsign bytes by 1
  for (int i=0; i<6;i++){
    callBytes[i] = callsign[i] << 1;
  }
  
  byte ssidByte;
  bitWrite(ssidByte,7,0);
  bitWrite(ssidByte,6,1);
  bitWrite(ssidByte,5,1);
  bitWrite(ssidByte,4,bitRead(ssid,3));
  bitWrite(ssidByte,3,bitRead(ssid,2));
  bitWrite(ssidByte,2,bitRead(ssid,1));
  bitWrite(ssidByte,1,bitRead(ssid,0));
  bitWrite(ssidByte,0,isLastCall);
  
  target[0] = callBytes[0];
  target[1] = callBytes[1];
  target[2] = callBytes[2];
  target[3] = callBytes[3];
  target[4] = callBytes[4];
  target[5] = callBytes[5];
  target[6] = ssidByte;
}

void handle_gps() {
  // non-blocking
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (nmea.process(c)) {
      if (nmea.isValid()) {
        if (!fixValid){
          shouldTransmitGotFixMessage = true;
        }
        long newLat = nmea.getLatitude();
        long newLon = nmea.getLongitude();
        long newSpeed = nmea.getSpeed();
        long newCourse = nmea.getCourse();
        long alt = -1;
        bool altValid = nmea.getAltitude(alt);

        if (newLat != latitude_uDeg || newLon != longitude_uDeg || newSpeed / 1000 != speed_knots || newCourse / 1000 != course_degs || (altValid ? altitude_m != alt / 1000 : false)) {
          gpsPosLastChangedAt = millis();
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
        if (fixValid) {
          shouldTransmitLostFixMessage = true;
        }
        fixValid = false;
      }
    }
  }
}

double get_dist_from_last_tx() {
  long xStart = lastLatTransmitted_uDeg;
  long xEnd = latitude_uDeg;
  long yStart = lastLonTransmitted_uDeg;
  long yEnd = longitude_uDeg;

  long dx = xEnd - xStart;
  long dy = yEnd - yStart;
  
  double dist = sqrt(pow(dx,2) + pow(dy,2));

  return dist;
}

bool far_from_last_tx(){
  
  if (!fixValid){
    return false;
  }

  double dist = get_dist_from_last_tx();

  bool overThreshold = dist > DISTANCE_THRESHOLD_uDEG;

  return overThreshold;
}

unsigned long nextScheduledBeacon = millis() + BEACON_INTERVAL_MS;

bool isTimeToBeacon(){
  if (millis() > nextScheduledBeacon){
    return true;
  }

  return false;
}

bool should_transmit_location() {

  if (bcnDemanded) {
    bcnDemanded = false;
    Serial.println("Transmitting because beacon button was pressed");
    return true;
  }

  // transmit at least this often
  if (isTimeToBeacon()){
    Serial.println("Transmitting because it's time to beacon");
    return true;
  }

  // transmit if we're > x distance from last TX position sent
  if (far_from_last_tx()) {
    Serial.println("Transmitting because we're far from where we last transmitted");
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

  infoField[19] = APRS_PRIMARY_SYMBOL;
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

  infoField[0] = '!';
 
  processLatitude();

  // separator
  infoField[9] = '/';

  processLongitude();

  processComment(msg, msgLen);
}

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

void transmitStatus(char msg[], int maxlen) {
  infoField[0] = '>'; // status message
  for (int i=0;i<maxlen;i++){
    infoField[1+i] = msg[i];
    Serial.write(msg[i]);
    if (msg[i] == 0){
      break;
    }
  }

  Serial.println();

  sendFrame();
}

void transmitLocation() {

  if (!fixValid){
    Serial.println("Not transmitting location - no fix");
    return;
  }

  nextScheduledBeacon = millis() + BEACON_INTERVAL_MS;
  
  lastLatTransmitted_uDeg = latitude_uDeg;
  lastLonTransmitted_uDeg = longitude_uDeg;
  build_info_field(locationComment, sizeof(locationComment));
  
  sendFrame();
}

void sendFrame(){ 
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

void handle_got_and_lost_fix_messages(){
  if (shouldTransmitGotFixMessage) {
    delay(2500);
    transmitStatus("Got fix", 10);
    shouldTransmitGotFixMessage = false;
  } else if (shouldTransmitLostFixMessage) {
    delay(2500);
    transmitStatus("Lost fix", 10);
    shouldTransmitLostFixMessage = false;
  }
}

unsigned long get_msecs_until_beacon() {
  return nextScheduledBeacon - millis();
}

unsigned long nextStatusPrint = 0;
void handle_secondly_status(){
  if (millis() > nextStatusPrint){

    Serial.print("msec until beacon: ");
    Serial.println(get_msecs_until_beacon());
    
    Serial.print("dist from last TX: ");
    Serial.println(get_dist_from_last_tx());
    
    nextStatusPrint = millis() + 2000;
  }
}

void loop() {
  handle_gps();

  handle_bcnbtn();

  if (should_transmit_location()) {
    transmitLocation();
  }

  handle_got_and_lost_fix_messages();

  handle_secondly_status();
}

