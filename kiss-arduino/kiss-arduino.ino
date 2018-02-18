#define ONEWIRETELEM
//#define VOLT_DEBUG
//#define DEBUG_GPS_ALL

#ifdef ONEWIRETELEM
#include <OneWire.h>
#include <DallasTemperature.h>
#endif
#include <SoftwareSerial.h>
#include <avr/wdt.h>

// board size
// don't use pin 13 for the GPS
// caps through board
// passives layout
// flip GPS
// LEDs
// buttons - beacon now

// sw:
// beacon at startup but with no pos
// object rather than station

#define GPSTXPIN 11 //not really, it's 13, but we use the LED, and we don't need to send anything to the GPS in this application
#define GPSRXPIN 12

#define TNCTXPIN 2
#define TNCRXPIN 3

#define BCN_BTN 8
#define LED_TX 9

#define ONE_WIRE_BUS 7
#define BATT_VOLT_PIN 1

#define VOLT_SAMPLES 5
int voltSample;
float volts[VOLT_SAMPLES];

SoftwareSerial kissSerial(TNCRXPIN, TNCTXPIN);
SoftwareSerial gpsSerial(GPSRXPIN, GPSTXPIN);

byte addressField[255];
byte infoField[255];

const int DEBUG = 1;
const int HEXDEBUG = 0;
const byte DELIM_1 = 0x03;
const byte DELIM_2 = 0xF0;
const byte KISS_FEND = 0xc0;
const byte KISS_FESC = 0xdb;
const byte KISS_CMD_DATAFRAME0 = 0x00;

const char SYM_HOUSE[3]              = "/-";
const char SYM_DIGI[3]               = "/#";
const char SYM_PHONE[3]              = "/$";
const char SYM_BOYSCOUTS[3]          = "/,";
const char SYM_REDCROSS[3]           = "/+";
const char SYM_MOTORCYCLE[3]         = "/<";
const char SYM_CAR[3]                = "/>";
const char SYM_CANOE[3]              = "/C";
const char SYM_HOTEL[3]              = "/H";
const char SYM_SCHOOL[3]             = "/K";
const char SYM_BUS[3]                = "/U";
const char SYM_YACHT[3]              = "/Y";
//const char SYM_JOGGER[3]             = "/[";
const char SYM_AMBULANCE[3]          = "/a";
const char SYM_BICYCLE[3]            = "/b";
const char SYM_JEEP[3]               = "/j";
const char SYM_TRUCK[3]              = "/k";
const char SYM_EMERGENCYOPSCENTRE[3] = "/o";
const char SYM_ANTENNA[3]            = "/r";
const char SYM_POWERBOAT[3]          = "/s";
const char SYM_TRUCK18W[3]           = "/u";
const char SYM_VAN[3]                = "/v";
const char SYM_WATER[3]              = "/w";
const char SYM_RESTROOMS[3]          = "\\r";

const char APRS_LOCATION_WITHOUT_TIMESTAMP = '!';

float latDec;
float lonDec;
bool fix = false;
unsigned long lastTx = 0;
unsigned long dontCornerPegUntil = 0;
const int rejectedCornerPegDisableWait = 3000; //ms. After not beaconing a suspect corner (e.g. going too slowly), don't check for corners for this many ms.
String fromCall = "M0LTE-1";

const int turn_threshold = 30; // degrees
const int min_time_between_cornerpegs = 15000; // milliseconds
const int min_transmit_interval = 15000; // min time between beacons
const int speed_threshold_kph = 10;
unsigned long last_corner_time;

unsigned long beacon_rate = 60000; // ms
int heading_at_last_beacon;
int heading_now;
float speedkmh_now;
unsigned long millisSinceLastTx;

char gpsBuf[255];
int gpsCur=0;

#ifdef ONEWIRETELEM
/*  ds18b20
 *  pin 1 - GND
 *  pin 2 - data
 *  pin 3 - +5V
 *  4.7k resistor between pin 2 and 3
 *  code untested - need to change over to mic-e format I think for telemetry - for now in the comments
 *  Requires: https://halckemy.s3.amazonaws.com/uploads/attachments/229743/OneWire.zip
 *            https://github.com/milesburton/Arduino-Temperature-Control-Library
 */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
unsigned long tempSched;
float tempC;
#endif

void setup() {
  kissSerial.begin(38400);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_TX, OUTPUT);
  pinMode(BCN_BTN, INPUT);

  gpsSerial.begin(9600);
  clearGpsBuffer();

  setTxDelay(900);

  for (int i=0; i<255; i++){
    addressField[i]=0;
  }

  for (int i=0; i<255; i++){
    infoField[i]=0;
  }

  wdt_enable(WDTO_1S);

  #ifdef ONEWIRETELEM
  sensors.begin();
  tempSched = millis() + 5000;
  #endif
  
  Serial.println("Started, waiting for fix");
}

void loop() {

  wdt_reset();
  
  unsigned long now = millis();
  millisSinceLastTx = now - lastTx;

  handle_rollover(now);
  
  handle_transmit(now);

  handle_tnc_output();

  handle_gps();

  handle_cornerpegging();

  handle_fixled();

  handle_bcnbtn();

  handle_temps();
  
  handle_voltage();
}

unsigned long voltSched;
float avgVolts;

void handle_voltage(){
  if (millis() > voltSched){
    voltSample = analogRead(BATT_VOLT_PIN);

    for (int i = VOLT_SAMPLES - 1; i > 0; i--) {
      volts[i] = volts[i-1];
    }
    
    volts[0] = voltSample / 22.75;

    avgVolts = 0;
    for (int i = 0; i < VOLT_SAMPLES; i++) {
      avgVolts += volts[i];
    }
    avgVolts = avgVolts / (float)VOLT_SAMPLES;

    #ifdef VOLT_DEBUG
    Serial.print("voltSample: ");
    Serial.print(voltSample);
    Serial.print(" volt:");
    Serial.print(volts[0], 2);
    Serial.print(" avgVolts:");
    Serial.println(avgVolts, 2);
    #endif
    
    voltSched = millis() + 1000;
  }
}

void handle_temps(){
  #ifdef ONEWIRETELEM
  if (millis() > tempSched){
    sensors.requestTemperatures();
    tempC = sensors.getTempCByIndex(0);
    Serial.print("Temp: ");
    Serial.print(tempC, 1);
    Serial.println("C");
    tempSched = millis() + 10000;
  }
  #endif
}

int buttonState;
bool bcnDemanded=false;

unsigned long resumeReadingButtonAt=0;
void handle_bcnbtn() {

  if (millis() < resumeReadingButtonAt)
    return;
    
  buttonState = digitalRead(BCN_BTN);

  if (buttonState == HIGH) {
    resumeReadingButtonAt = millis() + 1000;
    bcnDemanded = true;
    digitalWrite(LED_TX, HIGH);
  }
}

unsigned long sched, now;
bool flashState;

void handle_fixled() {

  now = millis();

  if (now > sched) {
   
    if (flashState) {
      digitalWrite(LED_BUILTIN, LOW);
      flashState = false;
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      flashState = true;
    }

    if (!fix) {
      sched = now + 200;
    } else {
      sched = now + 1000;
    }  
  }
}

void handle_tnc_output(){
  // read anything back from KISS TNC and output it to the hardware serial port (PC)
  while (kissSerial.available()){
    Serial.write(kissSerial.read());
  }
}

void handle_rollover(unsigned long now){
  if (millisSinceLastTx > now){
    // rollover has occurred, basically start over
    millisSinceLastTx = 0;
    dontCornerPegUntil = 0;
  }
}

bool corner=false;

bool shouldTransmit(){

  if (!fix)
    return false;

  if (bcnDemanded)
    return true;

  if (millisSinceLastTx > min_transmit_interval) {
    
    if (corner) {
      return true;
    }
    
    if (lastTx == 0) {
      return true;
    }

    if (millisSinceLastTx > beacon_rate) {
      return true;
    }
  }

  return false;
}

#ifdef ONEWIRETELEM
String comment;
#else
String comment = String("Pelicase");
#endif

void handle_transmit(unsigned long now){
  // send as soon as possible, and every 60 seconds
  if (shouldTransmit()){
    
    bcnDemanded = false;
    
    if (DEBUG) {
      if (corner) {
        Serial.println("corner");
      }
    }

    if (corner) {
      corner = false;
    }
    
    setFromCall(fromCall);
    setSymbol(SYM_CAR);

    #ifdef ONEWIRETELEM
    comment = String(tempC, 1) + "C " + String(avgVolts,2) + "V";
    #endif
    setComment(comment);
    setLat(latDec);
    setLon(lonDec);
    setPath();
    setAprsMessageType();
    
    txflash();
    
    tncWrite(KISS_FEND);
    tncWrite(KISS_CMD_DATAFRAME0);
    tncSendField(addressField);
    tncWrite(DELIM_1);
    tncWrite(DELIM_2);
    tncSendField(infoField);
    tncWrite(KISS_FEND);

    if (DEBUG) {
      Serial.print(F("now="));
      Serial.print(now/1000);
      Serial.print(F("s millisSinceLastTx="));
      Serial.print(millisSinceLastTx/1000);
      Serial.print(F("s heading_at_last_beacon="));
      Serial.print(heading_at_last_beacon);
      Serial.print(F(" heading_now="));
      Serial.print(heading_now);
      Serial.print(F(" "));
      Serial.print(fromCall);
      Serial.print(F(">WIDE1-1 via WIDE2-1: "));
    }
    
    for (int i=0; i<255;i++){
      if (infoField[i] == 0){
        break;
      }
      Serial.print((char)infoField[i]);
    }
    Serial.println();

    // clear the field down
    for (int i=0; i<255;i++){
      infoField[i] = 0;
    }

    heading_at_last_beacon = heading_now;
    lastTx = now;
  }
}

void handle_gps(){
  if (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == 10) {
      // discard LF
      return;
    } else if (c == 13) {
      interpretGps();
      clearGpsBuffer();
      gpsCur=0;
    } else {
      // put the letter into the buffer
      gpsBuf[gpsCur]=c;
      gpsCur++;
    }
  }
}

int heading_diff(int initial, int finalv) {
  int diff = finalv - initial;
  int absDiff = tabs_i(diff);
  if (absDiff <= 180){
    if (absDiff == 180) {
      return absDiff;
    } else {
      return diff;
    }
  } else if (finalv > initial) {
    return absDiff - 360;
  } else {
    return 360 - absDiff;
  }
}

void handle_cornerpegging(){
  // idea for future from web: calculate where the world thinks you are by extrapolating from your last position/speed (dead reckoning), then compare that to your current position. When the error is big enough, transmit. 

  int heading_change_since_last_beacon = tabs_i(heading_diff(heading_at_last_beacon, heading_now));

  unsigned long now = millis();
  unsigned long time_since_last_corner = now - last_corner_time;
  
  if (now > dontCornerPegUntil && heading_change_since_last_beacon > turn_threshold) {
    if (DEBUG) {
      Serial.print(now/1000);
      Serial.print(F("s "));
      Serial.print(F("possible corner - heading_at_last_beacon="));
      Serial.print(heading_at_last_beacon);
      Serial.print(F(" heading_now="));
      Serial.print(heading_now);
      Serial.print(F(" heading_change_since_last_beacon="));
      Serial.print(heading_change_since_last_beacon);
      Serial.print(F(" vs turn_threshold="));
      Serial.println(turn_threshold);
    }
    if (speedkmh_now > speed_threshold_kph) {
      if (time_since_last_corner > min_time_between_cornerpegs) {
        if (DEBUG) {
          Serial.print(F("triggered - millisSinceLastTx now "));
          Serial.print(beacon_rate);
          Serial.print(F(", last_corner_time="));
          Serial.println(last_corner_time);
        }
        //millisSinceLastTx = beacon_rate;
        corner=true;
        last_corner_time = now;
      } else {
        if (DEBUG) {
          Serial.print(F("rejected - too soon since last corner: time_since_last_corner="));
          Serial.print(time_since_last_corner);
          Serial.print(F(", min_time_between_cornerpegs="));
          Serial.println(min_time_between_cornerpegs);
        }
        dontCornerPegUntil = now + rejectedCornerPegDisableWait;
      }
    } else {
      if (DEBUG) {
        Serial.print(F("rejected - too slow: speedkmh_now="));
        Serial.print(speedkmh_now);
        Serial.print(F(", speed_threshold_kph="));
        Serial.println(speed_threshold_kph);
      }
      dontCornerPegUntil = now + rejectedCornerPegDisableWait;
    }
  }
}

void setTxDelay(int ms){
  // doesn't work
  /*kissSerial.write(0x06);
  kissSerial.write(0x01);
  kissSerial.write(0xc8);
  kissSerial.write(0x06);*/
}

void clearGpsBuffer(){
  for (int i=0;i<sizeof(gpsBuf);i++){
     gpsBuf[i]=0;
   }
}

String getCommaSeparatedField(String &data, int index){ 
 
  char separator = ',';
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;
  
  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void setFromCall(String &call) {
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

void setCallsign(byte callsignField[], String callsignAndSsid, bool isLastCall) {
  
  String callsign = "      ";
  String ssidStr = "  ";
  bool inSsid = false;

  int len = callsignAndSsid.length();
  
  // loop through the callsign, does it contain an SSID
  for (int i=0; i< len; i++) {
    char c = callsignAndSsid[i];

    if (!inSsid){
      if (c == '-') {
        inSsid = true;
      }else {
        callsign.setCharAt(i, c);
      }
    } else {
      if (ssidStr.charAt(0) == ' ') {
        ssidStr.setCharAt(0, c);
      } else {
        ssidStr.setCharAt(1, c);
      }
    }
  }
  
  byte ssid;
  if (ssidStr == "  ") {
    ssid = 0;
  }
  else{
    ssid = ssidStr.toInt();
  }

  // at this point, callsign is a six character ASCII string, padded with whitespace if necessary, and ssid is an integer.

  byte callBytes[6];
  // left shift all the callsign bytes by 1
  for (int i=0; i<6;i++){
    callBytes[i] = callsign.charAt(i) << 1;
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
  
  callsignField[0] = callBytes[0];
  callsignField[1] = callBytes[1];
  callsignField[2] = callBytes[2];
  callsignField[3] = callBytes[3];
  callsignField[4] = callBytes[4];
  callsignField[5] = callBytes[5];
  callsignField[6] = ssidByte;
}

void setAprsMessageType(){
  infoField[0] = APRS_LOCATION_WITHOUT_TIMESTAMP;
}

// this is necessary because SoftwareSerial undefs a load of stuff, including abs() which we need.
// seems simplest to provide our own.
// "tabs" == "tom's abs"
float tabs_f(float in) {
  if (in >= 0) {
    return in;
  }else{
    return -in;
  }
}

int tabs_i(int in) {
  if (in >= 0) {
    return in;
  }else{
    return -in;
  }
}

void setLat(float lat) {
  /*
   * C# equivalent:
   *   double absLat = Math.Abs(lat);
       string degStr = string.Format("{0:00}", absLat);
       double fraction = (absLat - (int)absLat) * 60;
       string fracStr = string.Format("{0:00.00}", fraction);
       string ew = lat > 0 ? "N" : "S";
       string result = $"{degStr}{fracStr}{ew}";
   */
  float absLat = tabs_f(lat);
  int absLatDeg = (int)absLat;
  
  char degBytes[2];
  sprintf(degBytes, "%02d", absLatDeg);
  infoField[1]=degBytes[0];
  infoField[2]=degBytes[1];

  float fraction = (absLat - absLatDeg) * 60;

  char str[5];
  dtostrf(fraction, 5, 2, str);
  if (str[0] == 0x20) {
    str[0] = '0';
  }
  if (str[1] == 0x20) {
    str[1] = '0';
  }
  infoField[3]=str[0];
  infoField[4]=str[1];
  infoField[5]=str[2];
  infoField[6]=str[3];
  infoField[7]=str[4];

  if (lat < 0.0) {
    infoField[8]='S';
  } else {
    infoField[8]='N';
  }
}

void setLon(float lon) {
  /*
   * C# version:
   *   double absLon = Math.Abs(lon);
       string degStr = string.Format("{0:000}", absLon);
       double fraction = (absLon - (int)absLon) * 60;
       string fracStr = string.Format("{0:00.00}", fraction);
       string ew = lon < 0 ? "W" : "E";
       string result = $"{degStr}{fracStr}{ew}";
   */
  float absLon = tabs_f(lon);
  int absLonDeg = (int)absLon;
  
  char degBytes[3];
  sprintf(degBytes, "%03d", absLonDeg);
  infoField[10]=degBytes[0];
  infoField[11]=degBytes[1];
  infoField[12]=degBytes[2];

  float fraction = (absLon - absLonDeg) * 60;

  char str[5];
  dtostrf(fraction, 5, 2, str);
  if (str[0] == 0x20) {
    str[0] = '0';
  }
  if (str[1] == 0x20) {
    str[1] = '0';
  }
  infoField[13]=str[0];
  infoField[14]=str[1];
  infoField[15]=str[2];
  infoField[16]=str[3];
  infoField[17]=str[4];

  if (lon < 0.0) {
    infoField[18]='W';
  } else {
    infoField[18]='E';
  }
}

void setSymbol(const char sym[]){
  infoField[9] = sym[0];
  infoField[19] = sym[1];
}

void setComment(String &comment){
  int len = comment.length();

  for (int i=0; i<len; i++) {
    char c = comment.charAt(i);
    infoField[i+20] = c;
  }
}

void setPath() {
  
  byte firstAddressField[7];
  byte thirdAddressField[7];

  for (int i=0; i<7; i++){
    firstAddressField[i]=0;
    thirdAddressField[i]=0;
  }
  
  setCallsign(firstAddressField, "WIDE1-1", false);
  setCallsign(thirdAddressField, "WIDE2-1", true);

  for (int i=0; i<7; i++){
    addressField[i] = firstAddressField[i];
  }

  for (int i=0; i<7; i++){
    addressField[i+14] = thirdAddressField[i];
  }
}

void interpretGps() {
  String str(gpsBuf);
  
  #ifdef DEBUG_GPS_ALL
  Serial.println(str);
  #endif
  
  if (!str.startsWith("$GPRMC,")){
    return;
  }
  
  //String utcTime = getCommaSeparatedField(str, 1);
  String dataStatus = getCommaSeparatedField(str, 2); // 'A' = valid

  if (dataStatus.charAt(0) != 'A'){
    if (fix) {
      Serial.println("Fix lost");
    }
    fix = false;
    return;
  }
  
  String lat = getCommaSeparatedField(str, 3); // e.g. 4121.23302
  String latDeg = lat.substring(0,2); // e.g. 41
  String latMin = lat.substring(2,lat.length()); // e.g. 21.23302
  latDec = latDeg.toFloat() + latMin.toFloat() / 60.0;

  String lon = getCommaSeparatedField(str, 5); // e.g. 00201.31654
  String lonDeg = lon.substring(0,3);
  String lonMin = lon.substring(3,lon.length());
  lonDec = lonDeg.toFloat() + lonMin.toFloat() / 60.0;
  float lonDegF = lonDeg.toFloat();
  float lonMinF = lonMin.toFloat();
  float local = lonDeg.toFloat() + lonMin.toFloat() / 60.0;

  String latNS = getCommaSeparatedField(str, 4); // N/S
  if (latNS.charAt(0) == 'S'){
    latDec = 0-latDec;
  }
  
  String lonEW = getCommaSeparatedField(str, 6); // E/W
  if (lonEW.charAt(0) == 'W'){
    lonDec = 0-lonDec;
  }
  
  String speedKts = getCommaSeparatedField(str, 7); // e.g. 2.703
  String trackDegS = getCommaSeparatedField(str, 8); // e.g. empty
  //String date = getCommaSeparatedField(str, 9); // e.g. 200417

  speedkmh_now = speedKts.toFloat() * 1.852;
  heading_now = trackDegS.toInt();

  if (!fix){
    Serial.print("Fix gained: ");
    Serial.print(latDec,6);
    Serial.print(", ");
    Serial.print(lonDec,6);
    Serial.print(", ");
    Serial.print(speedkmh_now,1);
    Serial.print("km/h");

    if (speedkmh_now > 3) {
      Serial.print(", ");
      Serial.print(heading_now,0);
      Serial.println("deg");
    }
    else {
      Serial.println();
    }
  }

  fix = true;
}

void tncWrite(byte b) {

  kissSerial.write(b);
  
  if (HEXDEBUG) {
    Serial.print(b, HEX);
    Serial.print(" ");
  }
}

void tncSendField(byte field[]) {
  for(int i = 0; i < 255; i++){
    if (field[i] == 0x00){
      break;
    }
    
    kissSerial.write(field[i]);
    
    if (HEXDEBUG) {
      Serial.print(field[i], HEX);
      Serial.print(" ");
    }
  }  
}

void txflash(){
  digitalWrite(LED_TX, HIGH); 
  delay(50);                       
  digitalWrite(LED_TX, LOW);  
  delay(50);
  digitalWrite(LED_TX, HIGH); 
  delay(50);                       
  digitalWrite(LED_TX, LOW);  
}

// sample address field - this is carefully packed, NOT straight ASCII
/*byte addressField_sample[255] = { 
  // from m0lte-13 to wide1-1, wide2-1
  0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x62,  // to WIDE1-1
  0x9A, 0x60, 0x98, 0xA8, 0x8A, 0x40, 0x7A,  // from M0LTE
  0xAE, 0x92, 0x88, 0x8A, 0x64, 0x40, 0x63   // via WIDE2-1 (last)
};*/

// Example information field - this is just ascii
/*byte infoField_sample[255] = {
  // !5126.82N/00101.68W>Arduino testing
    0x21, 0x35, 0x31, 0x32, 0x36, 0x2E, 0x38, 0x32, 0x4E, 0x2F, 0x30, 0x30, 0x31, 0x30, 0x31, 0x2E, 0x36, 0x38, 0x57, 0x3E, 0x41, 0x72, 0x64, 0x75, 0x69, 0x6E, 0x6F, 0x20, 0x74, 0x65, 0x73, 0x74, 0x69, 0x6E, 0x67
  //!     5     1     2     6     .     8     2     N     /     0     0     1     0     1     .     6     8     W     >     A     r     d     u     i     n     o     [spc] t     e     s     t     i     n     g
  //0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23    24    25    26    27    28    29    30    31    32    33    34
};*/

/*
 * Breakdown: 
 * ! = location report without timestamp
 * 51 = degrees - fixed length
 * 26.82 = decimal minutes - fixed length - always two digits + 2 d.p.
 * N = positive latitude
 * / = select primary symbol table - http://www.aprs.org/doc/APRS101.PDF page 104 (Appendix 2)
 * 001 = degrees - fixed length, always three digits
 * 01.68 = decimal minutes - fixed length - always two digits + 2 d.p.
 * W = negative longitude
 * > = select Car from primary symbol table
 * Arduino testing = free text comment
 */


/*
 * Hardware setup
 * 2x Arduino Pro Mini 5V
 * 1x NEO-6M GY-GPS6MV1 GPS APM2.5 (or anything else that outputs TTL level NMEA 0183 sentences)
 * 
 * First Arduino flashed with mobilinkd firmware - this is the TNC
 * Second Arduino flashed with this sketch - this is the GPS host
 * 
 * GPS Arduino pin 3  -> GPS board TX pin
 * GPS Arduino pin 10 -> TNC Arduino TXO
 * GPS Arduino pin 11 -> TNC Arduino RXI
 * 
 * TX audio:
 * TNC arduino pin 6 -> middle of a 100k variable resistor
 * One side of resistor -> ground
 * Other side of resistor -> 100nF / 0.1uF capacitor
 * Other side of that capacitor -> Baofeng/Kenwood/Wouxun headset cable red wire / 3.5mm jack ring
 * Headset cable blue wire / 2.5mm jack sleeve -> circuit ground
 * 
 * PTT:
 * TNC arduino pin 10 -> 1k resistor
 * Other side of resistor -> BC547B base (middle pin)
 * Transistor collector (right hand pin, looking at flat face) ->  circuit ground
 * Transistor emittor (left hand pin) -> Headset cable bare wire / 3.5mm jack sleeve
 * 
 * All GND tied together
 * All VCC tied together
 */
