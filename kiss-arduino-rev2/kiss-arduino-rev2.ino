#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>

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
#define TEMP_SENSE_EVERY_MS 60000
#define VOLT_SAMPLES 5
#define TRANSMIT_INTERVAL 60000

int voltSample;
float volts[VOLT_SAMPLES];
SoftwareSerial kissSerial(TNC_RX_PIN, TNC_TX_PIN);
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
float latNow;
float lonNow;
bool fix;
bool bcnDemanded;
bool corner;
float latLastTx;
float lonLastTx;
unsigned long lastTx;
unsigned long loopStart;
unsigned long millisSinceLastTx;
unsigned long tempSched;
byte packetBuf[256];
char gpsBuf[255];
int gpsCur=0;
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature tempSensor(&oneWire);

const char STR_TX_FIRST_TIME[] = "TX because we haven't yet";
const char STR_TX_BCN_BTN[] =    "TX because of button push";
const char STR_TX_CORNER[] =     "TX because of a corner";
const char STR_TX_INTERVAL[] =   "TX because of interval";

void setup() {
  kissSerial.begin(38400);
  gpsSerial.begin(9600);
  Serial.begin(9600);
  pinMode(LED_FIX_PIN, OUTPUT);
  pinMode(LED_TX_PIN, OUTPUT);
  pinMode(BCN_BTN_PIN, INPUT);
  pinMode(BATT_VOLT_SENSE_PIN, INPUT);
  tempSensor.begin();
  tempSched = millis() + TEMP_SENSE_EVERY_MS;
  packetBuf[0]=0xc0;
  Serial.println(F("Started, waiting for fix"));
}


void trace(char msg[]){
  Serial.println(msg);
}

bool should_transmit(){

  if (!fix) {
    return false;
  }
  
  if (lastTx == 0) {
    // has never TX'd
    trace(STR_TX_FIRST_TIME);
    return true;
  }
    
  if (bcnDemanded) {
    bcnDemanded = false;
    trace(STR_TX_BCN_BTN);
    return true;
  }

  if (corner) {
    corner = false;
    trace(STR_TX_CORNER);
    return true;
  }
    
  if (millisSinceLastTx > TRANSMIT_INTERVAL) {
    trace(STR_TX_INTERVAL);
    return true;
  }

  return false;
}

void construct_packet(){
}

void send_packet(){
}

void handle_transmit(){
  if (should_transmit()){
    construct_packet();
    send_packet();
  }
}

void handle_tnc_output(){
  
}

void clearGpsBuffer(){
  for (int i=0;i<sizeof(gpsBuf);i++){
     gpsBuf[i]=0;
   }
}

//225446.12
int timeOfFixHours;
int timeOfFixMins;
int timeOfFixSecs;
int timeOfFixSecsFrac;

//A
char navRcvWarn;

//4916.45,N
int latDeg, latMin, latMinFrac;
char ns;

//12311.12,W
int lonDeg, lonMin, lonMinFrac;
char ew;

//000.5
int sogKnots, sogKnotsFrac;

//054.7
int courseTrueDeg, courseTrueDegFrac;

//191194
int utcDateOfFixD, utcDateOfFixM, utcDateOfFixY;

//020.3,E
int magVarDeg, magVarDegFrac;
char magVarEW;

//*68
char checksum[3];

int startOfNextField=7;
void interpretGps() {
  // $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
  // 01234567
  
  if (strncmp(gpsBuf, "$GPRMC", 6) != 0) {
    // if the sentence is not Recommended minimum specific GPS/Transit data, ignore.
    return;
  }

  startOfNextField = interpret_timeOfFix(startOfNextField);
  
}

char tofBuf[7];
int interpret_timeOfFix(int startOfField) {

  // warning, all untested
  tofBuf[0] = gpsBuf[startOfField];
  tofBuf[1] = gpsBuf[startOfField+1];
  tofBuf[2] = 0;
  timeOfFixHours = atoi(tofBuf);

  tofBuf[0] = gpsBuf[startOfField+2];
  tofBuf[1] = gpsBuf[startOfField+3];
  tofBuf[2] = 0;
  timeOfFixMins = atoi(tofBuf);

  tofBuf[0] = gpsBuf[startOfField+4];
  tofBuf[1] = gpsBuf[startOfField+5];
  tofBuf[2] = 0;
  timeOfFixSecs = atoi(tofBuf);

  for (int i=0; i<sizeof(tofBuf);i++){
    tofBuf[i] = 0;
  }
  
  if (gpsBuf[startOfField + 6] == '.') {
    int cur=0;
    char c = gpsBuf[startOfField + 6 + cur];
    while (c != ','){
      tofBuf[cur] = c;
      cur++;
    }
    timeOfFixSecsFrac = atoi(tofBuf);
    return startOfField + 6 + cur; // hhmmss.ffff
  } else {
    timeOfFixSecsFrac = 0;
    return startOfField + 6; // hhmmss
  }
}

void handle_gps_output(){
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

void handle_fixled(){
}

void handle_bcnbtn(){
}

void handle_temps(){
}

void handle_voltage(){
}

void loop() {
  loopStart = millis();
  millisSinceLastTx = loopStart - lastTx;

  handle_transmit();

  handle_tnc_output();

  handle_gps_output();

  handle_fixled();

  handle_bcnbtn();

  handle_temps();

  handle_voltage();
}
