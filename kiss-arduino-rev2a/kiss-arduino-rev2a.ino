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

void build_info_field(){
  // e.g. !5126.82N/00101.68W>Arduino testing

  for (int i=0;i<INFO_FIELD_LEN;i++) {
    infoField[i] = 0;
  }

  infoField[0] = '!';
  
  //Serial.println(latitude_uDeg); // 51447326 -> 51d 26.84m

  long abslatitude_uDeg;
  if (_latitude_uDeg < 0){
    abslatitude_uDeg = _latitude_uDeg * -1;
  } else {
    abslatitude_uDeg = _latitude_uDeg;
  }

  int latWholeDeg = abslatitude_uDeg / 1000000; // 51
  Serial.println(latWholeDeg);
  
  long lat_whole = latWholeDeg * 1000000;
  long lat_frac = abslatitude_uDeg - lat_whole;
  Serial.println(lat_frac); // 447326

  long megaMins = lat_frac * 60;
  Serial.println(megaMins); // 26838480

  long minsPart = megaMins / 10000;
  Serial.println(minsPart); // 2683

  long wholeMins = minsPart/100;

  long decMinsPart = minsPart - wholeMins*100;
  Serial.println(decMinsPart); // 83
  
  char buf[2];
  itoa(latWholeDeg, buf, 10);
  infoField[1] = buf[0];
  infoField[2] = buf[1];
  itoa(wholeMins, buf, 10);
  infoField[3] = buf[0];
  infoField[4] = buf[1];
  infoField[5] = '.';
  itoa(decMinsPart, buf, 10);
  infoField[6] = buf[0];
  infoField[7] = buf[1];
  infoField[8] = latitude_uDeg > 0 ? 'N' : 'S';

  for (int i=0;i<INFO_FIELD_LEN;i++){
    if (infoField[i] == 0) {
      break;
    }
    Serial.write(infoField[i]);
  }

  // bookmark.
  Lat seems to work. Need to work on longitude encoding now.
}

void transmit() {
  lastTX = millis();
  Serial.println("tx");

  build_info_field();
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
