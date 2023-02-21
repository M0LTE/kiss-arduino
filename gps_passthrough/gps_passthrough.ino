#include <SoftwareSerial.h>

#define GPS_TX_PIN 11
#define GPS_RX_PIN 12

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

void setup() {
  gpsSerial.begin(9600);
  Serial.begin(9600);
}

int incomingByte;

void loop() {
  if (gpsSerial.available() > 0) {
    incomingByte = gpsSerial.read();
    Serial.write(incomingByte);
  }
}
