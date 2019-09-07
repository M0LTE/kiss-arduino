#include <SoftwareSerial.h>

#define TNC_TX_PIN 2
#define TNC_RX_PIN 3

SoftwareSerial kissSerial(TNC_RX_PIN, TNC_TX_PIN);

void setup() {
  Serial.begin(38400);
  kissSerial.begin(38400);
}

void loop() {
  if (Serial.available()) {      
    kissSerial.write(Serial.read()); 
  }

  if (kissSerial.available()) {     
    Serial.write(kissSerial.read()); 
  }
}
