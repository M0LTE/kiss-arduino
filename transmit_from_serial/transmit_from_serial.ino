#include <SoftwareSerial.h>

#define TNC_TX_PIN 2
#define TNC_RX_PIN 3
#define MAX_VIA_CALLS 10

// constants
#define KISS_FEND 0xC0
#define KISS_CMD_DATAFRAME0 0x00
#define DELIM_1 0x03
#define DELIM_2 0xF0
#define INFO_FIELD_LEN 300

// global variables
SoftwareSerial kissSerial(TNC_RX_PIN, TNC_TX_PIN);
byte addressField[14 + 7 * MAX_VIA_CALLS];
byte infoField[INFO_FIELD_LEN];
int addressFieldLength = 14;

// This program will just send a UI frame containing the text of whatever line of text it receives from the serial port.

void setup() {
  kissSerial.begin(38400);
  Serial.begin(9600);
  
  setFromCall("M0LTE", true);
  //setToCall("zzzzzz-5");
  /*setViaCall("aaaaaa-1", 0, false);
  setViaCall("aaaaab-1", 1, false);
  setViaCall("aaaaac-1", 2, false);
  setViaCall("aaaaad-1", 3, false);
  setViaCall("aaaaae-1", 4, false);
  setViaCall("aaaaaf-1", 5, false);
  setViaCall("aaaaag-1", 6, false);
  setViaCall("aaaaah-1", 7, false);
  setViaCall("aaaaai-1", 8, false);
  setViaCall("aaaaaj-1", 9, true);*/
}

void setFromCall(char call[], bool direct) {
  setCall(call, 7, direct);
}

void setToCall(char call[]) {
  setCall(call, 0, false);
}

void setViaCall(char call[], int callNum, bool isLast) {
  if (callNum < MAX_VIA_CALLS){
    setCall(call, 14 + callNum*7, isLast);
    if (isLast)
    {
      addressFieldLength = (callNum + 1) * 7 + 14;
    }
  }
}

void setCall(char call[], int offset, bool isLast) {
  byte buf[7];
  for (int i=0; i<7; i++) {
    buf[i]=0;
  }
  
  setCallsign(buf, call, isLast);

  for (int i = 0; i < 7; i++){
    addressField[offset+i] = buf[i];
  }
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

void put_payload_in_infoField(char msg[], int msgLen) {
  for (int i = 0; i<INFO_FIELD_LEN; i++) {
    infoField[i] = 0;
  }
  
  for (int i = 0; i < msgLen; i++) {
    infoField[i] = msg[i];
  }
}

void tncWrite(byte b) {
  kissSerial.write(b);
}

void tncSendField(byte field[], int maxlen) {
  for(int i = 0; i < maxlen; i++){

    if (field[i] == 0){
      break;
    }
    
    kissSerial.write(field[i]);
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

void sendFrame(){ 
  tncWrite(KISS_FEND);
  tncWrite(KISS_CMD_DATAFRAME0);
  tncSendField(addressField, addressFieldLength);
  tncWrite(DELIM_1);
  tncWrite(DELIM_2);
  tncSendField(infoField, INFO_FIELD_LEN);
  tncWrite(KISS_FEND);
}

char serialBuffer[500];
int cur = 0;

void loop() { 
  if (Serial.available() > 0){
    char c = (char)Serial.read();    
    if (c == '\r'){
      send("CQ", serialBuffer, cur); // NB this doesn't block
      cur = 0;
      for (int i=0; i<sizeof(serialBuffer); i++){
        serialBuffer[i]=0;
      }
    } else {
      serialBuffer[cur] = c;
      cur++;
    }
  }
}

void send(char to[], char serialBuffer[], int len){

  setToCall(to);
  
  put_payload_in_infoField(serialBuffer, len);
  
  sendFrame(); // NB this doesn't block
}
