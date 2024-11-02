//Mac-Adresse: 90:C9:22:EC:B9:80
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>


#define SERVOMIN 125
#define SERVOMAX 600


//Pin of Servos
#define SVL 0
#define SHR 1
#define SVR 2
#define SHL 3
#define TVL 4
#define THR 5
#define TVR 7
#define THL 8
#define BVL 9
#define BHR 10
#define BVR 11
#define BHL 12

//Hex-Adresse des Empfängers
#define FB0 0xFF6897
#define FB1 0xFF30CF
#define FB2 0xFF18E7
#define FB3 0xFF7A85
#define FB4 0xFF10EF
#define FB5 0xFF38C7
#define FB6 0xFF5AA5
#define FB7 0xFF42BD
#define FB8 0xFF4AB5
#define FB9 0xFF52AD
#define FBONOFF 0xFFA25D
#define FBUP 0xFF906F
#define FBDOWN 0xFFE01F
const uint16_t RECV_PIN = 4;

Adafruit_PWMServoDriver servoDriver_module = Adafruit_PWMServoDriver(0x40);
IRrecv irrecv(RECV_PIN);
decode_results results;

int angleToPulse(int ang);
//neutral positions
const int nsvl = 208;
const int nshr = 88;
const int nsvr = 105;
const int nshl = 100;
const int ntvl = 125;
const int nthr = 60;
const int ntvr = 65;
const int nthl = 145;
const int nbvl = 15;
const int nbhr = 160;
const int nbvr = 192;
const int nbhl = 15;

//current positions
int csvl, cshr, csvr, cshl, ctvl, cthr, ctvr, cthl, cbvl, cbhr, cbvr, cbhl;


uint8_t remoteMac[6] = {0x30, 0xC9, 0x22, 0xEC, 0xBE, 0xAC};
String success;
String sitpos = "208,88,105,100,130,60,70,145,15,160,192,15";
String standpos = "208,88,105,100,90,95,100,110,85,90,122,85";
int testnumber = 3;


void home();
void sit();
void stand();
void GoTo(String position);
void sidestepR();
void setServo(int motor, int &currmotor, int angle);
void setServoS(int GoUp);
void setServoT(int GoUp);
void setServoB(int GoUp);
void setServoTB(int GoUp);
void readMacAdress();
void onDataReceive(const uint8_t * mac, const uint8_t * data, int len);
void checkIR();

void setup() {
  Serial.begin(9600);
  servoDriver_module.begin();
  servoDriver_module.setPWMFreq(50);    //Arbeitsfrequenz
  home();
  irrecv.enableIRIn(); 
  Serial.println("IR enabled");
  readMacAdress();
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onDataReceive);
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, remoteMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_err_t addPeerResult = esp_now_add_peer(&peerInfo);
  /*if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    Serial.println(addPeerResult);
    return;
  }else{
    Serial.println("Peer added");
  }*/
}

void loop() {
  checkIR();

  /*delay(5000);
  esp_err_t result = esp_now_send(remoteMac, (uint8_t *) &testnumber, sizeof(testnumber));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }*/
}

int angleToPulse(int ang){
  int pulse = map(ang,0, 270, SERVOMIN,SERVOMAX);  
  Serial.print("Angle: ");Serial.print(ang);
  Serial.print(" pulse: ");Serial.println(pulse);
  return pulse;
}

void home() {
  setServo(SVL, csvl, nsvl);
  setServo(SHR, cshr, nshr);
  setServo(SVR, csvr, nsvr);
  setServo(SHL, cshl, nshl);
  setServo(TVL, ctvl, ntvl);
  setServo(THR, cthr, nthr);
  setServo(TVR, ctvr, ntvr);
  setServo(THL, cthl, nthl);
  setServo(BVL, cbvl, nbvl);
  setServo(BHR, cbhr, nbhr);
  setServo(BVR, cbvr, nbvr);
  setServo(BHL, cbhl, nbhl);
  return;
}

void sit(){
  int diff = abs(ctvl - ntvl);
  for (int i = 0; i<diff; i+=2){
    if (diff - i ==1){
      setServoTB(-1);
    }else{
      setServoTB(-2);
    }
    delay(20);
  }
}

void stand() {
}

void GoTo(String positions) {
  int stepSize = 2;
  bool allServosAtTarget = false;

  int targetPositions[12];
  int index = 0;
  int start = 0;
  for (int i = 0; i < positions.length(); i++) {
    if (positions[i] == ',') {
      targetPositions[index++] = positions.substring(start, i).toInt();
      start = i + 1;
    }
  }
  targetPositions[index] = positions.substring(start).toInt();
  while (!allServosAtTarget) {
    allServosAtTarget = true;

    // Sides
    if (abs(csvl - targetPositions[0]) < stepSize) {
      setServo(SVL, csvl, targetPositions[0]);
    } else if (csvl < targetPositions[0]) {
      setServo(SVL, csvl, csvl + stepSize);
      allServosAtTarget = false;
    } else if (csvl > targetPositions[0]) {
      setServo(SVL, csvl, csvl - stepSize);
      allServosAtTarget = false;
    }

    if (abs(cshr - targetPositions[1]) < stepSize) {
      setServo(SHR, cshr, targetPositions[1]);
    } else if (cshr < targetPositions[1]) {
      setServo(SHR, cshr, cshr + stepSize);
      allServosAtTarget = false;
    } else if (cshr > targetPositions[1]) {
      setServo(SHR, cshr, cshr - stepSize);
      allServosAtTarget = false;
    }

    if (abs(csvr - targetPositions[2]) < stepSize) {
      setServo(SVR, csvr, targetPositions[2]);
    } else if (csvr < targetPositions[2]) {
      setServo(SVR, csvr, csvr + stepSize);
      allServosAtTarget = false;
    } else if (csvr > targetPositions[2]) {
      setServo(SVR, csvr, csvr - stepSize);
      allServosAtTarget = false;
    }

    if (abs(cshl - targetPositions[3]) < stepSize) {
      setServo(SHL, cshl, targetPositions[3]);
    } else if (cshl < targetPositions[3]) {
      setServo(SHL, cshl, cshl + stepSize);
      allServosAtTarget = false;
    } else if (cshl > targetPositions[3]) {
      setServo(SHL, cshl, cshl - stepSize);
      allServosAtTarget = false;
    }

    //Top
    if (abs(ctvl - targetPositions[4]) < stepSize) {
      setServo(TVL, ctvl, targetPositions[4]);
    } else if (ctvl < targetPositions[4]) {
      setServo(TVL, ctvl, ctvl + stepSize);
      allServosAtTarget = false;
    } else if (ctvl > targetPositions[4]) {
      setServo(TVL, ctvl, ctvl - stepSize);
      allServosAtTarget = false;
    }

    if (abs(cthr - targetPositions[5]) < stepSize) {
      setServo(THR, cthr, targetPositions[5]);
    } else if (cthr < targetPositions[5]) {
      setServo(THR, cthr, cthr + stepSize);
      allServosAtTarget = false;
    } else if (cthr > targetPositions[5]) {
      setServo(THR, cthr, cthr - stepSize);
      allServosAtTarget = false;
    }

    if (abs(ctvr - targetPositions[6]) < stepSize) {
      setServo(TVR, ctvr, targetPositions[6]);
    } else if (ctvr < targetPositions[6]) {
      setServo(TVR, ctvr, ctvr + stepSize);
      allServosAtTarget = false;
    } else if (ctvr > targetPositions[6]) {
      setServo(TVR, ctvr, ctvr - stepSize);
      allServosAtTarget = false;
    } 

    if (abs(cthl - targetPositions[7]) < stepSize) {
      setServo(THL, cthl, targetPositions[7]);
    } else if (cthl < targetPositions[7]) {
      setServo(THL, cthl, cthl + stepSize);
      allServosAtTarget = false;
    } else if (cthl > targetPositions[7]) {
      setServo(THL, cthl, cthl - stepSize);
      allServosAtTarget = false;
    } 

    //Bottom
    if (abs(cbvl - targetPositions[8]) < stepSize*2) {
      setServo(BVL, cbvl, targetPositions[8]);
    } else if (cbvl < targetPositions[8]) {
      setServo(BVL, cbvl, cbvl + stepSize*2);
      allServosAtTarget = false;
    } else if (cbvl > targetPositions[8]) {
      setServo(BVL, cbvl, cbvl - stepSize*2);
      allServosAtTarget = false;
    }

    if (abs(cbhr - targetPositions[9]) < stepSize*2) {
      setServo(BHR, cbhr, targetPositions[9]);
    } else if (cbhr < targetPositions[9]) {
      setServo(BHR, cbhr, cbhr + stepSize*2);
      allServosAtTarget = false;
    } else if (cbhr > targetPositions[9]) {
      setServo(BHR, cbhr, cbhr - stepSize*2);
      allServosAtTarget = false;
    } 

    if (abs(cbvr - targetPositions[10]) < stepSize*2) {
      setServo(BVR, cbvr, targetPositions[10]);
    } else if (cbvr < targetPositions[10]) {
      setServo(BVR, cbvr, cbvr + stepSize*2);
      allServosAtTarget = false;
    } else if (cbvr > targetPositions[10]) {
      setServo(BVR, cbvr, cbvr - stepSize*2);
      allServosAtTarget = false;
    } 

    if (abs(cbhl - targetPositions[11]) < stepSize*2) {
      setServo(BHL, cbhl, targetPositions[11]);
    } else if (cbhl < targetPositions[11]) {
      setServo(BHL, cbhl, cbhl + stepSize*2);
      allServosAtTarget = false;
    } else if (cbhl > targetPositions[11]) {
      setServo(BHL, cbhl, cbhl - stepSize*2);
      allServosAtTarget = false;
    } 

    delay(20);
  }
}

void setServo(int motor, int &currmotor, int angle){
  servoDriver_module.setPWM(motor, 0, angleToPulse(angle));
  currmotor = angle;
}

void setServoS(int GoUp){
  setServo(SVL, csvl, (csvl+GoUp));
  setServo(SHR, cshr, (cshr+GoUp));
  setServo(SVR, csvr, (csvr+GoUp));
  setServo(SHL, cshl, (cshl+GoUp));
}

void setServoT(int GoUp){
  //Rechts: hoch addieren, Links: hoch subtrahieren
  setServo(TVL, ctvl, (ctvl-GoUp));
  setServo(THR, cthr, (cthr+GoUp));
  setServo(TVR, ctvr, (ctvr+GoUp));
  setServo(THL, cthl, (cthl-GoUp));
}

void setServoB(int GoUp){
  //Rechts: hoch subtrahieren, Links: hoch addieren
  setServo(BVL, cbvl, cbvl+GoUp);
  setServo(BHR, cbhr, cbhr-GoUp);
  setServo(BVR, cbvr, cbvr-GoUp);
  setServo(BHL, cbhl, cbhl+GoUp);
}

void setServoTB(int GoUp){
  setServoT(GoUp);
  setServoB(2*GoUp);
}

void readMacAdress(){
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void onDataReceive(const uint8_t * mac, const uint8_t * data, int len) {
  int receivedNumber;
  Serial.println("Received data");
  memcpy(&receivedNumber, data, sizeof(receivedNumber));
  Serial.println(receivedNumber);
}

void checkIR(){
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    switch (results.value){
      case FB0:
        Serial.println("FB0");
        Serial.println(csvl);
        Serial.println(cshr);
        Serial.println(csvr);
        Serial.println(cshl);
        Serial.println(ctvl);
        Serial.println(cthr);
        Serial.println(ctvr);
        Serial.println(cthl);
        Serial.println(cbvl);
        Serial.println(cbhr);
        Serial.println(cbvr);
        Serial.println(cbhl);
        break;
      case FB1:
        sidestepR();
        break;
      case FB2:
        setServo(SVL, csvl, 208);
        setServo(SHR, cshr, 88);
        setServo(SVR, csvr, 105);
        setServo(SHL, cshl, 100);
        setServo(TVL, ctvl, 90);
        setServo(THR, cthr, 95);
        setServo(TVR, ctvr, 100);
        setServo(THL, cthl, 110);
        setServo(BVL, cbvl, 85);
        setServo(BHR, cbhr, 90);
        setServo(BVR, cbvr, 122);
        setServo(BHL, cbhl, 85);
        break;
      case FB3:
        GoTo(standpos);
        break;
      case FB4:
        Serial.println("FB4");
        sit();
        break;
      case FB5:
        GoTo(sitpos);
        break;
      case FBUP:
        setServoTB(5);
        Serial.println("FBUP");
        break;
      case FBDOWN:
        setServoTB(-5);
        Serial.println("FBDOWN");
        break;
      default:
        Serial.println("Unknown");
        break;
    }

    irrecv.resume();
  }
}

void sidestepR(){
  //Beine auseinander:
  //Bein heben
  setServo(BVR, cbvr, cbvr-10);
  setServo(BHL, cbhl, cbhl+10);
  delay(1000);
  //Bein zur Seite
  setServo(SVL, csvl, csvl+10);
  setServo(SHR, cshr, cshr-10);
  setServo(SVR, csvr, csvr-10);
  setServo(SHL, cshl, cshl+10);
  delay(1000);
  //Bein absetzen
  setServo(BVR, cbvr, cbvr+10);
  setServo(BHL, cbhl, cbhl-10);
  delay(5000);
  //Beine zusammen:
  setServo(BVL, cbvl, cbvl+10);
  setServo(BHR, cbhr, cbhr-10);
  delay(1000);
  setServo(SVL, csvl, csvl-10);
  setServo(SHR, cshr, cshr+10);
  setServo(SVR, csvr, csvr+10);
  setServo(SHL, cshl, cshl-10);
  delay(1000);
  setServo(BVL, cbvl, cbvl-10);
  setServo(BHR, cbhr, cbhr+10);
}