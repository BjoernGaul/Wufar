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
const int nshr = 93;
const int nsvr = 105;
const int nshl = 100;
const int ntvl = 130;
const int nthr = 60;
const int ntvr = 70;
const int nthl = 140;
const int nbvl = 15;
const int nbhr = 160;
const int nbvr = 190;
const int nbhl = 10;

//current positions
int csvl, cshr, csvr, cshl, ctvl, cthr, ctvr, cthl, cbvl, cbhr, cbvr, cbhl;

uint8_t remoteMac[6] = {0x30, 0xC9, 0x22, 0xEC, 0xBE, 0xAC};
String success;
int testnumber = 3;


void home();
void up();
void down();
void squat();
void setServo(int motor, int &currmotor, int angle);
void setServoS(int GoUp);
void setServoT(int GoUp);
void setServoB(int GoUp);
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
  //squat();
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

void up(){
  for (int i = 0; i<40; i+=5){
    setServo(BVL, cbvl, nbvl+(2*i));
    setServo(BHR, cbhr, nbhr-(2*i));
    setServo(BVR, cbvr, nbvr-(2*i));
    setServo(BHL, cbhl, nbhl+(2*i));

    setServo(TVL, ctvl, ntvl-i);
    setServo(THR, cthr, nthr+i);
    setServo(TVR, ctvr, ntvr+i);
    setServo(THL, cthl, nthl-i);
  }
}

void down(){
  for (int j = 0; j<40; j+=5){
    setServo(BVL, cbvl, cbvl-(2*j));
    setServo(BHR, cbhr, cbhr+(2*j));
    setServo(BVR, cbvr, cbvr+(2*j));
    setServo(BHL, cbhl, cbhl-(2*j));
    setServo(TVL, ctvl, ctvl+j);
    setServo(THR, cthr, cthr-j);
    setServo(TVR, ctvr, ctvr-j);
    setServo(THL, cthl, cthl+j);
    /*
    servoDriver_module.setPWM(BVL, 0, angleToPulse(nbvl+80-(2*j)));
    servoDriver_module.setPWM(BHR, 0, angleToPulse(nbhr-70+(2*j)));
    servoDriver_module.setPWM(BVR, 0, angleToPulse(nbvr-80+(2*j)));
    servoDriver_module.setPWM(BHL, 0, angleToPulse(nbhl+80-(2*j)));
    servoDriver_module.setPWM(TVL, 0, angleToPulse(ntvl-40+j));
    servoDriver_module.setPWM(THR, 0, angleToPulse(nthr+40-j));
    servoDriver_module.setPWM(TVR, 0, angleToPulse(ntvr+40-j));
    servoDriver_module.setPWM(THL, 0, angleToPulse(nthl-40+j));*/
  }
}

void squat(){
  up();
  delay(5000);
  down();
  delay(5000);
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
  setServo(BHR, cbhr, cbhr-GoUp*1.4);
  setServo(BVR, cbvr, cbvr-GoUp);
  setServo(BHL, cbhl, cbhl+GoUp*1.4);
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
        Serial.println("FB1");
        break;
      case FB2:
        Serial.println("FB2");
        break;
      case FB3:
        Serial.println("FB3");
        break;
      case FB4:
        Serial.println("FB4");
        break;
      case FB5:
        home();
        break;
      case FBUP:
        setServoT(5);
        setServoB(10);
        Serial.println("FBUP");
        break;
      case FBDOWN:
        setServoT(-5);
        setServoB(-10);
        Serial.println("FBDOWN");
        break;
      default:
        Serial.println("Unknown");
        break;
    }

    irrecv.resume();
  }
}