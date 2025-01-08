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
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>


#define SERVOMIN 125
#define SERVOMAX 600


//Pin of Servos
#define SFL 0
#define TFL 1
#define BFL 2
#define SBR 3
#define TBR 4
#define BBR 5
#define SFR 7
#define TFR 8
#define BFR 9
#define SBL 10
#define TBL 11
#define BBL 12

//Hex-Adressen für die IR Fernbedienung
#define FBPOWER 0xFFA25D
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
#define FBLEFT 0x22DD
#define FBRIGHT 0xFFC23D
#define FBVOLUP 0xFF629D
#define FBVOLDOWN 0xFFA857
#define FBPLAY 0xFF02FD
#define FBEQ 0xFF9867
#define FBREPT 0xFFB04F
#define FBFUNC 0xFFE21D

const uint16_t RECV_PIN = 4;
//Variablen für Fernbedienung
int controlmode = 0;
int selectedServo;

Adafruit_PWMServoDriver servoDriver_module = Adafruit_PWMServoDriver(0x40);
IRrecv irrecv(RECV_PIN);
decode_results results;
Adafruit_MPU6050 mpu;

int angleToPulse(int ang);

//current positions
int cSFL, cTFL, cBFL, cSBR, cTBR, cBBR, cSFR, cTFR, cBFR, cSBL, cTBL, cBBL;

//neutral positions
const int nSFL = 202;//197
const int nTFL = 125;
const int nBFL = 20;
const int nSBR = 95;//90
const int nTBR = 60;
const int nBBR = 155;
const int nSFR = 105;//110
const int nTFR = 71;
const int nBFR = 180;
const int nSBL = 92;//97
const int nTBL = 140;
const int nBBL = 20;

//Standing Positions
const int sSFL = nSFL;
const int sTFL = 85;
const int sBFL = 100;
const int sSBR = nSBR;
const int sTBR = 100;
const int sBBR = 75;
const int sSFR = nSFR;
const int sTFR = 111;
const int sBFR = 100;
const int sSBL = nSBL;
const int sTBL = 100;
const int sBBL = 100;

//Bools for postions
bool sitting = false;
bool standing = false;
bool walking = false;

//Gyroscope offset values
float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

uint8_t remoteMac[6] = {0x30, 0xC9, 0x22, 0xEC, 0xBE, 0xAC};
String success;
int testnumber = 3;

void createPosStrings();    //Erstellt die Strings für die Positionen mithilfe der Variablen
char sitpos[100];           
char standpos[100];
void home();                    //Ausgansposition (Entspricht sit)
void sit();                     //Fährt in Liegeposition
void stand();                   //Fährt in Stehposition
void GoTo(const char* position);//Nimmt einen String mit 12 Positionen und fährt  
void sidestepR(); 
void rotateL();
void walk();
void walkback();
void correctAll();              //Hebt die Beine kurz an um unebenheiten auszugleichen (Alpha)
void setServo(int motor, int &currmotor, int angle);  //Bewegt einen Servo
void setServoS(int GoUp);                             //Bewegt alle Seitlichen Servos
void setServoT(int GoUp);                             //Bewegt alle Top Servos
void setServoB(int GoUp);                             //Bewegt alle Bottom Servos
void setServoTB(int GoUp);                            //Bewegt Top und Bottom Servos
void readMacAdress();
void onDataReceive(const uint8_t * mac, const uint8_t * data, int len);
void checkIR();
void moveServo(int selectedServo, int updown); //Bewegt einen einzelnen Servo in 5er schritten //updown: 1 = up, -1 = down
void gyrosetup();
void gyroread();
void calibrateGyro();


void setup() {
  Serial.begin(9600);
  servoDriver_module.begin();
  servoDriver_module.setPWMFreq(50);    //Arbeitsfrequenz
  createPosStrings();
  home();
  irrecv.enableIRIn(); //Infrarotfernbedienung
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
  //gyrosetup();
}

void loop() {
  checkIR();
  //gyroread();
  /*delay(5000);
  esp_err_t result = esp_now_send(remoteMac, (uint8_t *) &testnumber, sizeof(testnumber));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }*/
  if (walking){
    walk();
  }
}

int angleToPulse(int ang){
  int pulse = map(ang,0, 270, SERVOMIN,SERVOMAX);  
  Serial.print("Angle: ");Serial.print(ang);
  Serial.print(" pulse: ");Serial.println(pulse);
  return pulse;
}

void createPosStrings(){
snprintf(sitpos, sizeof(sitpos), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", nSFL, nTFL, nBFL, nSBR, nTBR, nBBR, nSFR, nTFR, nBFR, nSBL, nTBL, nBBL);
snprintf(standpos, sizeof(standpos), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", sSFL, sTFL, sBFL,sSBR, sTBR, sBBR, sSFR, sTFR, sBFR, sSBL, sTBL, sBBL);
}

void home() {
  setServo(SFL, cSFL, nSFL);
  setServo(SBR, cSBR, nSBR);
  setServo(SFR, cSFR, nSFR);
  setServo(SBL, cSBL, nSBL);
  setServo(TFL, cTFL, nTFL);
  setServo(TBR, cTBR, nTBR);
  setServo(TFR, cTFR, nTFR);
  setServo(TBL, cTBL, nTBL);
  setServo(BFL, cBFL, nBFL);
  setServo(BBR, cBBR, nBBR);
  setServo(BFR, cBFR, nBFR);
  setServo(BBL, cBBL, nBBL);
  sitting = true;
  standing = false;
  return;
}

void sit(){ //Überflüssig?
  int diff = abs(cTFL - nTFL);
  for (int i = 0; i<diff; i+=2){
    if (diff - i ==1){
      setServoTB(-1);
    }else{
      setServoTB(-2);
    }
    delay(20);
  }
  sitting = true;
  standing = false;
}

//Steht auf in 3 Schritten
void stand() {
  if(sitting){ 
    setServo(SFL, cSFL, sSFL); //Seiten ausrichten
    setServo(SBR, cSBR, sSBR);
    setServo(SFR, cSFR, sSFR);
    setServo(SBL, cSBL, sSBL);
    setServo(TFL, cTFL, cTFL-((cTFL-sTFL)/2)); //Top halb ausrichten
    setServo(TBR, cTBR, cTBR-((cTBR-sTBR)/2));
    setServo(TFR, cTFR, cTFR-((cTFR-sTFR)/2));
    setServo(TBL, cTBL, cTBL-((cTBL-sTBL)/2));
    setServo(BFL, cBFL, cBFL-((cBFL-sBFL)/3));  //Bottom halb ausrichten
    setServo(BBR, cBBR, cBBR-((cBBR-sBBR)/3));
    setServo(BFR, cBFR, cBFR-((cBFR-sBFR)/3));
    setServo(BBL, cBBL, cBBL-((cBBL-sBBL)/3));
    delay(300);
    setServo(TFL, cTFL, sTFL); //Top ausrichten
    setServo(TBR, cTBR, sTBR);
    setServo(TFR, cTFR, sTFR);
    setServo(TBL, cTBL, sTBL);
    delay(300);
    setServo(BFL, cBFL, sBFL);  //Bottom ausrichten
    setServo(BBR, cBBR, sBBR);
    setServo(BFR, cBFR, sBFR);
    setServo(BBL, cBBL, sBBL);
  }
  sitting = false;
  standing = true;
}

void GoTo(const char* positions) {
  int stepSize = 5;
  bool allServosAtTarget = false;

  int targetPositions[12];
  int index = 0;
  int start = 0;
  //Legt zuerst die Zielpositionen aller Servos in ein Array targetPositions
  for (int i = 0; i < strlen(positions); i++) {
    if (positions[i] == ',') {
      char buffer[10]; 
      strncpy(buffer, positions + start, i - start);
      buffer[i - start] = '\0'; 
      targetPositions[index++] = atoi(buffer);
      start = i + 1;
    }
  }
  char buffer[10];
  strncpy(buffer, positions + start, strlen(positions) - start);
  buffer[strlen(positions) - start] = '\0';
  targetPositions[index] = atoi(buffer);
  //Vergleicht Soll und Ist der einzelnen Motoren und passt sie in stepSize-Schritten an
  while (!allServosAtTarget) {
    allServosAtTarget = true;

    // Front Left
    if (abs(cSFL - targetPositions[0]) < stepSize) {
      setServo(SFL, cSFL, targetPositions[0]);
    } else if (cSFL < targetPositions[0]) {
      setServo(SFL, cSFL, cSFL + stepSize);
      allServosAtTarget = false;
    } else if (cSFL > targetPositions[0]) {
      setServo(SFL, cSFL, cSFL - stepSize);
      allServosAtTarget = false;
    }

    if (abs(cTFL - targetPositions[1]) < stepSize) {
      setServo(TFL, cTFL, targetPositions[1]);
    } else if (cTFL < targetPositions[1]) {
      setServo(TFL, cTFL, cTFL + stepSize);
      allServosAtTarget = false;
    } else if (cTFL > targetPositions[1]) {
      setServo(TFL, cTFL, cTFL - stepSize);
      allServosAtTarget = false;
    }

    if (abs(cBFL - targetPositions[2]) < stepSize*2) {
      setServo(BFL, cBFL, targetPositions[2]);
    } else if (cBFL < targetPositions[2]) {
      setServo(BFL, cBFL, cBFL + stepSize*2);
      allServosAtTarget = false;
    } else if (cBFL > targetPositions[2]) {
      setServo(BFL, cBFL, cBFL - stepSize*2);
      allServosAtTarget = false;
    }

    // Back Right
    if (abs(cSBR - targetPositions[3]) < stepSize) {
      setServo(SBR, cSBR, targetPositions[3]);
    } else if (cSBR < targetPositions[3]) {
      setServo(SBR, cSBR, cSBR + stepSize);
      allServosAtTarget = false;
    } else if (cSBR > targetPositions[3]) {
      setServo(SBR, cSBR, cSBR - stepSize);
      allServosAtTarget = false;
    }

    if (abs(cTBR - targetPositions[4]) < stepSize) {
      setServo(TBR, cTBR, targetPositions[4]);
    } else if (cTBR < targetPositions[4]) {
      setServo(TBR, cTBR, cTBR + stepSize);
      allServosAtTarget = false;
    } else if (cTBR > targetPositions[4]) {
      setServo(TBR, cTBR, cTBR - stepSize);
      allServosAtTarget = false;
    }

    if (abs(cBBR - targetPositions[5]) < stepSize*2) {
      setServo(BBR, cBBR, targetPositions[5]);
    } else if (cBBR < targetPositions[5]) {
      setServo(BBR, cBBR, cBBR + stepSize*2);
      allServosAtTarget = false;
    } else if (cBBR > targetPositions[5]) {
      setServo(BBR, cBBR, cBBR - stepSize*2);
      allServosAtTarget = false;
    } 

    // Front Right
    if (abs(cSFR - targetPositions[6]) < stepSize) {
      setServo(SFR, cSFR, targetPositions[6]);
    } else if (cSFR < targetPositions[6]) {
      setServo(SFR, cSFR, cSFR + stepSize);
      allServosAtTarget = false;
    } else if (cSFR > targetPositions[6]) {
      setServo(SFR, cSFR, cSFR - stepSize);
      allServosAtTarget = false;
    }

    if (abs(cTFR - targetPositions[7]) < stepSize) {
      setServo(TFR, cTFR, targetPositions[7]);
    } else if (cTFR < targetPositions[7]) {
      setServo(TFR, cTFR, cTFR + stepSize);
      allServosAtTarget = false;
    } else if (cTFR > targetPositions[7]) {
      setServo(TFR, cTFR, cTFR - stepSize);
      allServosAtTarget = false;
    } 

    if (abs(cBFR - targetPositions[8]) < stepSize*2) {
      setServo(BFR, cBFR, targetPositions[8]);
    } else if (cBFR < targetPositions[8]) {
      setServo(BFR, cBFR, cBFR + stepSize*2);
      allServosAtTarget = false;
    } else if (cBFR > targetPositions[8]) {
      setServo(BFR, cBFR, cBFR - stepSize*2);
      allServosAtTarget = false;
    } 

    // Back Left
    if (abs(cSBL - targetPositions[9]) < stepSize) {
      setServo(SBL, cSBL, targetPositions[9]);
    } else if (cSBL < targetPositions[9]) {
      setServo(SBL, cSBL, cSBL + stepSize);
      allServosAtTarget = false;
    } else if (cSBL > targetPositions[9]) {
      setServo(SBL, cSBL, cSBL - stepSize);
      allServosAtTarget = false;
    }

    if (abs(cTBL - targetPositions[10]) < stepSize) {
      setServo(TBL, cTBL, targetPositions[10]);
    } else if (cTBL < targetPositions[10]) {
      setServo(TBL, cTBL, cTBL + stepSize);
      allServosAtTarget = false;
    } else if (cTBL > targetPositions[10]) {
      setServo(TBL, cTBL, cTBL - stepSize);
      allServosAtTarget = false;
    } 

    if (abs(cBBL - targetPositions[11]) < stepSize*2) {
      setServo(BBL, cBBL, targetPositions[11]);
    } else if (cBBL < targetPositions[11]) {
      setServo(BBL, cBBL, cBBL + stepSize*2);
      allServosAtTarget = false;
    } else if (cBBL > targetPositions[11]) {
      setServo(BBL, cBBL, cBBL - stepSize*2);
      allServosAtTarget = false;
    } 

    delay(20);
  }
}

void setServo(int motor, int &currmotor, int angle){
  servoDriver_module.setPWM(motor, 0, angleToPulse(angle));
  currmotor = angle;
  delay(20);
}

void setServoS(int GoUp){
  setServo(SFL, cSFL, (cSFL+GoUp));
  setServo(SBR, cSBR, (cSBR+GoUp));
  setServo(SFR, cSFR, (cSFR+GoUp));
  setServo(SBL, cSBL, (cSBL+GoUp));
}

void setServoT(int GoUp){
  //Rechts: hoch addieren, Links: hoch subtrahieren
  setServo(TFL, cTFL, (cTFL-GoUp));
  setServo(TBR, cTBR, (cTBR+GoUp));
  setServo(TFR, cTFR, (cTFR+GoUp));
  setServo(TBL, cTBL, (cTBL-GoUp));
}

void setServoB(int GoUp){
  //Rechts: hoch subtrahieren, Links: hoch addieren
  setServo(BFL, cBFL, cBFL+GoUp);
  setServo(BBR, cBBR, cBBR-GoUp);
  setServo(BFR, cBFR, cBFR-GoUp);
  setServo(BBL, cBBL, cBBL+GoUp);
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
      case FBVOLUP:
        Serial.println(cSFL);
        Serial.println(cTFL);
        Serial.println(cBFL);
        Serial.println(cSBR);
        Serial.println(cTBR);
        Serial.println(cBBR);
        Serial.println(cSFR);
        Serial.println(cTFR);
        Serial.println(cBFR);
        Serial.println(cSBL);
        Serial.println(cTBL);
        Serial.println(cBBL);
        break;
      case FB1:
        if(controlmode == 0){
          sidestepR();
          } else if(controlmode == 1){
            selectedServo = SFL;
            printf("Selected Servo: %d\n", selectedServo);
          }
        break;
      case FB2:
        if(controlmode == 0){
          stand();
        } else if(controlmode == 1){
          selectedServo = TFL;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB3:
        if(controlmode == 0){
          GoTo(standpos);
          sitting = false;
          standing = true;
        } else if (controlmode == 1){
          selectedServo = BFL;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB4:
        if(controlmode == 0){
          walk();
        } else if (controlmode == 1){
          selectedServo = SBR;
          printf("Selected Servo: %d\n", selectedServo);
        }  
        break;
      case FB5:
        if(controlmode == 0){
          GoTo(sitpos);
          sitting = true;
          standing = false;
        } else if (controlmode == 1){
          selectedServo = TBR;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB6:
      if(controlmode == 0){
        walking = !walking;
      } else if (controlmode ==1){
          selectedServo = BBR;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB7:
        if (controlmode == 0){
          walkback();
        } else if (controlmode == 1){
          selectedServo = SFR;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB8:
        if (controlmode == 1){
          selectedServo = TFR;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB9:
        if (controlmode == 1){
          selectedServo = BFR;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB0:
        if (controlmode == 1){
          selectedServo = SBL;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FBEQ:
        if (controlmode == 1){
          selectedServo = TBL;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FBREPT:
        if (controlmode == 1){
          selectedServo = BBL;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FBUP:
        if(controlmode == 0){
          setServoTB(5);
          Serial.println("FBUP");
        } else if(controlmode ==1){
          moveServo(selectedServo, 1);
        }
        break;
      case FBDOWN:
        if(controlmode == 0){
          setServoTB(-5);
          Serial.println("FBDOWN");
        } else if(controlmode == 1){
          moveServo(selectedServo, -1);
        }
        break;
      case FBFUNC:
        if(controlmode == 0){
          controlmode = 1;
          Serial.println("Controlmode: 1");
        }else{
          controlmode = 0;
          Serial.println("Controlmode: 0");
        }
        break;
      default:
        Serial.println("Unknown");
        break;
    }

    irrecv.resume();
  }
}

void moveServo(int selectedServo, int updown){
  int step = 5*updown;
  switch(selectedServo){
    case SFL:
      setServo(SFL, cSFL, cSFL+step);
      break;
    case TFL:
      setServo(TFL, cTFL, cTFL-step);
      break;
    case BFL:
      setServo(BFL, cBFL, cBFL+step);
      break;
    case SBR:
      setServo(SBR, cSBR, cSBR+step);
      break;
    case TBR: 
      setServo(TBR, cTBR, cTBR+step);
      break;
    case BBR:
      setServo(BBR, cBBR, cBBR-step);
      break;
    case SFR:
      setServo(SFR, cSFR, cSFR+step);
      break;
    case TFR:
      setServo(TFR, cTFR, cTFR+step);
      break;
    case BFR:
      setServo(BFR, cBFR, cBFR-step);
      break;
    case SBL:
      setServo(SBL, cSBL, cSBL+step);
      break;
    case TBL:
      setServo(TBL, cTBL, cTBL-step);
      break;
    case BBL:
      setServo(BBL, cBBL, cBBL+step);
      break;
    default:
      Serial.println("What?");
      break;
  }
}
void sidestepR(){
  if (standing){
    setServo(BFR, cBFR, cBFR+50);
    setServo(SFR, cSFR, cSFR-10);
    setServo(BFR, cBFR, cBFR-50);
    delay(500);
    setServo(BBL, cBBL, cBBL-50);
    setServo(SBL, cSBL, cSBL+10);
    setServo(BBL, cBBL, cBBL+50);
    delay(500);

    setServo(BFL, cBFL, cBFL-50);
    setServo(SFL, cSFL, cSFL-10);
    setServo(BFL, cBFL, cBFL+50);
    delay(500);
    setServo(BBR, cBBR, cBBR+50);
    setServo(SBR, cSBR, cSBR+10);
    setServo(BBR, cBBR, cBBR-50);
    delay(500);
    setServo(SFR, cSFR, cSFR+10);
    setServo(SBL, cSBL, cSBL-10);
    setServo(SFL, cSFL, cSFL+10);
    setServo(SBR, cSBR, cSBR-10);
  }
}

void rotateL(){
  setServo(BFR, cBFR, cBFR+50);
  delay(20);
  setServo(SFR, cSFR, cSFR+10);
  delay(20);
  setServo(BFR, cBFR, cBFR-50);
  delay(200);
  setServo(BBL, cBBL, cBBL-50);
  delay(20);
  setServo(SBL, cSBL, cSBL+10);
  delay(20);
  setServo(BBL, cBBL, cBBL+50);
  delay(200);
  
  setServo(BFL, cBFL, cBFL-50);
  delay(20);
  setServo(SFL, cSFL, cSFL+10);
  delay(20);
  setServo(BFL, cBFL, cBFL+50);
  delay(200);
  setServo(BBR, cBBR, cBBR+50);
  delay(20);
  setServo(SBR, cSBR, cSBR+10);
  delay(20);
  setServo(BBR, cBBR, cBBR-50);
  delay(200);
  setServo(SFR, cSFR, cSFR-10);
  setServo(SBL, cSBL, cSBL-10);
  setServo(SFL, cSFL, cSFL-10);
  setServo(SBR, cSBR, cSBR-10);
}

void walk(){
  setServo(BFR, cBFR, cBFR+20);
  setServo(TFR, cTFR, sTFR+15);
  delay(100);
  setServo(BFR, cBFR, sBFR-5);
  // setServo(TFR, cTFR, sTFR);
  setServoT(-5);
  delay(100);

  setServo(BBL, cBBL, cBBL-20);
  setServo(TBL, cTBL, sTBL-5);
  delay(100);
  setServo(BBL, cBBL, sBBL);
  // setServo(TBL, cTBL, sTBL);
  setServoT(-5);
  delay(100);
    
  setServo(BFL, cBFL, cBFL-25);
  setServo(TFL, cTFL, sTFL-5);
  delay(100);
  setServo(BFL, cBFL, sBFL);
  // setServo(TFL, cTFL, sTFL);
  setServoT(-5);
  delay(100);

  setServo(BBR, cBBR, cBBR+15);
  setServo(TBR, cTBR, sTBR);
  delay(100);
  setServo(BBR, cBBR, sBBR);
  // setServo(TBR, cTBR, sTBR);
  // delay(200);

  // setServo(TFR, cTFR, sTFR+10);
  // setServo(TFL, cTFL, sTFL+10);
  // setServo(TBR, cTBR, sTBR+10);
  // setServo(TBL, cTBL, sTBL+10);


  delay(100);
  GoTo(standpos);

}

void walkback(){
  setServo(BFR, cBFR, cBFR+20);
  setServo(TFR, cTFR, sTFR-20);
  delay(100);
  setServo(BFR, cBFR, sBFR-5);
  // setServo(TFR, cTFR, sTFR);
  setServoT(+5);
  delay(100);

  setServo(BBL, cBBL, cBBL-20);
  setServo(TBL, cTBL, sTBL+10);
  delay(100);
  setServo(BBL, cBBL, sBBL);
  // setServo(TBL, cTBL, sTBL);
  setServoT(+5);
  delay(100);
    
  setServo(BFL, cBFL, cBFL-25);
  setServo(TFL, cTFL, sTFL+5);
  delay(100);
  setServo(BFL, cBFL, sBFL);
  // setServo(TFL, cTFL, sTFL);
  setServoT(+5);
  delay(100);

  setServo(BBR, cBBR, cBBR+25);
  setServo(TBR, cTBR, sTBR-5);
  delay(100);
  setServo(BBR, cBBR, sBBR-10);
  // setServo(TBR, cTBR, sTBR);
  // delay(200);

  // setServo(TFR, cTFR, sTFR+10);
  // setServo(TFL, cTFL, sTFL+10);
  // setServo(TBR, cTBR, sTBR+10);
  // setServo(TBL, cTBL, sTBL+10);


  delay(100);
  GoTo(standpos);

}

void correctAll(){
GoTo(standpos);
delay(100);
setServoTB(-10);
delay(100);
setServoTB(10);
}

/*void gyrosetup(){
  int breakcounter = 0;
  if (!mpu.begin()){
    Serial.println("Sensor init failed");
    while(1){
      delay(10);
      breakcounter++;
      if (breakcounter == 100){
        break;
      }
    }
  }
  if (breakcounter == 100){
    Serial.println("Canceled gyro setup");
  } else{
  Serial.println("Found MPU6050");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()){
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()){
    case MPU6050_RANGE_250_DEG:
      Serial.println("+-250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+-500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+-1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+-2000 deg/s");
      break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()){
    case MPU6050_BAND_260_HZ:
      Serial.println("260Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5Hz");
      break;
  }
  Serial.println("");
  calibrateGyro();
  delay(100);
}
*/
/*void gyroread(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelX = a.acceleration.x - accelXOffset;
  float accelY = a.acceleration.y - accelYOffset;
  float accelZ = a.acceleration.z - accelZOffset;
  float gyroX = g.gyro.x - gyroXOffset;
  float gyroY = g.gyro.y - gyroYOffset;
  float gyroZ = g.gyro.z - gyroZOffset;

  Serial.print("Acceleration X: "); Serial.print(accelX); Serial.print(" m/s^2");
  Serial.print(" Acceleration Y: "); Serial.print(accelY); Serial.print(" m/s^2");
  Serial.print(" Acceleration Z: "); Serial.print(accelZ); Serial.print(" m/s^2");
  Serial.println("");

  Serial.print("Gyro X: "); Serial.print(gyroX); Serial.print(" rad/s");
  Serial.print(" Gyro Y: "); Serial.print(gyroY); Serial.print(" rad/s");
  Serial.print(" Gyro Z: "); Serial.print(gyroZ); Serial.print(" rad/s");
  Serial.println("");
  delay(500);
}

void calibrateGyro(){
  
  const int numReadings = 1000;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accelXSum += a.acceleration.x;
    accelYSum += a.acceleration.y;
    accelZSum += a.acceleration.z;
    gyroXSum += g.gyro.x;
    gyroYSum += g.gyro.y;
    gyroZSum += g.gyro.z;

    delay(10);
  }

  accelXOffset = accelXSum / numReadings;
  accelYOffset = accelYSum / numReadings;
  accelZOffset = accelZSum / numReadings;
  gyroXOffset = gyroXSum / numReadings;
  gyroYOffset = gyroYSum / numReadings;
  gyroZOffset = gyroZSum / numReadings;

  Serial.println("Calibration complete");
  Serial.print("Accel Offsets: "); Serial.print(accelXOffset); Serial.print(", "); Serial.print(accelYOffset); Serial.print(", "); Serial.println(accelZOffset);
  Serial.print("Gyro Offsets: "); Serial.print(gyroXOffset); Serial.print(", "); Serial.print(gyroYOffset); Serial.print(", "); Serial.println(gyroZOffset);
}*/