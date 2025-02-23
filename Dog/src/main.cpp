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
#include "functions.h"

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

#define L1 8.4   // Oberschenkellänge (cm)
#define L2 12.2  // Unterschenkellänge (cm)
#define Z_STAND 15.0  // Ziel-Hüfthöhe für gebeugte Standhaltung
#define X_OFFSET 2.0  // Füße leicht vor der Hüfte für Stabilität
#define Y_OFFSET 2.0  // Abstand der Füße zur Mitte (seitlich)


const uint16_t RECV_PIN = 4;
//Variablen für Fernbedienung
int controlmode = 0;
int selectedServo;

Adafruit_PWMServoDriver servoDriver_module = Adafruit_PWMServoDriver(0x40);
IRrecv irrecv(RECV_PIN);
decode_results results;
Adafruit_MPU6050 mpu;



//current positions
int cSFL = 0, cTFL = 0, cBFL = 0, cSBR = 0, cTBR = 0, cBBR = 0, cSFR = 0, cTFR = 0, cBFR = 0, cSBL = 0, cTBL = 0, cBBL = 0;
int* cPositions[13] = {&cSFL, &cTFL, &cBFL, &cSBR, &cTBR, &cBBR,nullptr, &cSFR, &cTFR, &cBFR, &cSBL, &cTBL, &cBBL};


//neutral positions
const int servoOffsets[13] = {146, 125, 20, 150, 140, 30, -1, 99, 160, 185, 30, 60, 155}; //Offset
const int nSFL = 0;
const int nTFL = 0;
const int nBFL = 0;
const int nSBR = 0;
const int nTBR = 0;
const int nBBR = 0;
const int nSFR = 0;
const int nTFR = 0;
const int nBFR = 0;
const int nSBL = 0;
const int nTBL = 0;
const int nBBL = 0;
const int nPositions[13] = {nSFL, nTFL, nBFL, nSBR, nTBR, nBBR,-1, nSFR, nTFR, nBFR, nSBL, nTBL, nBBL};

//Standing Positions
const int sSFL = nSFL;
const int sTFL = nTFL-30;
const int sBFL = nBFL+60;
const int sSBR = nSBR;
const int sTBR = nTBR-30;
const int sBBR = nBBR+60;
const int sSFR = nSFR;
const int sTFR = nTFR+30;
const int sBFR = nBFR-60;
const int sSBL = nSBL;
const int sTBL = nTBL+30;
const int sBBL = nBBL-60;
const int sPositions[13] = {sSFL, sTFL, sBFL, sSBR, sTBR, sBBR,-1, sSFR, sTFR, sBFR, sSBL, sTBL, sBBL};

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

void waitforButton();
bool paused = false;


const int sitpos[12] = {nSFL, nTFL, nBFL, nSBR, nTBR, nBBR, nSFR, nTFR, nBFR, nSBL, nTBL, nBBL};
const int standpos[12] = {sSFL, sTFL, sBFL, sSBR, sTBR, sBBR, sSFR, sTFR, sBFR, sSBL, sTBL, sBBL};
const int slideright2[12] = {175,90,90,158,109,72,97,191,143,0,95,85};
const int slideright[12] = {nSFL+20, nTFL-35, nBFL+70, nSBR-2, nTBR-31, nBBR+42, nSFR+2, nTFR+31, nBFR-42, nSBL-15, nTBL+35, nBBL-70};

char command[10];
int idx = 0;

void readMacAdress();
void onDataReceive(const uint8_t * mac, const uint8_t * data, int len);
void checkIR();
void gyrosetup();
void gyroread();
void calibrateGyro();
void computeIK(int legID,float x, float y, float z, float &theta1, float &theta2, float &theta3);
void moveLegToPosition(int legID, float x, float y, float z);
void setStandingPose();

void setup() {
  Serial.begin(9600);
  servoDriver_module.begin();
  servoDriver_module.setPWMFreq(50);    //Arbeitsfrequenz
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    Serial.println(addPeerResult);
    return;
  }else{
    Serial.println("Peer added");
  }
  //gyrosetup();
}

void loop() {
  checkIR();
  //gyroread();
  if (walking){
    walk();
  }
  // for (int angle = 0; angle <= 270; angle++) {
  //     setServo(SFL, cSFL, angle);
  //     delay(15);  // Delay for smooth movement
  // }
  // for (int angle = 270; angle >= 0; angle--) {
  //     setServo(SFL, cSFL, angle);
  //     delay(15);  // Delay for smooth movement
  // }
  if (Serial.available())
  {
    char c = Serial.read();
    if (idx == 10)
    {                 //Wenn 20 Zeichen in command sind, wird der buffer gelöscht
      idx = 0;
      for (int i = 0; i < 10; i++)
      {
        command[i] = '\0';
      }
      Serial.println("Buffer overflow, reset");
    }
    else if (c == '\n') //Enter
    { 
      setServo(BFL, atoi(command));
      idx = 0;
      for (int i = 0; i < 10; i++)
      {
        command[i] = '\0';
      }
    }
    else if (c == '\r')
    { 
    }
    else
    { 
      command[idx] = c;
      idx++;
    }
  }
}




void waitforButton(){
  irrecv.resume();
  paused = true;
  delay(1000);
  while (paused)(checkIR());
}



//Steht auf in 3 Schritten

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
  if (receivedNumber == 69){
    esp_err_t result = esp_now_send(remoteMac, (uint8_t *) &walking, sizeof(walking));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
  }
}

void checkIR(){
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    switch (results.value){
      case FBVOLUP:
        Serial.printf("SFL = %d\n",cSFL);
        Serial.printf("TFL = %d\n",cTFL);
        Serial.printf("BFL = %d\n",cBFL);
        Serial.printf("SBR = %d\n",cSBR);
        Serial.printf("TBR = %d\n",cTBR);
        Serial.printf("BBR = %d\n",cBBR);
        Serial.printf("SFR = %d\n",cSFR);
        Serial.printf("TFR = %d\n",cTFR);
        Serial.printf("BFR = %d\n",cBFR);
        Serial.printf("SBL = %d\n",cSBL);
        Serial.printf("TBL = %d\n",cTBL);
        Serial.printf("BBL = %d\n",cBBL);
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
      if (controlmode == 0){
        //Zum Testen einzelner Bewegungen
        setServo(TFL, (cTFL-5));
        setServo(TBR, (cTBR+5));
        setServo(TFR, (cTFR-5));
        setServo(TBL, (cTBL+5));
      } else
        if (controlmode == 1){
          selectedServo = TFR;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB9:
        if (controlmode == 0){
          setStandingPose();
        } else
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
      case FBPLAY:
        paused = false;
      default:
        Serial.println("Unknown");
        break;
    }

    irrecv.resume();
  }
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




// Globale Variablen für die aktuellen Servo-Winkel

  

// Funktion zur Berechnung der inversen Kinematik für ein Bein in 3D
void computeIK(int legID, float x, float y, float z, float &theta1, float &theta2, float &theta3) {
  if (legID ==0 || legID == 2) {
    x = -x;  // Spiegelung der hinteren Beine
  }
  if (legID == 0 || legID == 3) {  //Linke Beine 
      theta1 = atan2(y, z) * 180.0 / M_PI;
  } else {  // Rechte Beine
      theta1 = -atan2(y, z) * 180.0 / M_PI;
  }

  float d = sqrt(x*x + z*z);  // Abstand von Hüfte zum Fuß in 2D

  float cosTheta3 = (L1*L1 + L2*L2 - d*d) / (2 * L1 * L2);
  theta3 = acos(cosTheta3) * 180.0 / M_PI;  // Kniewinkel in Grad

  float alpha = atan2(x, z);
  float beta = acos((L1*L1 + d*d - L2*L2) / (2 * L1 * d));
  theta2 = 90-((alpha+beta) * 180.0 / M_PI);  // Hüftwinkel in Grad
}

// Servo-Mapping (Pin-Nummern überspringen 6)
int servoMap[12] = {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12};

// Funktion zum Bewegen eines Beins mit Differenz-Winkel
void moveLegToPosition(int legID, float x, float y, float z) {
  float theta1, theta2, theta3;
  int nextPos[12] = {cSFL, cTFL, cBFL, cSBR, cTBR, cBBR, cSFR, cTFR, cBFR, cSBL, cTBL, cBBL};
  int currentTheta1[4] = {cSFL, cSBR, cSFR, cSBL};  
  int currentTheta2[4] = {cTFL, cTBR, cTFR, cTBL};  
  int currentTheta3[4] = {cBFL, cBBR, cBFR, cBBL};  

  computeIK(legID, x, y, z, theta1, theta2, theta3);
  // Winkelvorzeichen je nach bein anpassen
  if (legID == 0 || legID == 1){
    theta2 = -theta2;
  }
  if (legID == 2 || legID == 3){
    theta3 = -theta3;
  }
  if (legID == 1 || legID == 3){
    theta1 = -theta1;
  }
  // Differenz zum aktuellen Winkel berechnen
  int deltaTheta1 = round(theta1) - currentTheta1[legID];
  int deltaTheta2 = round(theta2) - currentTheta2[legID];
  int deltaTheta3 = round(theta3) - currentTheta3[legID];
  
  Serial.print("currentTheta1: "); Serial.println(currentTheta1[legID]);
  Serial.print("currentTheta2: "); Serial.println(currentTheta2[legID]);
  Serial.print("currentTheta3: "); Serial.println(currentTheta3[legID]);
  Serial.print("Theta1: "); Serial.println(theta1);
  Serial.print("Theta2: "); Serial.println(theta2);
  Serial.print("Theta3: "); Serial.println(theta3);
  Serial.print("Delta Theta1: "); Serial.println(deltaTheta1);
  Serial.print("Delta Theta2: "); Serial.println(deltaTheta2);
  Serial.print("Delta Theta3: "); Serial.println(deltaTheta3);
  // Servos um die Differenz bewegen
  nextPos[legID * 3] = currentTheta1[legID] + deltaTheta1;
  nextPos[legID * 3 + 1] = currentTheta2[legID] + deltaTheta2;
  nextPos[legID * 3 + 2] = currentTheta3[legID] + deltaTheta3;

  Serial.print("Next Positions: [");
  for (int i = 0; i < 12; i++) {
    Serial.print(nextPos[i]);
    if (i < 11) {
      Serial.print(", ");
    }
  }
  Serial.println("]");
  // setServo(servoMap[legID * 3], currentTheta1[legID] + deltaTheta1);
  // setServo(servoMap[legID * 3 + 1], currentTheta2[legID] + deltaTheta2);
  // setServo(servoMap[legID * 3 + 2], currentTheta3[legID] + deltaTheta3);
  GoTo(nextPos);
  // Neue Winkelwerte speichern --> muss nochmal extra in currentTheta gespeichert werden
  // currentTheta1[legID] += deltaTheta1;
  // currentTheta2[legID] += deltaTheta2;
  // currentTheta3[legID] += deltaTheta3;
}

// Funktion zum Einstellen der Standard-Standposition für alle 4 Beine
void setStandingPose() {
  moveLegToPosition(0, X_OFFSET, Y_OFFSET, Z_STAND);  // Links vorne (normal)
  moveLegToPosition(1, -X_OFFSET, Y_OFFSET, Z_STAND); // Rechts hinten (invertiert)
  moveLegToPosition(2, X_OFFSET, Y_OFFSET, Z_STAND); // Rechts vorne (normal)
  moveLegToPosition(3, -X_OFFSET, Y_OFFSET, Z_STAND);  // Links hinten (invertiert)
}

    