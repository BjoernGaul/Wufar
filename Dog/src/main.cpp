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
#include <LoRa.h>

//Pin of Servos
#define FLS 0
#define FLT 1
#define FLB 2
#define BRS 3
#define BRT 4
#define BRB 5
#define FRS 7
#define FRT 8
#define FRB 9
#define BLS 10
#define BLT 11
#define BLB 12
#define FLt 0
#define BRt 1
#define FRt 2
#define BLt 3

//Hex-Adressen for IR Remote
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
#define FBLEFT 0xFF22DD
#define FBRIGHT 0xFFC23D
#define FBVOLUP 0xFF629D
#define FBVOLDOWN 0xFFA857
#define FBPLAY 0xFF02FD
#define FBEQ 0xFF9867
#define FBREPT 0xFFB04F
#define FBFUNC 0xFFE21D

#define L1 8.4   // Upper Leg Length (cm)
#define L2 12.2  // Lower Leg Length (cm)
#define Z_STAND 12.5  // Hip Height while standing (cm)
#define X_OFFSET 2.0  // Feet slightly in front of the hip
#define Y_OFFSET 1.5  // Distance foot to hip

//LoRa
#define DIO0 D3
#define NSS D9
#define frequency 433E6
int LoRaType = 0; //0 = nothing, 1 = one value, 2 = array with 2 values
int LoRaValue = 0;
int* LoRaArray[2] = {0,0};
const uint8_t sitL = 0; //Array only have 1 Element: Value
const uint8_t standL = 1;
const uint8_t crab = 2;
const uint8_t LoFLS = 10;//Arrays have 2 Elements: Servo, Value
const uint8_t LoFLT = 11;
const uint8_t LoFLB = 12;
const uint8_t LoFRS = 20;
const uint8_t LoFRT = 21;
const uint8_t LoFRB = 22;
const uint8_t LoBRS = 30;
const uint8_t LoBRT = 31;
const uint8_t LoBRB = 32;
const uint8_t LoBLS = 40;
const uint8_t LoBLT = 41;
const uint8_t LoBLB = 42;
const uint8_t isStanding = 69;
const uint8_t reset = 255;

const uint16_t RECV_PIN = 4;
//Variables for IR Remote
int controlmode = 0;
int selectedServo;

Adafruit_PWMServoDriver servoDriver_module = Adafruit_PWMServoDriver(0x40);
IRrecv irrecv(RECV_PIN);
decode_results results;
Adafruit_MPU6050 mpu;
//Ultrasonic
const int trigPin = D6;
const int echoPin = D7;
float duration, distance;
//current positions
int cFLS = 0, cFLT = 0, cFLB = 0, cBRS = 0, cBRT = 0, cBRB = 0, cFRS = 0, cFRT = 0, cFRB = 0, cBLS = 0, cBLT = 0, cBLB = 0;
int* cPositions[13] = {&cFLS, &cFLT, &cFLB, &cBRS, &cBRT, &cBRB,nullptr, &cFRS, &cFRT, &cFRB, &cBLS, &cBLT, &cBLB};
int nextPos[12] = {cFLS, cFLT, cFLB, cBRS, cBRT, cBRB, cFRS, cFRT, cFRB, cBLS, cBLT, cBLB};

//neutral positions
const int servoOffsets[13] = {177, 175, 10, 150, 135, 30, -1, 99, 165, 185, 30, 60, 162}; //Offset
//zweiter offset: default 155
const int nFLS = 0;
const int nFLT = 0;
const int nFLB = 0;
const int nBRS = 0;
const int nBRT = 0;
const int nBRB = 0;
const int nFRS = 0;
const int nFRT = 0;
const int nFRB = 0;
const int nBLS = 0;
const int nBLT = 0;
const int nBLB = 0;
const int nPositions[13] = {nFLS, nFLT, nFLB, nBRS, nBRT, nBRB,-1, nFRS, nFRT, nFRB, nBLS, nBLT, nBLB};

//Standing Positions
const int sFLS = nFLS;
const int sFLT = nFLT-30;
const int sFLB = nFLB+60;
const int sBRS = nBRS;
const int sBRT = nBRT-30;
const int sBRB = nBRB+60;
const int sFRS = nFRS;
const int sFRT = nFRT+30;
const int sFRB = nFRB-60;
const int sBLS = nBLS;
const int sBLT = nBLT+30;
const int sBLB = nBLB-60;
const int sPositions[13] = {sFLS, sFLT, sFLB, sBRS, sBRT, sBRB,-1, sFRS, sFRT, sFRB, sBLS, sBLT, sBLB};

//Bools for postions
bool sitting = false;
bool standing = false;
bool walking = false;

//Gyroscope offset values
float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

//Other variables
void waitforButton();
bool paused = false;
bool singleLeg = true;
bool boppingTime = false;

//Position Arrays
const int sitpos[12] = {nFLS, nFLT, nFLB, nBRS, nBRT, nBRB, nFRS, nFRT, nFRB, nBLS, nBLT, nBLB};
const int standpos[12] = {sFLS, sFLT, sFLB, sBRS, sBRT, sBRB, sFRS, sFRT, sFRB, sBLS, sBLT, sBLB};
const int slideright2[12] = {175,90,90,158,109,72,97,191,143,0,95,85};
const int slideright[12] = {nFLS+20, nFLT-35, nFLB+70, nBRS-2, nBRT-31, nBRB+42, nFRS+2, nFRT+31, nFRB-42, nBLS-15, nBLT+35, nBLB-70};
//serial monitor input
char command[10];
int idx = 0;

void readMacAdress();
void onDataReceive(const uint8_t * mac, const uint8_t * data, int len);
void checkIR();
void gyrosetup();
void gyroread();
void calibrateGyro();

//LoRa functions
void LoRa_rxMode();
void LoRa_txMode();
void LoRa_sendMessage(String message);
void onReceive(int packetSize);
void onTxDone();
int* stringToIntArray(String str);

//Ultrasonic functions
float getDistance();
int distanceMillis;
bool displayDistance = false;
bool distanceFlag = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup Start");
  servoDriver_module.begin();
  servoDriver_module.setPWMFreq(50);    //operation frequency of the servos
  home();
  irrecv.enableIRIn(); //Infrared Remote
  Serial.println("IR enabled");
  //LoRa Setup
  delay(500);
  Serial.println("LoRa Start");
  LoRa.setPins( NSS , -1 , DIO0 );
  unsigned long startAttemptTime = millis();
  while (!LoRa.begin(frequency)) {
    if (millis() - startAttemptTime > 1000) {
      Serial.println("LoRa init failed. Check your connections.");
      break;
    }
  }
  if (millis() - startAttemptTime <= 1000) {
    Serial.println("LoRa init succeeded.");
  }
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  //Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //gyrosetup();
}

void loop() {
  checkIR();
  //gyroread();
  if (walking){
    walkFF();
  } else if (boppingTime){
    bop();
  } 
  if (displayDistance){
    distanceMillis = millis();
    if (distanceMillis % 1000 == 0){
      distance = getDistance();
    }  
  }
    

  //LoRa
  if (LoRaType == 1){
    switch (LoRaValue)
    {
    case 0:
      GoTo(sitpos);
      break;
    case 1: 
      GoTo(standpos);
      break;
    default:
      break;
    }
    LoRaType = 0;
  } else 
  if (LoRaType == 2){
    setServo(*LoRaArray[0], *LoRaArray[1]);
    LoRaType = 0;
  } else {}

  if (Serial.available())
  {
    char c = Serial.read();
    if (idx == 10)
    {                 //Wenn 10 Zeichen in command sind, wird der buffer gelöscht
      idx = 0;
      for (int i = 0; i < 10; i++)
      {
        command[i] = '\0';
      }
      Serial.println("Buffer overflow, reset");
    }
    else if (c == '\n') //Enter
    { 
      setServo(FLB, atoi(command));
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

void checkIR(){
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    switch (results.value){
      case FBVOLUP:
        Serial.printf("FLS = %d\n",cFLS);
        Serial.printf("FLT = %d\n",cFLT);
        Serial.printf("FLB = %d\n",cFLB);
        Serial.printf("BRS = %d\n",cBRS);
        Serial.printf("BRT = %d\n",cBRT);
        Serial.printf("BRB = %d\n",cBRB);
        Serial.printf("FRS = %d\n",cFRS);
        Serial.printf("FRT = %d\n",cFRT);
        Serial.printf("FRB = %d\n",cFRB);
        Serial.printf("BLS = %d\n",cBLS);
        Serial.printf("BLT = %d\n",cBLT);
        Serial.printf("BLB = %d\n",cBLB);
        break;
      case FB1:
        if(controlmode == 0){
          rotateLL();
          } else if(controlmode == 1){
            selectedServo = FLS;
            printf("Selected Servo: %d\n", selectedServo);
          }
        break;
      case FB2:
        if(controlmode == 0){
          stand();
        } else if(controlmode == 1){
          selectedServo = FLT;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB3:
        if(controlmode == 0){
          rotateRR();
        } else if (controlmode == 1){
          selectedServo = FLB;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB4:
        if(controlmode == 0){
          walk();
        } else if (controlmode == 1){
          selectedServo = BRS;
          printf("Selected Servo: %d\n", selectedServo);
        }  
        break;
      case FB5:
        if(controlmode == 0){
          GoTo(sitpos);
          sitting = true;
          standing = false;
        } else if (controlmode == 1){
          selectedServo = BRT;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB6:
      if(controlmode == 0){
        walking = !walking;
      } else if (controlmode ==1){
          selectedServo = BRB;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB7:
        if (controlmode == 0){
          //Rückwärts laufen
          moveLeg(FLt,1,0,3);
          delay(100);
          moveLeg(BRt,0,0,2);
          delay(100);
          moveLeg(BRt,-6,0,2);
          delay(100);
          moveLeg(BRt,-6,0,1);
          moveLeg(FLt,-6,0,1);
          delay(100);
          moveLeg(FLt,-6,0,0);
          
          standneutral();
  
          moveLeg(FRt,1,0,3);
          delay(100);
          moveLeg(BLt,0,0,2);
          delay(100);
          moveLeg(BLt,-6,0,2);
          delay(100);
          moveLeg(BLt,-6,0,1);
          moveLeg(FRt,-6,0,2);
          delay(100);
          moveLeg(FRt,-6,0,0);
          delay(100);
          standneutral();
        } else if (controlmode == 1){
          selectedServo = FRS;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB8:
      if (controlmode == 0){
        //Dieser Bereich Zum Testen einzelner Bewegungen
        walking = !walking;

        standneutral();




        // moveLeg(BRt,-1,0,3);
        // delay(100);
        // moveLeg(FLt,0,0,4);
        // delay(100);
        // moveLeg(FLt,6,0,4);
        // delay(100);
        // moveLeg(FLt,6,0,1);
        // moveLeg(BRt,6,0,1);
        // delay(100);
        // moveLeg(BRt,6,0,0);
        // standneutral();
        // moveLeg(BLt,-1,0,3);
        // delay(100);
        // moveLeg(FRt,0,0,4);
        // delay(100);
        // moveLeg(FRt,6,0,4);
        // delay(100);
        // moveLeg(FRt,6,0,1);
        // moveLeg(BLt,6,0,1);
        // delay(100);
        // moveLeg(BLt,6,0,0);
        // delay(100);
        // standneutral();
      } else
        if (controlmode == 1){
          selectedServo = FRT;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB9:
        if (controlmode == 0){
          setStandingPose();
        } else
        if (controlmode == 1){
          selectedServo = FRB;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FB0:
        if (controlmode == 1){
          selectedServo = BLS;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FBEQ:
        if (controlmode == 0){
          displayDistance = !displayDistance;
        }
        if (controlmode == 1){
          selectedServo = BLT;
          printf("Selected Servo: %d\n", selectedServo);
        }
        break;
      case FBREPT:
        if (controlmode == 0){
          LoRa_sendMessage("69");
        } else
        if (controlmode == 1){
          selectedServo = BLB;
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
        break;
      case FBRIGHT:
        sidestepRR();
        break;
      case FBLEFT:
        sidestepLL();
        break;
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


void LoRa_rxMode(){
  Serial.println("LoRa_rxMode");
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}
 
void LoRa_txMode(){
  Serial.println("LoRa_txMode");
  LoRa.idle();  
  Serial.println("idle done");          // set standby mode;
  LoRa.disableInvertIQ();              // normal mode
  Serial.println("disableInvertIQ done");
}
 
void LoRa_sendMessage(String message) {
  LoRa_txMode();                       // set tx mode
  Serial.println("Begin message");
  LoRa.beginPacket();                   // start packet
  Serial.println("Begin packet done");
  LoRa.print(message);                  // add payload
  Serial.println("Print done");
  LoRa.endPacket(false);                 // finish packet and send it
  Serial.println("End packet done");
}

void onReceive(int packetSize) {
  String message = "";
 
  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  Serial.print("Node Receive: ");
  Serial.println(message);
  if (sizeof(message) == 1){
    LoRaValue = message.toInt();
    LoRaType = 1;
  } else if (sizeof(message) == 2){
    int* LoRaArray = stringToIntArray(message);
    switch (LoRaArray[0]){
      case LoFLS:
        LoRaArray[0] = FLS;
        break;
      case LoFLT:
        LoRaArray[0] = FLT;
        break;
      case LoFLB:
        LoRaArray[0] = FLB;
        break;
      case LoFRS:
        LoRaArray[0] = FRS;
        break;
      case LoFRT:
        LoRaArray[0] = FRT;
        break;
      case LoFRB:
        LoRaArray[0] = FRB;
        break;
      case LoBRS:
        LoRaArray[0] = BRS;
        break;
      case LoBRT:
        LoRaArray[0] = BRT;
        break;
      case LoBRB:
        LoRaArray[0] = BRB;
        break;
      case LoBLS:
        LoRaArray[0] = BLS;
        break;
      case LoBLT:
        LoRaArray[0] = BLT;
        break;
      case LoBLB:
        LoRaArray[0] = BLB;
        break;
    }
    LoRaType = 2;
  } else {
    LoRaType = 0;
  }
}
 
void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}

int* stringToIntArray(String str)
{
  // Convert the string back to an integer array
  int startIndex = 0;
  int endIndex = str.indexOf(',');
  int* intArray = new int[2];
  int arrayIndex = 0;
 
  while (endIndex != -1) {
    intArray[arrayIndex++] = str.substring(startIndex, endIndex).toInt();
    startIndex = endIndex + 2; // Move past the comma and space
    endIndex = str.indexOf(',', startIndex);
  }
  intArray[arrayIndex] = str.substring(startIndex).toInt(); // Add the last number
 
  return intArray;
}

float getDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  float distanceFnct = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distanceFnct);
  if (distanceFnct < 10){
    distanceFlag = true;
  } else {
    distanceFlag = false;
  }
  return distanceFnct;
}
