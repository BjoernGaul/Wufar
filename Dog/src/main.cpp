#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <Wire.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <LoRa.h>
#include "functions.h"

//* Pin of Servos
#define FLS 0
#define FLT 1
#define FLB 2
#define BRS 3
#define BRT 4
#define BRB 5
#define FRS 6
#define FRT 7
#define FRB 8
#define BLS 9
#define BLT 10
#define BLB 11
//* Leg IDs
#define FLt 0
#define BRt 1
#define FRt 2
#define BLt 3

//* Hex-Adresses for IR Remote
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

//* Servo Parameters
#define L1 8.4       // Upper Leg Length (cm)
#define L2 12.2      // Lower Leg Length (cm)
#define Z_STAND 12.5 // Hip Height while standing (cm)
#define X_OFFSET 2.0 // Feet slightly in front of the hip
#define Y_OFFSET 1.5 // Distance foot to hip
#define H_MAX -6.0
#define H_MIN 6.0

//* LoRa
#define DIO0 D10
#define NSS D5
#define frequency 433E6
int LoRaType = 0;
int LoRaValue = 0;
int LoRaServo = 0;  // Servo ID when the Controller is setting a single servo
int LoRaOffset = 0; // Offset of the angle sent by the Controller
int LoRaAngle = 0;  // Angle of the Servo sent by the Controller (incl offset)
int LoRaGetAngle;
int LoRaSendAngle;
volatile bool gotMsg = false;
String TempMsg = "";
//* LoRaArray[0] values
const uint8_t LoSit = 1;
const uint8_t LoHump = 3;
const uint8_t LoBop = 9;
const uint8_t LoIsSitting = 69;
const uint8_t joyRight = 100;
const uint8_t joyLeft = 101;
const uint8_t LoDistance = 200;
const uint8_t LoGetPosLegs = 250;
//* LoRaArray[1] values
const uint8_t LoStand = 10;
const uint8_t LoRotL = 11;
const uint8_t LoRotR = 12;
const uint8_t LoWalkB = 13;
const uint8_t LoWalkF = 14;
const uint8_t LoCrabL = 11;
const uint8_t LoCrabR = 12;
const uint8_t LoUp = 13;
const uint8_t LoDown = 14;
//* LoRaArray[0] values for single Servos
const uint8_t LoFLS = 10;
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
const uint8_t reset = 255;

//*IR Remote
const uint16_t RECV_PIN = 4;
int controlmode = 0;
int selectedServo;
Adafruit_PWMServoDriver servoDriver_module = Adafruit_PWMServoDriver(0x40);
IRrecv irrecv(RECV_PIN);
decode_results results;
Adafruit_MPU6050 mpu;

//* Ultrasonic
const int trigPin = D11;
const int echoPin = D3;
float duration, distance;

//* current positions
int cFLS = 0, cFLT = 0, cFLB = 0, cBRS = 0, cBRT = 0, cBRB = 0, cFRS = 0, cFRT = 0, cFRB = 0, cBLS = 0, cBLT = 0, cBLB = 0;
int *cPositions[12] = {&cFLS, &cFLT, &cFLB, &cBRS, &cBRT, &cBRB, &cFRS, &cFRT, &cFRB, &cBLS, &cBLT, &cBLB};
int nextPos[12] = {cFLS, cFLT, cFLB, cBRS, cBRT, cBRB, cFRS, cFRT, cFRB, cBLS, cBLT, cBLB};

//* neutral positions
const int servoOffsets[12] = {97, 160, 5, 135, 150, 35, 94, 165, 185, 28, 55, 162}; // Offset
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
const int nPositions[12] = {nFLS, nFLT, nFLB, nBRS, nBRT, nBRB, nFRS, nFRT, nFRB, nBLS, nBLT, nBLB};

//* Standing Positions
const int sFLS = nFLS;
const int sFLT = nFLT - 30;
const int sFLB = nFLB + 60;
const int sBRS = nBRS;
const int sBRT = nBRT - 30;
const int sBRB = nBRB + 60;
const int sFRS = nFRS;
const int sFRT = nFRT + 30;
const int sFRB = nFRB - 60;
const int sBLS = nBLS;
const int sBLT = nBLT + 30;
const int sBLB = nBLB - 60;
const int sPositions[12] = {sFLS, sFLT, sFLB, sBRS, sBRT, sBRB, sFRS, sFRT, sFRB, sBLS, sBLT, sBLB};

//* Gyroscope offset values
float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

//* Other variables
int task = 0; // provides the task for the robot
void waitforButton();
bool paused = false;
bool singleLeg = true;
bool sitting = true;
float height = 0;
bool heightChanged = false;

//* Position Arrays
const int sitpos[12] = {nFLS, nFLT, nFLB, nBRS, nBRT, nBRB, nFRS, nFRT, nFRB, nBLS, nBLT, nBLB};
const int standpos[12] = {sFLS, sFLT, sFLB, sBRS, sBRT, sBRB, sFRS, sFRT, sFRB, sBLS, sBLT, sBLB};

//* serial monitor input
char command[10];
int idx = 0;

void checkIR();
void gyrosetup();
void gyroread();
void calibrateGyro();

//* LoRa functions
void LoRa_sendMessage(String message);
void onReceive(String message);
int *stringToIntArray(String str);
void LoRaHandleMsg(int packetSize);

//* Ultrasonic functions
float getDistance();
int distanceMillisC;
int distanceMillisL;
bool displayDistance = false;
bool distanceFlag = false;


void setup()
{
  Serial.begin(9600);
  Serial.println("Setup Start");
  //* Servo Setup
  servoDriver_module.begin();
  servoDriver_module.setPWMFreq(50); // operation frequency of the servos
  GoTo(sitpos);
  //* IR Setup
  irrecv.enableIRIn(); // Infrared Remote
  Serial.println("IR enabled");
  //* LoRa Setup
  delay(500);
  Serial.println("LoRa Start");
  LoRa.setPins(NSS, -1, DIO0);
  if (!LoRa.begin(frequency))
  {
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ;
  }
  LoRa.onReceive(LoRaHandleMsg);
  LoRa.receive();
  //*  Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // gyrosetup();
}

void loop()
{
  //* Handle Communication messages
  if (gotMsg)
  {
    gotMsg = false;
    onReceive(TempMsg);
  }
  checkIR();
  // gyroread();
  //* Checks height before any other task
  if (heightChanged && (task > 0) && (task < 10))
  { // If the height is changed, it will first reset its height
    height = 0;
    heightChanged = false;
    setStandingPose();
  }

  //* State Machine
  switch (task)
  {
  case 0:
    break;
  case 1:
    GoTo(sitpos);
    sitting = true;
    task = 0;
    break;
  case 2:
    setStandingPose();
    sitting = false;
    task = 0;
    break;
  case 3:
    if (!sitting)
    {
      walkFF();
      if (distanceFlag)
      {
        task = 2;
      }
    }
    break;
  case 4:
    if (!sitting)
    {
      walkBB();
    }
    break;
  case 5:
    if (!sitting)
    {
      sidestepRR();
    }
    break;
  case 6:
    if (!sitting)
    {
      sidestepLL();
    }
    break;
  case 7:
    if (!sitting)
    {
      rotateRR();
    }
    break;
  case 8:
    if (!sitting)
    {
      rotateLL();
    }
    break;
  case 9:
    if (!sitting)
    {
      bop();
    }
    break;
  case 10:
    setServo(LoRaServo, LoRaAngle);
    task = 0;
    break;
  case 11:
    if (!sitting)
    {
      hump();
    }
    break;
  case 12:
    if (!sitting)
    {
      changeHeight(0.1);
    }
    break;
  case 13:
    if (!sitting)
    {
      changeHeight(-0.1);
    }
    break;
  default:
    setStandingPose();
    break;
  }
  //* Ultrasonic Module
  if (displayDistance)
  {
    distanceMillisC = millis();
    if (distanceMillisC - distanceMillisL > 1000)
    {
      distance = getDistance();
      String message = String(200) + ", " + String(distance);
      LoRa_sendMessage(message);
      distanceMillisL = distanceMillisC;
      if (task != 3 && task != 4)
      {
        displayDistance = false;
      }
    }
  }
  //* Serial Monitor
  if (Serial.available())
  {
    char c = Serial.read();
    if (idx == 10)
    { // Delete Buffer if full
      idx = 0;
      for (int i = 0; i < 10; i++)
      {
        command[i] = '\0';
      }
      Serial.println("Buffer overflow, reset");
    }
    else if (c == '\n') // Enter
    {
      Serial.printf("Command: %s\n", command);
      if (command[0] == 'u')
      {
        displayDistance = true;
      }
      else if (command[0] == 'd')
      {
        displayDistance = false;
      }
      // setServo(BLT, atoi(command));
      // String message = String(200) + ", " + String(command);
      // LoRa_sendMessage(message);
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

void waitforButton()
{
  irrecv.resume();
  paused = true;
  delay(1000);
  while (paused)
    (checkIR());
}

void checkIR()
{// For testing with IR Remote
  if (irrecv.decode(&results))
  {
    Serial.println(results.value, HEX);
    switch (results.value)
    {
    case FBPOWER:
      if (sitting)
      {
        task = 2;
      }
      else
      {
        task = 1;
      }
      LoRa_sendMessage(String(LoIsSitting) + ", " + String(sitting));
      break;
    case FBVOLUP:
      if (controlmode == 0)
      { // Walking Forward
        task = 3;
        // displayDistance = true;
      }
      if (controlmode == 1)
      {
        Serial.printf("FLS = %d\n", cFLS);
        Serial.printf("FLT = %d\n", cFLT);
        Serial.printf("FLB = %d\n", cFLB);
        Serial.printf("BRS = %d\n", cBRS);
        Serial.printf("BRT = %d\n", cBRT);
        Serial.printf("BRB = %d\n", cBRB);
        Serial.printf("FRS = %d\n", cFRS);
        Serial.printf("FRT = %d\n", cFRT);
        Serial.printf("FRB = %d\n", cFRB);
        Serial.printf("BLS = %d\n", cBLS);
        Serial.printf("BLT = %d\n", cBLT);
        Serial.printf("BLB = %d\n", cBLB);
      }
      break;
    case FBVOLDOWN:
      if (controlmode == 0)
      { // Walking Backward
        task = 4;
        // displayDistance = true;
      }
      break;
    case FB1:
      if (controlmode == 0)
      { // Höher
        task = 12;

        heightChanged = true;
      }
      else if (controlmode == 1)
      {
        selectedServo = FLS;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FB2:
      if (controlmode == 0)
      { // Tiefer
        task = 13;

        heightChanged = true;
      }
      else if (controlmode == 1)
      {
        selectedServo = FLT;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FB3:
      if (controlmode == 0)
      {
        task = 11;
      }
      else if (controlmode == 1)
      {
        selectedServo = FLB;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FB4:
      if (controlmode == 0)
      {
      }
      else if (controlmode == 1)
      {
        selectedServo = BRS;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FB5:
      if (controlmode == 0)
      {
        task = 0;
      }
      else if (controlmode == 1)
      {
        selectedServo = BRT;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FB6:
      if (controlmode == 0)
      {
      }
      else if (controlmode == 1)
      {
        selectedServo = BRB;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FB7:
      if (controlmode == 0)
      {
      }
      else if (controlmode == 1)
      {
        selectedServo = FRS;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FB8:
      if (controlmode == 0)
      { // Bopping
        task = 9;
      }
      else if (controlmode == 1)
      {
        selectedServo = FRT;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FB9:
      if (controlmode == 0)
      {
        paused = false;
      }
      else if (controlmode == 1)
      {
        selectedServo = FRB;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FB0:
      if (controlmode == 0)
      {
        Serial.printf("FLS = %d\n", cFLS);
        Serial.printf("FLT = %d\n", cFLT);
        Serial.printf("FLB = %d\n", cFLB);
        Serial.printf("BRS = %d\n", cBRS);
        Serial.printf("BRT = %d\n", cBRT);
        Serial.printf("BRB = %d\n", cBRB);
        Serial.printf("FRS = %d\n", cFRS);
        Serial.printf("FRT = %d\n", cFRT);
        Serial.printf("FRB = %d\n", cFRB);
        Serial.printf("BLS = %d\n", cBLS);
        Serial.printf("BLT = %d\n", cBLT);
        Serial.printf("BLB = %d\n", cBLB);
      }
      else if (controlmode == 1)
      {
        selectedServo = BLS;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FBEQ:
      if (controlmode == 0)
      {
        displayDistance = !displayDistance;
      }
      if (controlmode == 1)
      {
        selectedServo = BLT;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FBREPT:
      if (controlmode == 0)
      {
        // LoRa_sendMessage("69");
      }
      else if (controlmode == 1)
      {
        selectedServo = BLB;
        printf("Selected Servo: %d\n", selectedServo);
      }
      break;
    case FBUP:
      if (controlmode == 0)
      {
        task = 7;
        Serial.println("FBUP");
      }
      else if (controlmode == 1)
      {
        moveServo(selectedServo, 1);
      }
      break;
    case FBDOWN:
      if (controlmode == 0)
      {
        task = 8;
        Serial.println("FBDOWN");
      }
      else if (controlmode == 1)
      {
        moveServo(selectedServo, -1);
      }
      break;
    case FBFUNC:
      if (controlmode == 0)
      {
        controlmode = 1;
        Serial.println("Controlmode: 1");
      }
      else
      {
        controlmode = 0;
        Serial.println("Controlmode: 0");
      }
      break;
    case FBPLAY:
      task = 2;
      break;
    case FBRIGHT:
      task = 5;
      break;
    case FBLEFT:
      task = 6;
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

void LoRa_sendMessage(String message)
{
  Serial.println("Begin message");
  LoRa.beginPacket();
  LoRa.write(message.length());
  LoRa.print(message);
  Serial.println(message);
  LoRa.endPacket();
  LoRa.receive();
}

void onReceive(String message)
{
  int *LoRaArray = stringToIntArray(message);
  if (LoRaArray[0] <= 3 || LoRaArray[0] == LoGetPosLegs)
  {
    LoRaValue = LoRaArray[0];
    if (LoRaValue <= 2)
    {
      task = LoRaValue;
    }
    else if (LoRaValue == 3)
    {
      task = 11;
    }
    else if (LoRaValue == LoGetPosLegs)
    {
      delay(800);
      LoRa_sendMessage(String(LoFLS) + ", " + String(cFLS + servoOffsets[0]));
      delay(50);
      LoRa_sendMessage(String(LoFLT) + ", " + String(cFLT + servoOffsets[1]));
      delay(50);
      LoRa_sendMessage(String(LoFLB) + ", " + String(cFLB + servoOffsets[2]));
      delay(50);
      LoRa_sendMessage(String(LoBRS) + ", " + String(cBRS + servoOffsets[3]));
      delay(50);
      LoRa_sendMessage(String(LoBRT) + ", " + String(cBRT + servoOffsets[4]));
      delay(50);
      LoRa_sendMessage(String(LoBRB) + ", " + String(cBRB + servoOffsets[5]));
      delay(50);
      LoRa_sendMessage(String(LoFRS) + ", " + String(cFRS + servoOffsets[6]));
      delay(50);
      LoRa_sendMessage(String(LoFRT) + ", " + String(cFRT + servoOffsets[7]));
      delay(50);
      LoRa_sendMessage(String(LoFRB) + ", " + String(cFRB + servoOffsets[8]));
      delay(50);
      LoRa_sendMessage(String(LoBLS) + ", " + String(cBLS + servoOffsets[9]));
      delay(50);
      LoRa_sendMessage(String(LoBLT) + ", " + String(cBLT + servoOffsets[10]));
      delay(50);
      LoRa_sendMessage(String(LoBLB) + ", " + String(cBLB + servoOffsets[11]));
    }
  }
  else if (2 < LoRaArray[0] <= 50)
  {
    if (LoRaArray[0] == joyLeft)
    {
      switch (LoRaArray[1])
      {
      case LoStand:
        task = 2;
        break;
      case LoRotL:
        task = 8;
        break;
      case LoRotR:
        task = 7;
        break;
      case LoWalkB:
        task = 4;
        displayDistance = true;
        break;
      case LoWalkF:
        task = 3;
        displayDistance = true;
        break;
      default:
        task = 0;
        break;
      }
    }
    else if (LoRaArray[0] == joyRight)
    {
      switch (LoRaArray[1])
      {
      case LoStand:
        task = 0;
        break;
      case LoCrabL:
        task = 6;
        break;
      case LoCrabR:
        task = 5;
        break;
      case LoUp:
        task = 13;
        heightChanged = true;
        break;
      case LoDown:
        task = 12;
        heightChanged = true;
        break;
      default:
        task = 0;
        break;
      }
    }
    else if (10 <= LoRaArray[0] < 50)
    {
      switch (LoRaArray[0])
      {
      case LoFLS:
        LoRaServo = FLS;
        LoRaAngle = LoRaArray[1] - servoOffsets[0];
        break;
      case LoFLT:
        LoRaServo = FLT;
        LoRaAngle = LoRaArray[1] - servoOffsets[1];
        break;
      case LoFLB:
        LoRaServo = FLB;
        LoRaAngle = LoRaArray[1] - servoOffsets[2];
        break;
      case LoBRS:
        LoRaServo = BRS;
        LoRaAngle = LoRaArray[1] - servoOffsets[3];
        break;
      case LoBRT:
        LoRaServo = BRT;
        LoRaAngle = LoRaArray[1] - servoOffsets[4];
        break;
      case LoBRB:
        LoRaServo = BRB;
        LoRaAngle = LoRaArray[1] - servoOffsets[5];
        break;
      case LoFRS:
        LoRaServo = FRS;
        LoRaAngle = LoRaArray[1] - servoOffsets[6];
        break;
      case LoFRT:
        LoRaServo = FRT;
        LoRaAngle = LoRaArray[1] - servoOffsets[7];
        break;
      case LoFRB:
        LoRaServo = FRB;
        LoRaAngle = LoRaArray[1] - servoOffsets[8];
        break;
      case LoBLS:
        LoRaServo = BLS;
        LoRaAngle = LoRaArray[1] - servoOffsets[9];
        break;
      case LoBLT:
        LoRaServo = BLT;
        LoRaAngle = LoRaArray[1] - servoOffsets[10];
        break;
      case LoBLB:
        LoRaServo = BLB;
        LoRaAngle = LoRaArray[1] - servoOffsets[11];
        break;
      default:
        LoRaServo = -1;
        break;
      }
      task = 10;
    }
  }
  TempMsg = "";
}

int *stringToIntArray(String str)
{// Convert the string back to an integer array
  
  int startIndex = 0;
  int endIndex = str.indexOf(',');
  int *intArray = new int[2];
  int arrayIndex = 0;

  while (endIndex != -1)
  {
    intArray[arrayIndex++] = str.substring(startIndex, endIndex).toInt();
    startIndex = endIndex + 2; // Move past the comma and space
    endIndex = str.indexOf(',', startIndex);
  }
  intArray[arrayIndex] = str.substring(startIndex).toInt(); // Add the last number
  // printf("Array: %d, %d\n", intArray[0], intArray[1]);
  return intArray;
}

float getDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  float distanceFnct = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distanceFnct);
  if (distanceFnct < 10)
  {
    distanceFlag = true;
  }
  else
  {
    distanceFlag = false;
  }
  return distanceFnct;
}

void LoRaHandleMsg(int packetSize)
{
  gotMsg = true;
  if (packetSize == 0)
  {
    return;
  }
  byte incomingLength = LoRa.read();
  String message = "";
  while (LoRa.available())
  {
    message += (char)LoRa.read();
  }
  if (incomingLength != message.length())
  { // check length for error
    Serial.println("error: message length does not match length");
    return; // skip rest of function
  }
  TempMsg = message;
}