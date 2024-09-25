#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

#define SERVOMIN 500
#define SERVOMAX 2500
Adafruit_PWMServoDriver servoDriver_module = Adafruit_PWMServoDriver();

//function declarations
int angleToPulse(int ang);

void home();

void setup() {
  Serial.begin(9600);
  servoDriver_module.begin();
  servoDriver_module.setPWMFreq(50);    //Arbeitsfrequenz
  home();
}

void loop() {
  // put your main code here, to run repeatedly:
}

void home() {
      //(Port, steigende Flanke, sinkende Flanke)
      servoDriver_module.setPWM(0, 0, 420);   //VL Lower
      servoDriver_module.setPWM(1, 0, 360);   //VL Upper
      servoDriver_module.setPWM(2, 0, 350);   //VL Side
      servoDriver_module.setPWM(4, 0, 400);   //HLL
      servoDriver_module.setPWM(5, 0, 440);   //HLU
      servoDriver_module.setPWM(6, 0, 280);   //HLS
      servoDriver_module.setPWM(8, 0, 240);   //HRL
      servoDriver_module.setPWM(9, 0, 220);   //HRU
      servoDriver_module.setPWM(10, 0, 340);  //HRS
      servoDriver_module.setPWM(12, 0, 190);  //VRL
      servoDriver_module.setPWM(13, 0, 260);  //VRU
      servoDriver_module.setPWM(14, 0, 260);  //VRS
      delay(10000);
}

int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     Serial.print("Angle: ");Serial.print(ang);
     Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
  }