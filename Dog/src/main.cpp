#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

#define SERVOMIN 125
#define SERVOMAX 600
Adafruit_PWMServoDriver servoDriver_module = Adafruit_PWMServoDriver(0x40);

int angleToPulse(int ang);

//void home();

void setup() {
  Serial.begin(9600);
  servoDriver_module.begin();
  servoDriver_module.setPWMFreq(50);    //Arbeitsfrequenz
}

void loop() {

  servoDriver_module.setPWM(0, 0, angleToPulse(100));
  delay(1000000);
}

int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  int pulse = map(ang,0, 270, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     Serial.print("Angle: ");Serial.print(ang);
     Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
  }

/*void home() {
      //(Port, steigende Flanke, sinkende Flanke)
      servoDriver_module.setPWM(0, 0, angleToPulse(30));   //VL Lower     Neutral 95
      servoDriver_module.setPWM(1, 0, angleToPulse(270/2));   //VL Upper  Neutral 95
      servoDriver_module.setPWM(2, 0, angleToPulse(0));   //VL Side       Neutral 100
      servoDriver_module.setPWM(4, 0, 0);   //HLL
      servoDriver_module.setPWM(5, 0, 0);   //HLU
      servoDriver_module.setPWM(6, 0, 0);   //HLS
      servoDriver_module.setPWM(8, 0, 0);   //HRL
      servoDriver_module.setPWM(9, 0, 0);   //HRU
      servoDriver_module.setPWM(10, 0, 0);  //HRS
      servoDriver_module.setPWM(12, 0, 0);  //VRL
      servoDriver_module.setPWM(16, 0, 200);  //VRU
      servoDriver_module.setPWM(15, 0, 200);  //VRS
      delay(10000);
}

*/