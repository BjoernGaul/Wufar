#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

#define SERVOMIN 125
#define SERVOMAX 600

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

Adafruit_PWMServoDriver servoDriver_module = Adafruit_PWMServoDriver(0x40);

int angleToPulse(int ang);

void home();
void up();
void down();

void setup() {
  Serial.begin(9600);
  servoDriver_module.begin();
  servoDriver_module.setPWMFreq(50);    //Arbeitsfrequenz
  Serial.printf("Begin Wait\n");
  delay(5000);
  Serial.printf("Finished Wait\n");
  home();
}

void loop() {
    up();
    delay(5000);
    down();
    delay(5000);
}

int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  int pulse = map(ang,0, 270, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     Serial.print("Angle: ");Serial.print(ang);
     Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
  }

void home() {
    servoDriver_module.setPWM(SVL, 0, angleToPulse(95));
    Serial.printf("Setze Seite VL\n");
    servoDriver_module.setPWM(SHR, 0, angleToPulse(95));
    Serial.printf("Setze Seite HR\n");
    servoDriver_module.setPWM(SVR, 0, angleToPulse(95));
    Serial.printf("Setze Seite VR\n");
    servoDriver_module.setPWM(SHL, 0, angleToPulse(95));
    Serial.printf("Setze Seite HL\n");
    delay(3000);
    servoDriver_module.setPWM(TVL, 0, angleToPulse(95));
    Serial.printf("Setze Top VL\n");
    servoDriver_module.setPWM(THR, 0, angleToPulse(105));
    Serial.printf("Setze Top HR\n");
    servoDriver_module.setPWM(TVR, 0, angleToPulse(105));
    Serial.printf("Setze Top VR\n");
    servoDriver_module.setPWM(THL, 0, angleToPulse(65));
    Serial.printf("Setze Top HL\n");
    delay(3000);
    servoDriver_module.setPWM(BVL, 0, angleToPulse(95));
    Serial.printf("Setze Bottom VL\n");
    servoDriver_module.setPWM(BHR, 0, angleToPulse(95));
    Serial.printf("Setze Bottom HR\n");
    servoDriver_module.setPWM(BVR, 0, angleToPulse(95));
    Serial.printf("Setze Bottom VR\n");
    servoDriver_module.setPWM(BHL, 0, angleToPulse(95));
    Serial.printf("Setze Bottom HL\n");
    return;
}

void up(){
  for (int i = 0; i<40; i+=5){
    servoDriver_module.setPWM(BVL, 0, angleToPulse(95+i));  //BVL
    Serial.printf("Setze Bottom VL\n");
    servoDriver_module.setPWM(BHR, 0, angleToPulse(95-i));  //BHR
    Serial.printf("Setze Bottom HR\n");
    servoDriver_module.setPWM(BVR, 0, angleToPulse(95-i));  //BVR
    Serial.printf("Setze Bottom VR\n");
    servoDriver_module.setPWM(BHL, 0, angleToPulse(95+i));  //BHL
    Serial.printf("Setze Bottom HL\n");
  }
}

void down(){
  for (int j = 0; j<40; j+=5){
    servoDriver_module.setPWM(BVL, 0, angleToPulse(130-j));  //BVL
    Serial.printf("Setze Bottom VL\n");
    servoDriver_module.setPWM(BHR, 0, angleToPulse(60+j));  //BHR
    Serial.printf("Setze Bottom HR\n");
    servoDriver_module.setPWM(BVR, 0, angleToPulse(60+j));  //BVR
    Serial.printf("Setze Bottom VR\n");
    servoDriver_module.setPWM(BHL, 0, angleToPulse(130-j));  //BHL
    Serial.printf("Setze Bottom HL\n");
  }
}