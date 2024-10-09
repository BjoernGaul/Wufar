#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

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


Adafruit_PWMServoDriver servoDriver_module = Adafruit_PWMServoDriver(0x40);

int angleToPulse(int ang);
//neutral positions
const int nsvl = 95;
const int nshr = 95;
const int nsvr = 95;
const int nshl = 95;
const int ntvl = 95;
const int nthr = 105;
const int ntvr = 105;
const int nthl = 65;
const int nbvl = 95;
const int nbhr = 95;
const int nbvr = 95;
const int nbhl = 95;
//current positions
int csvl;
int cshr;
int csvr;
int cshl;
int ctvl;
int cthr;
int ctvr;
int cthl;
int cbvl;
int cbhr;
int cbvr;
int cbhl;

void home();
void up();
void down();
void squat();
void setServo(int motor, int currmotor, int angle);

void setup() {
  Serial.begin(9600);
  servoDriver_module.begin();
  servoDriver_module.setPWMFreq(50);    //Arbeitsfrequenz
  Serial.printf("Begin Wait\n");
  delay(2000);
  Serial.printf("Finished Wait\n");
  home();
}

void loop() {
  squat();
}

int angleToPulse(int ang){
  int pulse = map(ang,0, 270, SERVOMIN,SERVOMAX);  
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
  delay(1000);
  servoDriver_module.setPWM(TVL, 0, angleToPulse(95));
  Serial.printf("Setze Top VL\n");
  servoDriver_module.setPWM(THR, 0, angleToPulse(105));
  Serial.printf("Setze Top HR\n");
  servoDriver_module.setPWM(TVR, 0, angleToPulse(105));
  Serial.printf("Setze Top VR\n");
  servoDriver_module.setPWM(THL, 0, angleToPulse(65));
  Serial.printf("Setze Top HL\n");
  delay(1000);
  servoDriver_module.setPWM(BVL, 0, angleToPulse(95));
  Serial.printf("Setze Bottom VL\n");
  servoDriver_module.setPWM(BHR, 0, angleToPulse(95));
  Serial.printf("Setze Bottom HR\n");
  servoDriver_module.setPWM(BVR, 0, angleToPulse(95));
  Serial.printf("Setze Bottom VR\n");
  servoDriver_module.setPWM(BHL, 0, angleToPulse(95));
  Serial.printf("Setze Bottom HL\n");
  delay(1000);
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
    /*servoDriver_module.setPWM(BVL, 0, angleToPulse(nbvl+(2*i)));
    servoDriver_module.setPWM(BHR, 0, angleToPulse(nbhr-(2*i)));
    servoDriver_module.setPWM(BVR, 0, angleToPulse(nbvr-(2*i)));
    servoDriver_module.setPWM(BHL, 0, angleToPulse(nbhl+(2*i)));

    servoDriver_module.setPWM(TVL, 0, angleToPulse(ntvl-i));
    servoDriver_module.setPWM(THR, 0, angleToPulse(nthr+i));
    servoDriver_module.setPWM(TVR, 0, angleToPulse(ntvr+i));
    servoDriver_module.setPWM(THL, 0, angleToPulse(nthl-i));*/
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

void setServo(int motor, int currmotor, int angle){
  servoDriver_module.setPWM(motor, 0, angleToPulse(angle));
  currmotor = angleToPulse(angle);
}