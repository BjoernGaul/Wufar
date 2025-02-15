#include "functions.h"

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

extern Adafruit_PWMServoDriver servoDriver_module;
extern int cSFL, cTFL, cBFL, cSBR, cTBR, cBBR, cSFR, cTFR, cBFR, cSBL, cTBL, cBBL;
extern int sSFL, sTFL, sBFL, sSBR, sTBR, sBBR, sSFR, sTFR, sBFR, sSBL, sTBL, sBBL;
extern uint8_t remoteMac[6];
extern bool walking;
extern bool sitting;
extern bool standing;
extern int selectedServo;
extern String standpos;
extern String sitpos;
extern String slideright;
extern void checkIR();
extern void waitforButton();
extern int angleToPulse(int ang);


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
    int stepSize = 1;
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
  }
  
  void setServoSlow(int motor, int &currmotor, int angle, int stepsize){
    if (currmotor < angle){
      for (int i = currmotor; i <= angle; i+=stepsize){
        setServo(motor, currmotor, i);
        delay(10);
      }
    }else if (currmotor > angle){
      for (int i = currmotor; i >= angle; i-=stepsize){
        setServo(motor, currmotor, i);
        delay(10);
      }
    }else if (currmotor - angle < stepsize){
      setServo(motor, currmotor, angle);
    }
  }
  
  void setServoS(int GoUp){
    setServo(SFL, cSFL, (cSFL+GoUp));
    setServo(SBR, cSBR, (cSBR-GoUp));
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
    setServo(BBR, cBBR, cBBR+GoUp);
    setServo(BFR, cBFR, cBFR-GoUp);
    setServo(BBL, cBBL, cBBL-GoUp);
  }
  
  void setServoTB(int GoUp){
    setServoT(GoUp);
    setServoB(2*GoUp);
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
        setServo(TBR, cTBR, cTBR-step);
        break;
      case BBR:
        setServo(BBR, cBBR, cBBR+step);
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
        setServo(TBL, cTBL, cTBL+step);
        break;
      case BBL:
        setServo(BBL, cBBL, cBBL-step);
        break;
      default:
        Serial.println("What?");
        break;
    }
  }
  void sidestepR(){
    if (standing){
      //Linke Seite absenken
      setServo(BFL, cBFL, cBFL-20);
      setServo(BBL, cBBL, cBBL+20);
      delay(100);
      waitforButton();
      //Rechtes Bein Vorne bewegen
      setServoSlow(BFR, cBFR, cBFR+20, 3);
      setServoSlow(TFR, cTFR, cTFR-10, 3);
      setServoSlow(SFR, cSFR, cSFR-20, 3);
      delay(50);
      setServoSlow(TFR, cTFR, cTFR+15, 3);
      setServoSlow(BFR, cBFR, cBFR-30, 3);
      //Rechtes Bein Hinten bewegen
      setServoSlow(BBR, cBBR, cBBR-20, 3);
      setServoSlow(TBR, cTBR, cTBR+10, 3);
      setServoSlow(SBR, cSBR, cSBR+20, 3);
      delay(50);
      setServoSlow(TBR, cTBR, cTBR-15, 3);
      setServoSlow(BBR, cBBR, cBBR+30, 3);
      waitforButton();
      //Alle Beine Seitlich bewegen
      GoTo(slideright.c_str());
  
    }
  }
  
  void rotateL(){
    setServo(BFR, cBFR, cBFR+50);
    delay(20);
    setServo(SFR, cSFR, cSFR+10);
    delay(20);
    setServo(BFR, cBFR, cBFR-50);
    delay(200);
    setServo(BBL, cBBL, cBBL+50);
    delay(20);
    setServo(SBL, cSBL, cSBL+10);
    delay(20);
    setServo(BBL, cBBL, cBBL-50);
    delay(200);
    
    setServo(BFL, cBFL, cBFL-50);
    delay(20);
    setServo(SFL, cSFL, cSFL+10);
    delay(20);
    setServo(BFL, cBFL, cBFL+50);
    delay(200);
    setServo(BBR, cBBR, cBBR-50);
    delay(20);
    setServo(SBR, cSBR, cSBR+10);
    delay(20);
    setServo(BBR, cBBR, cBBR+50);
    delay(200);
    setServo(SFR, cSFR, cSFR-10);
    setServo(SBL, cSBL, cSBL-10);
    setServo(SFL, cSFL, cSFL-10);
    setServo(SBR, cSBR, cSBR-10);
  }
  
  void walk(){
    setServo(BFR, cBFR, cBFR+30);
    setServo(TFR, cTFR, sTFR+15);
    delay(100);
    setServo(BFR, cBFR, sBFR-10);
    // setServo(TFR, cTFR, sTFR);
    setServoT(-5);
    delay(100);
  
    setServo(BBL, cBBL, cBBL+30);
    setServo(TBL, cTBL, sTBL-5);
    delay(100);
    setServo(BBL, cBBL, sBBL-10);
    // setServo(TBL, cTBL, sTBL);
    setServoT(-5);
    delay(100);
      
    setServo(BFL, cBFL, cBFL-35);
    setServo(TFL, cTFL, sTFL-5);
    delay(100);
    setServo(BFL, cBFL, sBFL-10);
    // setServo(TFL, cTFL, sTFL);
    setServoT(-5);
    delay(100);
  
    setServo(BBR, cBBR, cBBR-25);
    setServo(TBR, cTBR, sTBR);
    delay(100);
    setServo(BBR, cBBR, sBBR-10);
    // setServo(TBR, cTBR, sTBR);
    // delay(200);
  
    // setServo(TFR, cTFR, sTFR+10);
    // setServo(TFL, cTFL, sTFL+10);
    // setServo(TBR, cTBR, sTBR+10);
    // setServo(TBL, cTBL, sTBL+10);
  
  
    GoTo(standpos.c_str());
  
  }
  
  void walkback(){
    setServo(BFR, cBFR, cBFR+20);
    setServo(TFR, cTFR, sTFR-20);
    delay(100);
    setServo(BFR, cBFR, sBFR-5);
    // setServo(TFR, cTFR, sTFR);
    setServoT(+5);
    delay(100);
  
    setServo(BBL, cBBL, cBBL+20);
    setServo(TBL, cTBL, sTBL-10);
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
  
    setServo(BBR, cBBR, cBBR-25);
    setServo(TBR, cTBR, sTBR+5);
    delay(100);
    setServo(BBR, cBBR, sBBR+10);
    // setServo(TBR, cTBR, sTBR);
    // delay(200);
  
    // setServo(TFR, cTFR, sTFR+10);
    // setServo(TFL, cTFL, sTFL+10);
    // setServo(TBR, cTBR, sTBR+10);
    // setServo(TBL, cTBL, sTBL+10);
  
  
    delay(100);
    GoTo(standpos.c_str());
  
  }
  
  void correctAll(){
  GoTo(standpos.c_str());
  delay(100);
  setServoTB(-10);
  delay(100);
  setServoTB(10);
  }
  