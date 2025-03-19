#ifndef FUNCTIONS_H
#define FUNCTIONS_H

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

extern int angleToPulse(int ang);
void home();                     // Startup Position
void stand();                    // Standing position
void GoTo(const int targetPositions[12]); // Takes an array of 12 ints and moves the servos to those positions
void setServo(int motor, int angle);                                   // Moves a single servo to a certain angle
void setServoS(int GoUp);                                              // Moves all side Servos
void setServoT(int GoUp);                                              // Moves all Top Servos
void setServoB(int GoUp);                                              // Moves all Bottom Servos
void setServoTB(int GoUp);                                             // Moves all Top and Bottom Servos (Bottom moves double the value)
void setServoSlow(int motor, int angle, int stepsize);                 // Moves a single servo to a certain angle with a certain stepsize
void moveServo(int selectedServo, int updown);                         // Moves a selected Servo in steps of 5 //updown: 1 = up, -1 = down
void computeIK(int legID,float x, float y, float z, float &theta1, float &theta2, float &theta3);
void moveLegGeneralFunc(int legID, float x, float y, float z, int stepsize = 0);
void moveLeg(int legID, float x, float y, float z, int stepsize = 0);
void sidestepR();
void rotateL();
void walk();
void walkback();
void setStandingPose();
void standneutral();
void walkFF();
void sidestepRR();
void sidestepLL();
void rotateRR();
void rotateLL();
void bop(); //bops up and down
extern void waitforButton();
extern void checkIR();

extern int cFLS, cFLT, cFLB, cBRS, cBRT, cBRB, cFRS, cFRT, cFRB, cBLS, cBLT, cBLB;
extern const int nFLS, nFLT, nFLB, nBRS, nBRT, nBRB, nFRS, nFRT, nFRB, nBLS, nBLT, nBLB;
extern const int sFLS, sFLT, sFLB, sBRS, sBRT, sBRB, sFRS, sFRT, sFRB, sBLS, sBLT, sBLB;
extern const int servoOffsets[13];
extern const int nPositions[13];
extern const int sPositions[13];
extern const int sitpos[12];
extern const int standpos[12];
extern const int slideright[12];
extern int* cPositions[13];
extern int nextPos[12];
extern bool walking;
extern bool sitting;
extern bool standing;
extern int selectedServo;
extern bool singleLeg;
extern bool boppingTime;
#endif