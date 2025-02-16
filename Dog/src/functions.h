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

int angleToPulse(int ang);
void home();                     // Ausgansposition (Entspricht sit)
void stand();                    // Fährt in Stehposition
void GoTo(const int targetPositions[12]); // Nimmt einen String mit 12 Positionen und fährt
void sidestepR();
void rotateL();
void walk();
void walkback();
void correctAll();                                                     // Hebt die Beine kurz an um unebenheiten auszugleichen (Alpha)
void setServo(int motor, int angle);                 // Bewegt einen Servo
void setServoS(int GoUp);                                              // Bewegt alle Seitlichen Servos
void setServoT(int GoUp);                                              // Bewegt alle Top Servos
void setServoB(int GoUp);                                              // Bewegt alle Bottom Servos
void setServoTB(int GoUp);                                             // Bewegt Top und Bottom Servos
void setServoSlow(int motor, int angle, int stepsize); // Bewegt einen Servo langsam
void moveServo(int selectedServo, int updown);                         // Bewegt einen einzelnen Servo in 5er schritten //updown: 1 = up, -1 = down     
int getArrayIndex(int pin);                                        // Erstellt die Strings für die Positionen mithilfe der Variablen

extern int cSFL, cTFL, cBFL, cSBR, cTBR, cBBR, cSFR, cTFR, cBFR, cSBL, cTBL, cBBL;
extern const int nSFL, nTFL, nBFL, nSBR, nTBR, nBBR, nSFR, nTFR, nBFR, nSBL, nTBL, nBBL;
extern const int sSFL, sTFL, sBFL, sSBR, sTBR, sBBR, sSFR, sTFR, sBFR, sSBL, sTBL, sBBL;
extern int* cPositions[13];
extern const int nPositions[13];
extern const int sPositions[13];
extern uint8_t remoteMac[6];
extern bool walking;
extern bool sitting;
extern bool standing;
extern int selectedServo;
extern const int sitpos[12];
extern const int standpos[12];
extern const int slideright[12];
extern int angleToPulse(int ang);

#endif