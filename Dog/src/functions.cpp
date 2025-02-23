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
/*extern int cSFL, cTFL, cBFL, cSBR, cTBR, cBBR, cSFR, cTFR, cBFR, cSBL, cTBL, cBBL;
extern const int nSFL, nTFL, nBFL, nSBR, nTBR, nBBR, nSFR, nTFR, nBFR, nSBL, nTBL, nBBL;
extern const int sSFL, sTFL, sBFL, sSBR, sTBR, sBBR, sSFR, sTFR, sBFR, sSBL, sTBL, sBBL;
extern int* cPositions[13];
extern const int nPositions[13];
extern const int sPositions[13];
//Positions for the GoTo function
extern const int sitpos[12];
extern const int standpos[12];
extern const int slideright[12];
extern uint8_t remoteMac[6];
extern bool walking;
extern bool sitting;
extern bool standing;
extern int selectedServo;*/


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
#define SERVOMIN 125
#define SERVOMAX 600


int angleToPulse(int ang){
    int pulse = map(ang,0, 270, SERVOMIN,SERVOMAX);
    return pulse;
}


void setServo(int motor, int angle) {
  if (motor == 6) {
      Serial.println("Invalid servo number");
      return;
  }
    int adjustedAngle = angle + servoOffsets[motor];
    servoDriver_module.setPWM(motor, 0, angleToPulse(adjustedAngle));
  *cPositions[motor] = angle;
}


void home()
{
  setServo(SFL, nSFL);
  setServo(SBR, nSBR);
  setServo(SFR, nSFR);
  setServo(SBL, nSBL);
  setServo(TFL, nTFL);
  setServo(TBR, nTBR);
  setServo(TFR, nTFR);
  setServo(TBL, nTBL);
  setServo(BFL, nBFL);
  setServo(BBR, nBBR);
  setServo(BFR, nBFR);
  setServo(BBL, nBBL);
  sitting = true;
  standing = false;
  return;
}

void stand()
{
  if (sitting)
  {
    setServo(SFL, sSFL); // Seiten ausrichten
    setServo(SBR, sSBR);
    setServo(SFR, sSFR);
    setServo(SBL, sSBL);
    setServo(TFL, cTFL - ((cTFL - sTFL) / 2)); // Top halb ausrichten
    setServo(TBR, cTBR - ((cTBR - sTBR) / 2));
    setServo(TFR, cTFR - ((cTFR - sTFR) / 2));
    setServo(TBL, cTBL - ((cTBL - sTBL) / 2));
    setServo(BFL, cBFL - ((cBFL - sBFL) / 3)); // Bottom halb ausrichten
    setServo(BBR, cBBR - ((cBBR - sBBR) / 3));
    setServo(BFR, cBFR - ((cBFR - sBFR) / 3));
    setServo(BBL, cBBL - ((cBBL - sBBL) / 3));
    delay(300);
    setServo(TFL, sTFL); // Top ausrichten
    setServo(TBR, sTBR);
    setServo(TFR, sTFR);
    setServo(TBL, sTBL);
    delay(300);
    setServo(BFL, sBFL); // Bottom ausrichten
    setServo(BBR, sBBR);
    setServo(BFR, sBFR);
    setServo(BBL, sBBL);
  }
  sitting = false;
  standing = true;
}

void GoTo(const int targetPositions[12])
{
    int stepSize = 1;
    bool allServosAtTarget = false;

    while (!allServosAtTarget)
    {
        allServosAtTarget = true;

        // Front Left
        if (abs(cSFL - targetPositions[0]) < stepSize)
        {
            setServo(SFL, targetPositions[0]);
        }
        else if (cSFL < targetPositions[0])
        {
            setServo(SFL, cSFL + stepSize);
            allServosAtTarget = false;
        }
        else if (cSFL > targetPositions[0])
        {
            setServo(SFL, cSFL - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cTFL - targetPositions[1]) < stepSize)
        {
            setServo(TFL, targetPositions[1]);
        }
        else if (cTFL < targetPositions[1])
        {
            setServo(TFL, cTFL + stepSize);
            allServosAtTarget = false;
        }
        else if (cTFL > targetPositions[1])
        {
            setServo(TFL, cTFL - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cBFL - targetPositions[2]) < stepSize * 2)
        {
            setServo(BFL, targetPositions[2]);
        }
        else if (cBFL < targetPositions[2])
        {
            setServo(BFL, cBFL + stepSize * 2);
            allServosAtTarget = false;
        }
        else if (cBFL > targetPositions[2])
        {
            setServo(BFL, cBFL - stepSize * 2);
            allServosAtTarget = false;
        }

        // Back Right
        if (abs(cSBR - targetPositions[3]) < stepSize)
        {
            setServo(SBR, targetPositions[3]);
        }
        else if (cSBR < targetPositions[3])
        {
            setServo(SBR, cSBR + stepSize);
            allServosAtTarget = false;
        }
        else if (cSBR > targetPositions[3])
        {
            setServo(SBR, cSBR - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cTBR - targetPositions[4]) < stepSize)
        {
            setServo(TBR, targetPositions[4]);
        }
        else if (cTBR < targetPositions[4])
        {
            setServo(TBR, cTBR + stepSize);
            allServosAtTarget = false;
        }
        else if (cTBR > targetPositions[4])
        {
            setServo(TBR, cTBR - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cBBR - targetPositions[5]) < stepSize * 2)
        {
            setServo(BBR, targetPositions[5]);
        }
        else if (cBBR < targetPositions[5])
        {
            setServo(BBR, cBBR + stepSize * 2);
            allServosAtTarget = false;
        }
        else if (cBBR > targetPositions[5])
        {
            setServo(BBR, cBBR - stepSize * 2);
            allServosAtTarget = false;
        }

        // Front Right
        if (abs(cSFR - targetPositions[6]) < stepSize)
        {
            setServo(SFR, targetPositions[6]);
        }
        else if (cSFR < targetPositions[6])
        {
            setServo(SFR, cSFR + stepSize);
            allServosAtTarget = false;
        }
        else if (cSFR > targetPositions[6])
        {
            setServo(SFR, cSFR - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cTFR - targetPositions[7]) < stepSize)
        {
            setServo(TFR, targetPositions[7]);
        }
        else if (cTFR < targetPositions[7])
        {
            setServo(TFR, cTFR + stepSize);
            allServosAtTarget = false;
        }
        else if (cTFR > targetPositions[7])
        {
            setServo(TFR, cTFR - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cBFR - targetPositions[8]) < stepSize * 2)
        {
            setServo(BFR, targetPositions[8]);
        }
        else if (cBFR < targetPositions[8])
        {
            setServo(BFR, cBFR + stepSize * 2);
            allServosAtTarget = false;
        }
        else if (cBFR > targetPositions[8])
        {
            setServo(BFR, cBFR - stepSize * 2);
            allServosAtTarget = false;
        }

        // Back Left
        if (abs(cSBL - targetPositions[9]) < stepSize)
        {
            setServo(SBL, targetPositions[9]);
        }
        else if (cSBL < targetPositions[9])
        {
            setServo(SBL, cSBL + stepSize);
            allServosAtTarget = false;
        }
        else if (cSBL > targetPositions[9])
        {
            setServo(SBL, cSBL - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cTBL - targetPositions[10]) < stepSize)
        {
            setServo(TBL, targetPositions[10]);
        }
        else if (cTBL < targetPositions[10])
        {
            setServo(TBL, cTBL + stepSize);
            allServosAtTarget = false;
        }
        else if (cTBL > targetPositions[10])
        {
            setServo(TBL, cTBL - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cBBL - targetPositions[11]) < stepSize * 2)
        {
            setServo(BBL, targetPositions[11]);
        }
        else if (cBBL < targetPositions[11])
        {
            setServo(BBL, cBBL + stepSize * 2);
            allServosAtTarget = false;
        }
        else if (cBBL > targetPositions[11])
        {
            setServo(BBL, cBBL - stepSize * 2);
            allServosAtTarget = false;
        }

        delay(20);
    }
}


void setServoSlow(int motor, int angle, int stepsize) {
  if (motor == 6) {
      Serial.println("Invalid servo number");
      return;
  }
  int currmotor = *cPositions[motor];
  if (currmotor < angle) {
      for (int i = currmotor; i <= angle; i += stepsize) {
          setServo(motor, i);
          delay(10);
      }
  } else if (currmotor > angle) {
      for (int i = currmotor; i >= angle; i -= stepsize) {
          setServo(motor, i);
          delay(10);
      }
  } else if (currmotor - angle < stepsize) {
      setServo(motor, angle);
  }
}

void setServoS(int GoUp)
{
    setServo(SFL, cSFL + GoUp);
    setServo(SBR, cSBR - GoUp);
    setServo(SFR, cSFR + GoUp);
    setServo(SBL, cSBL + GoUp);
}

void setServoT(int GoUp)
{
    // Rechts: hoch addieren, Links: hoch subtrahieren
    setServo(TFL, cTFL - GoUp);
    setServo(TBR, cTBR + GoUp);
    setServo(TFR, cTFR + GoUp);
    setServo(TBL, cTBL - GoUp);
}

void setServoB(int GoUp)
{
    // Rechts: hoch subtrahieren, Links: hoch addieren
    setServo(BFL, cBFL + GoUp);
    setServo(BBR, cBBR + GoUp);
    setServo(BFR, cBFR - GoUp);
    setServo(BBL, cBBL - GoUp);
}

void setServoTB(int GoUp)
{
  setServoT(GoUp);
  setServoB(2 * GoUp);
}

void moveServo(int selectedServo, int updown) {
  if (selectedServo == 6) {
      Serial.println("Invalid servo number");
      return;
  }
  int step = 5 * updown;
  setServo(selectedServo, *cPositions[selectedServo] + step);
}

void sidestepR()
{
    if (standing)
    {
        // Linke Seite absenken
        setServo(BFL, cBFL - 20);
        setServo(BBL, cBBL + 20);
        delay(100);
        waitforButton();
        // Rechtes Bein Vorne bewegen
        setServoSlow(BFR, cBFR + 20, 3);
        setServoSlow(TFR, cTFR - 10, 3);
        setServoSlow(SFR, cSFR - 20, 3);
        delay(50);
        setServoSlow(TFR, cTFR + 15, 3);
        setServoSlow(BFR, cBFR - 30, 3);
        // Rechtes Bein Hinten bewegen
        setServoSlow(BBR, cBBR - 20, 3);
        setServoSlow(TBR, cTBR + 10, 3);
        setServoSlow(SBR, cSBR + 20, 3);
        delay(50);
        setServoSlow(TBR, cTBR - 15, 3);
        setServoSlow(BBR, cBBR + 30, 3);
        waitforButton();
        // Alle Beine Seitlich bewegen
        GoTo(slideright);
    }
}

void rotateL()
{
    setServo(BFR, cBFR + 50);
    delay(20);
    setServo(SFR, cSFR + 10);
    delay(20);
    setServo(BFR, cBFR - 50);
    delay(200);
    setServo(BBL, cBBL + 50);
    delay(20);
    setServo(SBL, cSBL + 10);
    delay(20);
    setServo(BBL, cBBL - 50);
    delay(200);

    setServo(BFL, cBFL - 50);
    delay(20);
    setServo(SFL, cSFL + 10);
    delay(20);
    setServo(BFL, cBFL + 50);
    delay(200);
    setServo(BBR, cBBR - 50);
    delay(20);
    setServo(SBR, cSBR + 10);
    delay(20);
    setServo(BBR, cBBR + 50);
    delay(200);
    setServo(SFR, cSFR - 10);
    setServo(SBL, cSBL - 10);
    setServo(SFL, cSFL - 10);
    setServo(SBR, cSBR - 10);
}

void walk()
{
    setServo(BFR, cBFR + 30);
    setServo(TFR, sTFR + 15);
    delay(100);
    setServo(BFR, sBFR - 10);
    // setServo(TFR, sTFR);
    setServoT(-5);
    delay(100);

    setServo(BBL, cBBL + 30);
    setServo(TBL, sTBL - 5);
    delay(100);
    setServo(BBL, sBBL - 10);
    // setServo(TBL, sTBL);
    setServoT(-5);
    delay(100);

    setServo(BFL, cBFL - 35);
    setServo(TFL, sTFL - 5);
    delay(100);
    setServo(BFL, sBFL - 10);
    // setServo(TFL, sTFL);
    setServoT(-5);
    delay(100);

    setServo(BBR, cBBR - 25);
    setServo(TBR, sTBR);
    delay(100);
    setServo(BBR, sBBR - 10);
    // setServo(TBR, sTBR);
    // delay(200);

    // setServo(TFR, sTFR+10);
    // setServo(TFL, sTFL+10);
    // setServo(TBR, sTBR+10);
    // setServo(TBL, sTBL+10);

    GoTo(standpos);
}

void walkback()
{
    setServo(BFR, cBFR + 20);
    setServo(TFR, sTFR - 20);
    delay(100);
    setServo(BFR, sBFR - 5);
    // setServo(TFR, sTFR);
    setServoT(+5);
    delay(100);

    setServo(BBL, cBBL + 20);
    setServo(TBL, sTBL - 10);
    delay(100);
    setServo(BBL, sBBL);
    // setServo(TBL, sTBL);
    setServoT(+5);
    delay(100);

    setServo(BFL, cBFL - 25);
    setServo(TFL, sTFL + 5);
    delay(100);
    setServo(BFL, sBFL);
    // setServo(TFL, sTFL);
    setServoT(+5);
    delay(100);

    setServo(BBR, cBBR - 25);
    setServo(TBR, sTBR + 5);
    delay(100);
    setServo(BBR, sBBR + 10);
    // setServo(TBR, sTBR);
    // delay(200);

    // setServo(TFR, sTFR+10);
    // setServo(TFL, sTFL+10);
    // setServo(TBR, sTBR+10);
    // setServo(TBL, sTBL+10);

    delay(100);
    GoTo(standpos);
}

void correctAll()
{
  GoTo(standpos);
  delay(100);
  setServoTB(-10);
  delay(100);
  setServoTB(10);
}