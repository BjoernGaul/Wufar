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
/*extern int cFLS, cFLT, cFLB, cBRS, cBRT, cBRB, cFRS, cFRT, cFRB, cBLS, cBLT, cBLB;
extern const int nFLS, nFLT, nFLB, nBRS, nBRT, nBRB, nFRS, nFRT, nFRB, nBLS, nBLT, nBLB;
extern const int sFLS, sFLT, sFLB, sBRS, sBRT, sBRB, sFRS, sFRT, sFRB, sBLS, sBLT, sBLB;
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
  setServo(FLS, nFLS);
  setServo(BRS, nBRS);
  setServo(FRS, nFRS);
  setServo(BLS, nBLS);
  setServo(FLT, nFLT);
  setServo(BRT, nBRT);
  setServo(FRT, nFRT);
  setServo(BLT, nBLT);
  setServo(FLB, nFLB);
  setServo(BRB, nBRB);
  setServo(FRB, nFRB);
  setServo(BLB, nBLB);
  sitting = true;
  standing = false;
  return;
}

void stand()
{
  if (sitting)
  {
    setServo(FLS, sFLS); // Seiten ausrichten
    setServo(BRS, sBRS);
    setServo(FRS, sFRS);
    setServo(BLS, sBLS);
    setServo(FLT, cFLT - ((cFLT - sFLT) / 2)); // Top halb ausrichten
    setServo(BRT, cBRT - ((cBRT - sBRT) / 2));
    setServo(FRT, cFRT - ((cFRT - sFRT) / 2));
    setServo(BLT, cBLT - ((cBLT - sBLT) / 2));
    setServo(FLB, cFLB - ((cFLB - sFLB) / 3)); // Bottom halb ausrichten
    setServo(BRB, cBRB - ((cBRB - sBRB) / 3));
    setServo(FRB, cFRB - ((cFRB - sFRB) / 3));
    setServo(BLB, cBLB - ((cBLB - sBLB) / 3));
    delay(300);
    setServo(FLT, sFLT); // Top ausrichten
    setServo(BRT, sBRT);
    setServo(FRT, sFRT);
    setServo(BLT, sBLT);
    delay(300);
    setServo(FLB, sFLB); // Bottom ausrichten
    setServo(BRB, sBRB);
    setServo(FRB, sFRB);
    setServo(BLB, sBLB);
  }
  sitting = false;
  standing = true;
}

void GoTo(const int targetPositions[12])
{
    int stepSize = 2;
    bool allServosAtTarget = false;

    while (!allServosAtTarget)
    {
        allServosAtTarget = true;

        // Front Left
        if (abs(cFLS - targetPositions[0]) < stepSize)
        {
            setServo(FLS, targetPositions[0]);
        }
        else if (cFLS < targetPositions[0])
        {
            setServo(FLS, cFLS + stepSize);
            allServosAtTarget = false;
        }
        else if (cFLS > targetPositions[0])
        {
            setServo(FLS, cFLS - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cFLT - targetPositions[1]) < stepSize)
        {
            setServo(FLT, targetPositions[1]);
        }
        else if (cFLT < targetPositions[1])
        {
            setServo(FLT, cFLT + stepSize);
            allServosAtTarget = false;
        }
        else if (cFLT > targetPositions[1])
        {
            setServo(FLT, cFLT - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cFLB - targetPositions[2]) < stepSize * 2)
        {
            setServo(FLB, targetPositions[2]);
        }
        else if (cFLB < targetPositions[2])
        {
            setServo(FLB, cFLB + stepSize * 2);
            allServosAtTarget = false;
        }
        else if (cFLB > targetPositions[2])
        {
            setServo(FLB, cFLB - stepSize * 2);
            allServosAtTarget = false;
        }

        // Back Right
        if (abs(cBRS - targetPositions[3]) < stepSize)
        {
            setServo(BRS, targetPositions[3]);
        }
        else if (cBRS < targetPositions[3])
        {
            setServo(BRS, cBRS + stepSize);
            allServosAtTarget = false;
        }
        else if (cBRS > targetPositions[3])
        {
            setServo(BRS, cBRS - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cBRT - targetPositions[4]) < stepSize)
        {
            setServo(BRT, targetPositions[4]);
        }
        else if (cBRT < targetPositions[4])
        {
            setServo(BRT, cBRT + stepSize);
            allServosAtTarget = false;
        }
        else if (cBRT > targetPositions[4])
        {
            setServo(BRT, cBRT - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cBRB - targetPositions[5]) < stepSize * 2)
        {
            setServo(BRB, targetPositions[5]);
        }
        else if (cBRB < targetPositions[5])
        {
            setServo(BRB, cBRB + stepSize * 2);
            allServosAtTarget = false;
        }
        else if (cBRB > targetPositions[5])
        {
            setServo(BRB, cBRB - stepSize * 2);
            allServosAtTarget = false;
        }

        // Front Right
        if (abs(cFRS - targetPositions[6]) < stepSize)
        {
            setServo(FRS, targetPositions[6]);
        }
        else if (cFRS < targetPositions[6])
        {
            setServo(FRS, cFRS + stepSize);
            allServosAtTarget = false;
        }
        else if (cFRS > targetPositions[6])
        {
            setServo(FRS, cFRS - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cFRT - targetPositions[7]) < stepSize)
        {
            setServo(FRT, targetPositions[7]);
        }
        else if (cFRT < targetPositions[7])
        {
            setServo(FRT, cFRT + stepSize);
            allServosAtTarget = false;
        }
        else if (cFRT > targetPositions[7])
        {
            setServo(FRT, cFRT - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cFRB - targetPositions[8]) < stepSize * 2)
        {
            setServo(FRB, targetPositions[8]);
        }
        else if (cFRB < targetPositions[8])
        {
            setServo(FRB, cFRB + stepSize * 2);
            allServosAtTarget = false;
        }
        else if (cFRB > targetPositions[8])
        {
            setServo(FRB, cFRB - stepSize * 2);
            allServosAtTarget = false;
        }

        // Back Left
        if (abs(cBLS - targetPositions[9]) < stepSize)
        {
            setServo(BLS, targetPositions[9]);
        }
        else if (cBLS < targetPositions[9])
        {
            setServo(BLS, cBLS + stepSize);
            allServosAtTarget = false;
        }
        else if (cBLS > targetPositions[9])
        {
            setServo(BLS, cBLS - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cBLT - targetPositions[10]) < stepSize)
        {
            setServo(BLT, targetPositions[10]);
        }
        else if (cBLT < targetPositions[10])
        {
            setServo(BLT, cBLT + stepSize);
            allServosAtTarget = false;
        }
        else if (cBLT > targetPositions[10])
        {
            setServo(BLT, cBLT - stepSize);
            allServosAtTarget = false;
        }

        if (abs(cBLB - targetPositions[11]) < stepSize * 2)
        {
            setServo(BLB, targetPositions[11]);
        }
        else if (cBLB < targetPositions[11])
        {
            setServo(BLB, cBLB + stepSize * 2);
            allServosAtTarget = false;
        }
        else if (cBLB > targetPositions[11])
        {
            setServo(BLB, cBLB - stepSize * 2);
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
    setServo(FLS, cFLS + GoUp);
    setServo(BRS, cBRS - GoUp);
    setServo(FRS, cFRS + GoUp);
    setServo(BLS, cBLS + GoUp);
}

void setServoT(int GoUp)
{
    // Rechts: hoch addieren, Links: hoch subtrahieren
    setServo(FLT, cFLT - GoUp);
    setServo(BRT, cBRT + GoUp);
    setServo(FRT, cFRT + GoUp);
    setServo(BLT, cBLT - GoUp);
}

void setServoB(int GoUp)
{
    // Rechts: hoch subtrahieren, Links: hoch addieren
    setServo(FLB, cFLB + GoUp);
    setServo(BRB, cBRB + GoUp);
    setServo(FRB, cFRB - GoUp);
    setServo(BLB, cBLB - GoUp);
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
        setServo(FLB, cFLB - 20);
        setServo(BLB, cBLB + 20);
        delay(100);
        waitforButton();
        // Rechtes Bein Vorne bewegen
        setServoSlow(FRB, cFRB + 20, 3);
        setServoSlow(FRT, cFRT - 10, 3);
        setServoSlow(FRS, cFRS - 20, 3);
        delay(50);
        setServoSlow(FRT, cFRT + 15, 3);
        setServoSlow(FRB, cFRB - 30, 3);
        // Rechtes Bein Hinten bewegen
        setServoSlow(BRB, cBRB - 20, 3);
        setServoSlow(BRT, cBRT + 10, 3);
        setServoSlow(BRS, cBRS + 20, 3);
        delay(50);
        setServoSlow(BRT, cBRT - 15, 3);
        setServoSlow(BRB, cBRB + 30, 3);
        waitforButton();
        // Alle Beine Seitlich bewegen
        GoTo(slideright);
    }
}

void rotateL()
{
    setServo(FRB, cFRB + 50);
    delay(20);
    setServo(FRS, cFRS + 10);
    delay(20);
    setServo(FRB, cFRB - 50);
    delay(200);
    setServo(BLB, cBLB + 50);
    delay(20);
    setServo(BLS, cBLS + 10);
    delay(20);
    setServo(BLB, cBLB - 50);
    delay(200);

    setServo(FLB, cFLB - 50);
    delay(20);
    setServo(FLS, cFLS + 10);
    delay(20);
    setServo(FLB, cFLB + 50);
    delay(200);
    setServo(BRB, cBRB - 50);
    delay(20);
    setServo(BRS, cBRS + 10);
    delay(20);
    setServo(BRB, cBRB + 50);
    delay(200);
    setServo(FRS, cFRS - 10);
    setServo(BLS, cBLS - 10);
    setServo(FLS, cFLS - 10);
    setServo(BRS, cBRS - 10);
}

void walk()
{
    setServo(FRB, cFRB + 30);
    setServo(FRT, sFRT + 15);
    delay(100);
    setServo(FRB, sFRB - 10);
    // setServo(FRT, sFRT);
    setServoT(-5);
    delay(100);

    setServo(BLB, cBLB + 30);
    setServo(BLT, sBLT - 5);
    delay(100);
    setServo(BLB, sBLB - 10);
    // setServo(BLT, sBLT);
    setServoT(-5);
    delay(100);

    setServo(FLB, cFLB - 35);
    setServo(FLT, sFLT - 5);
    delay(100);
    setServo(FLB, sFLB - 10);
    // setServo(FLT, sFLT);
    setServoT(-5);
    delay(100);

    setServo(BRB, cBRB - 25);
    setServo(BRT, sBRT);
    delay(100);
    setServo(BRB, sBRB - 10);
    // setServo(BRT, sBRT);
    // delay(200);

    // setServo(FRT, sFRT+10);
    // setServo(FLT, sFLT+10);
    // setServo(BRT, sBRT+10);
    // setServo(BLT, sBLT+10);

    GoTo(standpos);
}

void walkback()
{
    setServo(FRB, cFRB + 20);
    setServo(FRT, sFRT - 20);
    delay(100);
    setServo(FRB, sFRB - 5);
    // setServo(FRT, sFRT);
    setServoT(+5);
    delay(100);

    setServo(BLB, cBLB + 20);
    setServo(BLT, sBLT - 10);
    delay(100);
    setServo(BLB, sBLB);
    // setServo(BLT, sBLT);
    setServoT(+5);
    delay(100);

    setServo(FLB, cFLB - 25);
    setServo(FLT, sFLT + 5);
    delay(100);
    setServo(FLB, sFLB);
    // setServo(FLT, sFLT);
    setServoT(+5);
    delay(100);

    setServo(BRB, cBRB - 25);
    setServo(BRT, sBRT + 5);
    delay(100);
    setServo(BRB, sBRB + 10);
    // setServo(BRT, sBRT);
    // delay(200);

    // setServo(FRT, sFRT+10);
    // setServo(FLT, sFLT+10);
    // setServo(BRT, sBRT+10);
    // setServo(BLT, sBLT+10);

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