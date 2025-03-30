#include "functions.h"
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

extern Adafruit_PWMServoDriver servoDriver_module;

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
#define FLt 0
#define BRt 1
#define FRt 2
#define BLt 3
#define SERVOMIN 125
#define SERVOMAX 600

#define L1 8.4       // Upper Leg Length (cm)
#define L2 12.2      // Lower Leg Length (cm)
#define Z_STAND 12.5 // Hip Height while standing (cm)
#define X_OFFSET 2.0 // Feet slightly in front of the hip
#define Y_OFFSET 1.5 // Distance foot to hip
#define H_MAX -6.0
#define H_MIN 6.0

int angleToPulse(int ang)
{
  int pulse = map(ang, 0, 270, SERVOMIN, SERVOMAX);
  return pulse;
}

void setServo(int motor, int angle)
{
  if (motor < 0)
  {
    return;
  }
  int adjustedAngle = angle + servoOffsets[motor];
  servoDriver_module.setPWM(motor, 0, angleToPulse(adjustedAngle));
  *cPositions[motor] = angle;
}

void home()
{//! Delete if useless
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
  return;
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

void setServoSlow(int motor, int angle, int stepsize)
{
  int currmotor = *cPositions[motor];
  if (currmotor < angle)
  {
    for (int i = currmotor; i <= angle; i += stepsize)
    {
      setServo(motor, i);
      delay(10);
    }
  }
  else if (currmotor > angle)
  {
    for (int i = currmotor; i >= angle; i -= stepsize)
    {
      setServo(motor, i);
      delay(10);
    }
  }
  else if (currmotor - angle < stepsize)
  {
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
  setServo(FLT, cFLT - GoUp);
  setServo(BRT, cBRT + GoUp);
  setServo(FRT, cFRT + GoUp);
  setServo(BLT, cBLT - GoUp);
}

void setServoB(int GoUp)
{
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

void moveServo(int selectedServo, int updown)
{
  int step = 5 * updown;
  setServo(selectedServo, *cPositions[selectedServo] + step);
}

void sidestepR()
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
  moveLeg(FLt, 1, 0, 3);
  delay(100);
  moveLeg(BRt, 0, 0, 2);
  delay(100);
  moveLeg(BRt, -6, 0, 2);
  delay(100);
  moveLeg(BRt, -6, 0, 1);
  moveLeg(FLt, -6, 0, 1);
  delay(100);
  moveLeg(FLt, -6, 0, 0);

  standneutral();

  moveLeg(FRt, 1, 0, 3);
  delay(100);
  moveLeg(BLt, 0, 0, 2);
  delay(100);
  moveLeg(BLt, -6, 0, 2);
  delay(100);
  moveLeg(BLt, -6, 0, 1);
  moveLeg(FRt, -6, 0, 2);
  delay(100);
  moveLeg(FRt, -6, 0, 0);
  delay(100);
  standneutral();
}

// Function to compute the Inverse Kinematics for a leg
void computeIK(int legID, float x, float y, float z, float &theta1, float &theta2, float &theta3)
{
  x = -x;
  if (legID == 0 || legID == 3)
  { // Left Legs
    theta1 = atan2(y, z) * 180.0 / M_PI;
  }
  else
  { // Right Legs
    theta1 = -atan2(y, z) * 180.0 / M_PI;
  }

  float d = sqrt(x * x + z * z); // Distance Hip - Leg

  float cosTheta3 = (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2);
  theta3 = acos(cosTheta3) * 180.0 / M_PI; // Angle of Knee

  float alpha = atan2(x, z);
  float beta = acos((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d));
  theta2 = 90 - ((alpha + beta) * 180.0 / M_PI); // Angle of Hip
}

// Servo-Mapping (Pin-Number skip 6)
// int servoMap[12] = {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12};

// general Function to move a leg in a local coordinate system
void moveLegGeneralFunc(int legID, float x, float y, float z, int stepsize)
{
  if (singleLeg)
  {
    for (int i = 0; i < 12; i++)
    {
      nextPos[i] = *cPositions[i];
    }
  }
  float theta1, theta2, theta3;
  int currentTheta1[4] = {cFLS, cBRS, cFRS, cBLS};
  int currentTheta2[4] = {cFLT, cBRT, cFRT, cBLT};
  int currentTheta3[4] = {cFLB, cBRB, cFRB, cBLB};

  computeIK(legID, x, y, z, theta1, theta2, theta3);
  // flips angle values for different legs
  if (legID == 0 || legID == 1)
  {
    theta2 = -theta2;
  }
  if (legID == 2 || legID == 3)
  {
    theta3 = -theta3;
  }
  if (legID == 1 || legID == 3)
  {
    theta1 = -theta1;
  }
  // Calculate the difference between the current and the target angle
  int deltaTheta1 = round(theta1) - currentTheta1[legID];
  int deltaTheta2 = round(theta2) - currentTheta2[legID];
  int deltaTheta3 = round(theta3) - currentTheta3[legID];

  // Move the servos to the new position
  nextPos[legID * 3] = currentTheta1[legID] + deltaTheta1;
  nextPos[legID * 3 + 1] = currentTheta2[legID] + deltaTheta2;
  nextPos[legID * 3 + 2] = currentTheta3[legID] + deltaTheta3;

  if (singleLeg)
  {//Set to false to save all new positions first in order to execute them at the same time
    if (stepsize > 0)
    {
      setServoSlow(legID * 3, currentTheta1[legID] + deltaTheta1, stepsize);
      setServoSlow(legID * 3 + 1, currentTheta2[legID] + deltaTheta2, stepsize);
      setServoSlow(legID * 3 + 2, currentTheta3[legID] + deltaTheta3, stepsize);
    }
    else
    {
      setServo(legID * 3, currentTheta1[legID] + deltaTheta1);
      setServo(legID * 3 + 1, currentTheta2[legID] + deltaTheta2);
      setServo(legID * 3 + 2, currentTheta3[legID] + deltaTheta3);
    }
  }
}
// Leg function including offset calculation
void moveLeg(int legID, float x, float y, float z, int stepsize)
{
  // Offsets einbinden damit die Standard-Stehposition 0,0,0 ist
  float adjustedX;
  if (legID == 1 || legID == 3)
  {
    adjustedX = X_OFFSET - x;
  }
  else
  {
    adjustedX = x + X_OFFSET;
  }
  float adjustedY = y + Y_OFFSET;
  float adjustedZ = Z_STAND - z;

  moveLegGeneralFunc(legID, adjustedX, adjustedY, adjustedZ, stepsize);
}

void setStandingPose()
{
  singleLeg = false;
  for (int i = 0; i < 12; i++)
  {
    nextPos[i] = *cPositions[i];
  }
  moveLegGeneralFunc(0, X_OFFSET, Y_OFFSET, Z_STAND);
  moveLegGeneralFunc(1, X_OFFSET, Y_OFFSET, Z_STAND);
  moveLegGeneralFunc(2, X_OFFSET, Y_OFFSET, Z_STAND);
  moveLegGeneralFunc(3, X_OFFSET, Y_OFFSET, Z_STAND);
  GoTo(nextPos);
  singleLeg = true;
}

void standneutral()
{
  moveLeg(FLt, 0, 0, 0);
  moveLeg(BRt, 0, 0, 0);
  moveLeg(FRt, 0, 0, 0);
  moveLeg(BLt, 0, 0, 0);
}

void sidestepRR()
{
  // Lower left legs to shift weight to the left
  moveLeg(FLt, 0, 0, 3);
  moveLeg(BLt, 0, 0, 3);
  moveLeg(FRt, 0, 0, -1);
  moveLeg(BRt, 0, 0, -1);
  delay(200);
  // waitforButton();
  moveLeg(FLt, 0, 0, 4); // FL bisschen mehr absenken fürs gleichgewicht
  moveLeg(BRt, 0, 6, 2, 4);
  delay(100);
  moveLeg(BRt, 0, 6, -3, 4);
  delay(100);
  // waitforButton();
  // FR nach rechts und dann absenken
  moveLeg(FLt, 0, 0, 3);
  moveLeg(BLt, 0, 0, 4);
  moveLeg(FRt, 0, 6, 2, 4);
  delay(100);
  moveLeg(FRt, 0, 6, -3, 4);
  delay(100);
  // waitforButton();
  // FL nach rechts und dann absenken
  // Slide to the right
  moveLeg(FLt, 0, 6, -2);
  moveLeg(BLt, 0, 6, -2);
  moveLeg(FRt, 0, 0, 3);
  moveLeg(BRt, 0, 0, 3);
  delay(100);
  // waitforButton();
  // BL nachziehen
  moveLeg(BLt, -3, 1, 8, 4);
  setServo(BLS, cBLS + 10);
  // waitforButton();
  setServo(BLT, cBLT - 30);
  delay(100);
  // waitforButton();
  moveLeg(BLt, 0, 0, -1, 4);
  delay(100);
  // waitforButton();
  // FL nachziehen
  moveLeg(FLt, 4, 1, 8, 4);
  setServo(FLT, cFLT + 20);
  delay(100);
  moveLeg(FLt, 0, 0, -1, 4);
  delay(100);
  // waitforButton();
  // Normal stehen
  standneutral();
}

void sidestepLL()
{
  // Gewicht nach links verteilen
  moveLeg(FRt, 0, 0, 3);
  moveLeg(BRt, 0, 0, 3);
  moveLeg(FLt, 0, 0, -1);
  moveLeg(BLt, 0, 0, -1);
  delay(200);
  // waitforButton();
  moveLeg(FRt, 0, 0, 4); // FL bisschen mehr absenken fürs gleichgewicht
  moveLeg(BLt, 0, 7, 2, 4);
  delay(100);
  moveLeg(BLt, 0, 7, -3, 4);
  delay(100);
  // waitforButton();
  // FR nach rechts und dann absenken
  moveLeg(FRt, 0, 0, 3);
  moveLeg(BRt, 0, 0, 4);
  moveLeg(FLt, 0, 6, 2, 4);
  delay(100);
  moveLeg(FLt, 0, 6, -3, 4);
  delay(100);
  // waitforButton();
  // FL nach rechts und dann absenken
  // Slide to the right
  moveLeg(FRt, 0, 6, -2);
  moveLeg(BRt, 0, 6, -2);
  moveLeg(FLt, 0, 0, 3);
  moveLeg(BLt, 0, 0, 3);
  delay(100);
  // waitforButton();
  // BL nachziehen
  moveLeg(BRt, -3, 1, 8, 4);
  setServo(BRS, cBRS + 10);
  // waitforButton();
  setServo(BRT, cBRT + 30);
  delay(100);
  // waitforButton();
  moveLeg(BRt, 0, 0, -1, 4);
  delay(100);
  // waitforButton();
  // FL nachziehen
  moveLeg(FRt, 4, 1, 8, 4);
  setServo(FRT, cFRT - 20);
  delay(100);
  moveLeg(FRt, 0, 0, -1, 4);
  delay(100);
  // waitforButton();
  // Normal stehen
  standneutral();
}

void rotateRR()
{
  singleLeg = false;
  for (int i = 0; i < 12; i++)
  {
    nextPos[i] = *cPositions[i];
  }
  moveLeg(FRt, 0, -6, 6);
  moveLeg(BLt, 0, -6, 6);
  moveLeg(FLt, 0, 6, -3);
  moveLeg(BRt, 0, 6, -3);
  GoTo(nextPos);
  singleLeg = true;
  setServo(FLB, cFLB - 10);
  delay(100);
  // waitforButton();
  moveLeg(BRt, 0, 0, 6, 4);
  delay(100);
  // waitforButton();
  moveLeg(BRt, 0, -2, 2, 4);
  moveLeg(FLt, 0, 0, 6, 4);
  delay(100);
  // waitforButton();
  moveLeg(FLt, 0, -3, 1, 4);
  delay(100);
  // waitforButton();
  moveLeg(FRt, 0, 0, 8, 4);
  delay(100);
  // waitforButton();
  moveLeg(FRt, 0, 0, 2, 4);
  delay(100);
  // waitforButton();
  setServo(BLT, cBLT - 20);
  setServo(BLB, cBLB + 20);

  // waitforButton();
  setServo(BLS, cBLS - 30);
  moveLeg(BLt, 0, 0, 2, 4);
  // waitforButton();
  setStandingPose();
}

void rotateLL()
{
  singleLeg = false;
  for (int i = 0; i < 12; i++)
  {
    nextPos[i] = *cPositions[i];
  }
  moveLeg(FLt, 0, -6, 6);
  moveLeg(BRt, 0, -6, 6);
  moveLeg(FRt, 0, 6, -3);
  moveLeg(BLt, 0, 6, -3);
  GoTo(nextPos);
  singleLeg = true;
  setServo(FRB, cFRB - 10);
  delay(100);
  // waitforButton();
  moveLeg(BLt, 0, 0, 6, 4);
  delay(100);
  // waitforButton();
  moveLeg(BLt, 0, -2, 2, 4);
  moveLeg(FRt, 2, 0, 7, 4);
  delay(100);
  // waitforButton();
  moveLeg(FRt, 0, -3, 1, 4);
  delay(100);
  // waitforButton();
  moveLeg(FLt, 0, 0, 8, 4);
  delay(100);
  // waitforButton();
  moveLeg(FLt, 0, 0, 2, 4);
  delay(100);
  // waitforButton();
  setServo(BRT, cBRT - 20);
  setServo(BRB, cBRB + 20);

  // waitforButton();
  setServo(BRS, cBRS - 30);
  moveLeg(BRt, 0, 0, 2, 4);
  // waitforButton();
  setStandingPose();
}

void walkFF()
{

  moveLeg(FLt, 5, 0, 6);
  moveLeg(BRt, 5, 0, 6);
  moveLeg(FRt, 0, 0, 0);
  moveLeg(BLt, 0, 0, 0);
  delay(100);
  // waitforButton();
  moveLeg(FLt, 5, 0, 0);
  moveLeg(BRt, 5, 0, 0);
  delay(400);

  moveLeg(FRt, 5, 0, 6);
  moveLeg(BLt, 5, 0, 6);
  moveLeg(FLt, 0, 0, 0);
  moveLeg(BRt, 0, 0, 0);
  delay(100);
  // waitforButton();
  moveLeg(FRt, 5, 0, 0);
  moveLeg(BLt, 5, 0, 0);
  delay(400);
}

void bop()
{
  singleLeg = false;
  for (int i = 0; i < 12; i++)
  {
    nextPos[i] = *cPositions[i];
  }
  moveLeg(FLt, 0, 0, 5);
  moveLeg(BRt, 0, 0, 5);
  moveLeg(FRt, 0, 0, 5);
  moveLeg(BLt, 0, 0, 5);
  GoTo(nextPos);
  singleLeg = true;
  delay(300);
  singleLeg = false;
  for (int i = 0; i < 12; i++)
  {
    nextPos[i] = *cPositions[i];
  }
  moveLeg(FLt, 0, 0, -4);
  moveLeg(BRt, 0, 0, -4);
  moveLeg(FRt, 0, 0, -4);
  moveLeg(BLt, 0, 0, -4);
  GoTo(nextPos);
  singleLeg = true;
  delay(300);
}

void hump(){
  singleLeg = false;
  for (int i = 0; i < 12; i++)
  {
    nextPos[i] = *cPositions[i];
  }
  moveLeg(FLt, -4, 0, -3);
  moveLeg(BRt, -6, 0, 7);
  moveLeg(FRt, -4, 0, -3);
  moveLeg(BLt, -6, 0, 6);
  GoTo(nextPos);
  singleLeg = true;
  delay(100);
  setStandingPose();
  delay(100);
}

void changeHeight(float heightChange)
{
  height += heightChange;
  if (height < H_MAX)
  {
    height = H_MAX;
  }
  else if (height > H_MIN)
  {
    height = H_MIN;
  }
  singleLeg = false;
  for (int i = 0; i < 12; i++)
  {
    nextPos[i] = *cPositions[i];
  }
  moveLegGeneralFunc(0, X_OFFSET, Y_OFFSET, Z_STAND + height);
  moveLegGeneralFunc(1, X_OFFSET, Y_OFFSET, Z_STAND + height);
  moveLegGeneralFunc(2, X_OFFSET, Y_OFFSET, Z_STAND + height);
  moveLegGeneralFunc(3, X_OFFSET, Y_OFFSET, Z_STAND + height);
  GoTo(nextPos);
  singleLeg = true;
}