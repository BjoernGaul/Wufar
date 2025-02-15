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

void home();                    //Ausgansposition (Entspricht sit)
void stand();                   //Fährt in Stehposition
void GoTo(const char* position);//Nimmt einen String mit 12 Positionen und fährt  
void sidestepR(); 
void rotateL();
void walk();
void walkback();
void correctAll();              //Hebt die Beine kurz an um unebenheiten auszugleichen (Alpha)
void setServo(int motor, int &currmotor, int angle);  //Bewegt einen Servo
void setServoS(int GoUp);                             //Bewegt alle Seitlichen Servos
void setServoT(int GoUp);                             //Bewegt alle Top Servos
void setServoB(int GoUp);                             //Bewegt alle Bottom Servos
void setServoTB(int GoUp);                            //Bewegt Top und Bottom Servos
void setServoSlow(int motor, int &currmotor, int angle, int stepsize); //Bewegt einen Servo langsam
void moveServo(int selectedServo, int updown); //Bewegt einen einzelnen Servo in 5er schritten //updown: 1 = up, -1 = down


#endif