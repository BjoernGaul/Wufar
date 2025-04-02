#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

uint16_t posStep = 5;

String intArraytoString(int intArray[]) {
  String str = "";
  int arraySize = 2;

  for (int i = 0; i < arraySize; i++)
  {
      str += String(intArray[i]);
      if (i < arraySize - 1) 
      {
          str += ", "; // Add a separator
      }
  }
  // Serial.println(str);
  return str;
}

int* stringToIntArray(String str)
{
  // Convert the string back to an integer array
  int startIndex = 0;
  int endIndex = str.indexOf(',');
  int* intArray = new int[2];
  int arrayIndex = 0;

  while (endIndex != -1) {
    intArray[arrayIndex++] = str.substring(startIndex, endIndex).toInt();
    startIndex = endIndex + 2; // Move past the comma and space
    endIndex = str.indexOf(',', startIndex);
  }
  intArray[arrayIndex] = str.substring(startIndex).toInt(); // Add the last number

  return intArray;
}

int readJoystick(int xPin,int yPin)
{
  int xValue = analogRead(xPin);
  int yValue = analogRead(yPin);
  
  if((xValue < 1300))
  {
    Serial.println("Left");
    return 1;
  }
  else if((xValue > 2700))
  {
    Serial.println("Right");
    return 2;
  }
  else if((yValue > 2900))
  {
    Serial.println("Down");
    return 3;
  }
  else if((yValue < 1300))
  {
    Serial.println("Up");
    return 4;
  }else{
    return 0;
  }
}

boolean runEvery(unsigned long interval, unsigned long &lastRunTime)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

void LoRa_sendMessage(String message) {
  Serial.println("Begin message");
  LoRa.beginPacket();                   // start packet
  LoRa.write(message.length());
  Serial.println(message.length());
  LoRa.print(message);
  Serial.println(message);
  LoRa.endPacket();

}

String onReceive(int packetSize) {
  if (packetSize == 0) return "";
  byte incomingLength = LoRa.read();    // incoming msg length

  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  if (incomingLength != message.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return "";                             // skip rest of function
  }

  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + message);
  Serial.println();
  return message;
}


void manageSend(uint8_t joyLeft, uint8_t joyRight)
{
  int msgArray[2];
  static uint8_t taskLeft = 0;
  static uint8_t taskRight = 0;
  if(!taskLeft)
  {
    int msgArray[2] = {100, joyLeft};
    LoRa_sendMessage(intArraytoString(msgArray));
    taskLeft = joyLeft;
  }else if(taskLeft != joyLeft)
  {
    taskLeft = 0;
  }
  
  if(!taskRight)
  {
    int msgArray[2] = {101, joyRight};
    LoRa_sendMessage(intArraytoString(msgArray));
    taskRight = joyRight;
  }else if(taskRight != joyRight)
  {
    taskRight = 0;
  }
}
