//Mac Adresse Remote: 30:C9:22:EC:BE:AC
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

uint8_t remoteMac[6] = {0x30, 0xC9, 0x22, 0xEC, 0xB9, 0x80};
void readMacAdress();
void onDataReceive(const uint8_t * mac, const uint8_t * data, int len) {
  Serial.print("Received data from: ");
  int receivedNumber;
  int answer = 5;
  memcpy(&receivedNumber, data, sizeof(receivedNumber));
  if (receivedNumber == 3) {
    esp_err_t result = esp_now_send(remoteMac, (uint8_t *) &answer, sizeof(answer));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(5000);
  readMacAdress();
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onDataReceive);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, remoteMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
}

void loop() {

}

void readMacAdress(){
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}