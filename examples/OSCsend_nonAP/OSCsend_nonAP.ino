/*
  ESP8266 / ESP32 OSC sender
  2018/4/11
  morecat_lab

*/

#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#if defined(ESP32)
#include <WiFi.h>
#endif

#include <WiFiUdp.h>
#include "tinyosc.h"

#include "wifi_config.h"
// "wifi_config.h" should define the followings;
// const char ssid[] = "SSID";
// const char pass[] = "PASSWORD";

tinyosc tosc = tinyosc();

WiFiUDP udp;
unsigned int localPort = 9000;
unsigned int sendPort = 10000;
const int PACKET_SIZE = 256;
char udpAddress[] = "192.168.1.255"; // broadcast
char pBuffer[PACKET_SIZE];

float val = 0;

#define LED 4
#define BUTTON 0

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  //Connect to the WiFi network
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);

}

void loop() {
  int len;

  if (digitalRead(BUTTON) == 0) {
    udp.beginPacket(udpAddress,sendPort);
    if (val == 0) {
      val = 1.0;
      digitalWrite(LED, 1);
    } else {
      val = 0.0;
      digitalWrite(LED, 0);
    }
    // create new message
    len = tosc.writeMessage(pBuffer, sizeof(pBuffer), "/1/toggle1", "f", val);
    // send udp packet
    udp.write((const uint8_t *)pBuffer, len);
    udp.endPacket();
    printf("/1/toggle1 %g\n", val);
    delay(300);
  }
}

