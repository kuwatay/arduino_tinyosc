/*
  ESP8266 / ESP32 OSC button (5 button version)
  2018/6/26
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
unsigned int sendPort = 9000;
const int PACKET_SIZE = 256;
char udpAddress[] = "192.168.1.255"; // broadcast
char pBuffer[PACKET_SIZE];

struct buttonConfigTag {
  int gpioNo;
  char oscCommand[20];
  int pinState;
} buttonConfig[5] = {
  32, "/1/toggle1", HIGH,
  33, "/1/toggle2", HIGH,
  34, "/1/toggle3", HIGH,
  35, "/1/toggle4", HIGH,
  39, "/1/toggle5", HIGH,
};
  // note that ESP32 (input-only port) has no builtin pull-up/pull-down register.
  // need to pull-up GPIO34, 35, 39 with EXTERNAL 103 register

const int ledPin = 4;

void setup() {
  Serial.begin(115200);

  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);

  // initialize the pushbutton pin as an input:
  for (int i = 0 ; i < 5 ; i++) {
    pinMode(buttonConfig[i].gpioNo, INPUT_PULLUP);
  }

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
  int v;
  float fval;
  
  for (int i = 0 ; i < 5 ; i++) {
    if ((v = digitalRead(buttonConfig[i].gpioNo)) != buttonConfig[i].pinState) {
      // new state
      buttonConfig[i].pinState = v;
      if (v == 0) {
        fval = 1.0f;
        // turn LED on:
        digitalWrite(ledPin, HIGH);
      } else {
        fval = 0.0f;
        // turn LED off:
        digitalWrite(ledPin, LOW);
      }
      udp.beginPacket(udpAddress,sendPort);
      // create new message
      len = tosc.writeMessage(pBuffer, sizeof(pBuffer), buttonConfig[i].oscCommand, "f", fval);
      // send udp packet
      udp.write((const uint8_t *)pBuffer, len);
      udp.endPacket();
      printf("%s %g\n", buttonConfig[i].oscCommand, fval);
      delay(300);
    }
  }
}

