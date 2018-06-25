/*
  ESP8266 / ESP32 OSC server
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
const int PACKET_SIZE = 256;
char pBuffer[PACKET_SIZE];

#define LED 4

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

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

  len = udp.parsePacket();
  if (len == 0) {  // ignore
    delay(10);
  } else {
    udp.read(pBuffer, (len > PACKET_SIZE) ? PACKET_SIZE : len);
    if (tosc.isBundle(pBuffer)) {
      tosc_bundle bundle;
      tosc.parseBundle(&bundle, pBuffer, len);
      const uint64_t timetag = tosc.getTimetag(&bundle);
      tosc_message osc;
      while (tosc.getNextMessage(&bundle, &osc)) {
        tosc.printMessage(&osc);
      }
    } else {
      tosc_message osc;
      tosc.parseMessage(&osc, pBuffer, len); // parse OSC message
      tosc.printMessage(&osc);

      tosc.parseMessage(&osc, pBuffer, len); // parse OSC message (again)
      if (strcmp(tosc.getAddress(&osc), "/1/toggle1") == 0) { // if "/1/toggle1" button
        if (strcmp(tosc.getFormat(&osc), "f") == 0) {   // make sure paramener is float
          float val = tosc.getNextFloat(&osc);
          if (val == 0.0) {
            printf("TOGGLE1 OFF %g\n", val);
            digitalWrite(LED, 0);
          } else if (val == 1.0) {
            printf("TOGGLE1 ON %g\n", val);
            digitalWrite(LED, 1);
          }
        }
      }
    }
  }
}

