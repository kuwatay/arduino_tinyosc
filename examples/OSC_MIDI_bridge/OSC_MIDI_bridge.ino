/*
  ESP8266 / ESP32 OSC MIDI Bridge
  2018/6/6
  morecat_lab

  OSC              ->  MIDI
  /1/push1 f  1.0     NOTE ON
  /1/push1 f  0.0     NOTE OFF

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

#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

tinyosc tosc = tinyosc();

WiFiUDP udp;
unsigned int localPort = 9000;
const int PACKET_SIZE = 256;
char pBuffer[PACKET_SIZE];

#define LED 4

void setup() {
  static char ledStat = 1;
  pinMode(LED, OUTPUT);

  // setup MIDI
  MIDI.begin(1);

  //Connect to the WiFi network
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    if (ledStat == 0) {
      ledStat = 1;
      digitalWrite(LED, HIGH);
    } else {
      ledStat = 0;
      digitalWrite(LED, LOW);
    }
    Serial.print(".");
    delay(500);
  }

  udp.begin(localPort);

  for (int i = 0 ; i < 10 ; i++) {
    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
    delay(300);
  }
}

void loop() {
  int len;
  int note;

  len = udp.parsePacket();
  if (len == 0) {  // ignore
    delay(10);
  } else {
    udp.read(pBuffer, (len > PACKET_SIZE) ? PACKET_SIZE : len);
    if (tosc.isBundle(pBuffer)) {
      // ignore
    } else {
      tosc_message osc;
      tosc.parseMessage(&osc, pBuffer, len); // parse OSC message
      char *oscAddress = tosc.getAddress(&osc);
      if (strncmp(oscAddress, "/1/push", 7) == 0) { // if "/1/pushXX"
        note = strtol(oscAddress+7, NULL, 10) + 41; // note number
        if (strcmp(tosc.getFormat(&osc), "f") == 0) {   // make sure paramener is float
          float val = tosc.getNextFloat(&osc);
          if (val == 0.0) {
            digitalWrite(LED, 0);
            MIDI.sendNoteOff(note,0,1);
          } else if (val == 1.0) {
            digitalWrite(LED, 1);
            MIDI.sendNoteOn(note,127,1);
          }
        }
      }
    }
  }
}

