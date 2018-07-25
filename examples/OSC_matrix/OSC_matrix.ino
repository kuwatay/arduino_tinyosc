/*
  ESP32 based OSC Controller for Matrix LED/SW
  2018/7/16
  
  Copyright (C) 2018 by morecat_lab

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
char udpAddress[] = "192.168.0.255"; // broadcast
char pBuffer[PACKET_SIZE];

const int ledPin = 4;

#define DEBUG
// #define LED_TEST

#define OSC_MESSAGE_HEADER "/4/multitoggle/"
#define OSC_MESSAGE_HEADER_LENGTH 15
#define OSC_MESSAGE_HEADER_SEND "/4/multitoggle/%d/%d"

#define SCAN_ROW_SIZE 8
#define SCAN_COL_SIZE 8
#define MAX_KEYS (SCAN_ROW_SIZE * SCAN_COL_SIZE)
#define SCAN_COUNT_TH 150  /* read key every SCAN_COUNT_TH loop */

/* Pin Assignment for LED/SW matrix */
int scanCol [SCAN_COL_SIZE] = {21, 22, 23, 25, 26, 27, 32, 33};
int scanRow [SCAN_ROW_SIZE] = {12, 13, 14, 15, 16, 17, 18, 19};

static uint8_t scan_col = 0;
static uint16_t scan_wait_count = 1;

/* key status */
static uint8_t keyCountButton[MAX_KEYS];
static uint8_t buttonStatus[MAX_KEYS];

/* matrix LED status */
static uint8_t matrixLedStatus[MAX_KEYS];

/* OSC message queue */
#define MAX_QUEUE (1 << 5)
#define MAX_QUEUE_MASK (MAX_QUEUE -1)
#define WRAP_NEXT_INDEX(x) (((x)+1) & MAX_QUEUE_MASK)
struct oscMessage_t {
  uint8_t note;
  uint8_t state;
} oscMessage [MAX_QUEUE];

int oscMessagePtr = 0, oscMessageReadPtr = 0;

void addOscMessage( uint8_t note, uint8_t state) {
  if (WRAP_NEXT_INDEX(oscMessagePtr) == oscMessageReadPtr ) {
    return; /* when overflow, ignore new messages */ 
  } else {
    oscMessage[oscMessagePtr].note = note;
    oscMessage[oscMessagePtr].state = state;
    oscMessagePtr = WRAP_NEXT_INDEX(oscMessagePtr);
  }
}

int getNextOscMessageIndex() {
  if (oscMessagePtr == oscMessageReadPtr) {
    return -1;
  } else {
    int rtn = oscMessageReadPtr;
    oscMessageReadPtr = WRAP_NEXT_INDEX(oscMessageReadPtr);
    return rtn;
  }
}

void matrixInit() {
  uint8_t i;
  for (i = 0 ; i < MAX_KEYS ; i++) {
    keyCountButton[i] = 0;
    buttonStatus[i] = 1;
#ifdef LED_TEST
    // for cheaker pattern
    uint8_t x = i / 8;
    uint8_t y = i % 8;
    matrixLedStatus[i] = (x + y) % 2;     /* LED on/off */
#else
    matrixLedStatus[i] = 0;     /* LED off */
#endif

  }
}

void matrixOut(uint8_t note, uint8_t state) {
  if (note < MAX_KEYS) {
    matrixLedStatus[note] = state;
  }
}

void scanMatrix() {

  if ((--scan_wait_count) == 0) {
    /* make pre-column to Hi-Z */
    uint8_t preCol = (scan_col == 0) ? (SCAN_COL_SIZE - 1) : (scan_col - 1);
    pinMode(scanCol[preCol], INPUT_PULLUP);

/* =============KEY INPUT ===================== */
    /* make all row-pin to Hi-Z */
    for (uint8_t row = 0 ; row < SCAN_ROW_SIZE ; row++) {
      pinMode(scanRow[row], INPUT_PULLUP);
    }
    /* change COL to low and read key status */
    digitalWrite(scanCol[scan_col], LOW);
    pinMode(scanCol[scan_col], OUTPUT);

    for (uint8_t row = 0 ; row < SCAN_ROW_SIZE ; row++) {
      uint8_t note = (scan_col * SCAN_ROW_SIZE) + row;
      uint8_t x = digitalRead(scanRow[row]);     /* READ PORT */
      /* bounce canceler */
      if (x != buttonStatus[note]) {
        if (++keyCountButton[note] > 1) { /* KEY change detected */
          if (x == 0) {
            addOscMessage(note, 1);
#ifdef DEBUG
            Serial.print("Key ON ");
            Serial.println( note );
#endif
          } else {
            addOscMessage(note, 0);
#ifdef DEBUG
            Serial.print("Key OFF ");
            Serial.println( note );
#endif
          }
          keyCountButton[note] = 0;
          buttonStatus[note] = x;
        }
      } else {
        keyCountButton[note] = 0;
      }
    }

    /* COLをHi-Zにする */
    digitalWrite(scanCol[scan_col], HIGH);
    pinMode(scanCol[scan_col], INPUT_PULLUP);

    /* =============LED OCONTROL ===================== */
    for (uint8_t row = 0 ; row < SCAN_ROW_SIZE ; row++) {
      /* set row(j) to H(OUT) or L(OUT) */
      if (matrixLedStatus[(scan_col * SCAN_ROW_SIZE) + row] != 0) {
        /* LED ON */
        pinMode(scanRow[row], OUTPUT);
        digitalWrite(scanRow[row], LOW);
      } else {
        /* LED OFF */
        pinMode(scanRow[row], INPUT_PULLUP);
      }
    }

    /* set col(i) to H */
    pinMode(scanCol[scan_col], OUTPUT);
    digitalWrite(scanCol[scan_col], HIGH);

    /* set pointer to next column */
    scan_col ++;
    if (scan_col >= SCAN_COL_SIZE) {
      scan_col = 0;
    }

    /* set counter */
    scan_wait_count = SCAN_COUNT_TH; /* RESET COUNTER */
  }

}

void sendOscMessage(uint8_t note, uint8_t state) {
  udp.beginPacket(udpAddress,sendPort);
  // create new message
  float fval = (state == 1) ? (1.0) : (0.0);
  char msg [30];
  sprintf(msg, OSC_MESSAGE_HEADER_SEND, note % SCAN_ROW_SIZE +1, note / SCAN_ROW_SIZE +1);
  int len = tosc.writeMessage(pBuffer, sizeof(pBuffer), msg, "f", fval);
  // send udp packet
  udp.write((const uint8_t *)pBuffer, len);
  udp.endPacket();
#ifdef DEBUG
  Serial.printf("%s %g\n", msg, fval);
#endif
}

/* ----------------------------------------------------------- */

void setup() {
  Serial.begin(115200);

  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);

  matrixInit();

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
  int idx;
  int len;
  static int cnt = 1;
  /* maintain matrix and read key */
   scanMatrix();

/* send key status, if changed */
  if ((idx = getNextOscMessageIndex()) != -1) {
    sendOscMessage(oscMessage[idx].note, oscMessage[idx].state);
  }

  if ((--cnt) == 0) { cnt = 10;
    /* receive osc message from wi-fi, if any */
    if ((len = udp.parsePacket()) > 0) {
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
#ifdef DEBUG
        tosc.parseMessage(&osc, pBuffer, len); // parse OSC message
        tosc.printMessage(&osc);
#endif
        tosc.parseMessage(&osc, pBuffer, len); // parse OSC message (again)
        char *address = tosc.getAddress(&osc);
        if (strncmp(address, OSC_MESSAGE_HEADER, OSC_MESSAGE_HEADER_LENGTH) == 0) { // if "/4/multitoggle/X/X" 
          int row = atoi(address + OSC_MESSAGE_HEADER_LENGTH) - 1;
          int col = atoi(address + OSC_MESSAGE_HEADER_LENGTH+2) - 1;
          if (strcmp(tosc.getFormat(&osc), "f") == 0) {   // make sure paramener is float
            float val = tosc.getNextFloat(&osc);
            if (val == 0.0) {
              printf("TOGGLE %d %d OFF %g\n", col, row, val);
              // digitalWrite(LED, 0);
              matrixLedStatus[col * 8 + row] = 0;
            } else if (val == 1.0) {
              printf("TOGGLE %d %d ON %g\n", col, row, val);
              // digitalWrite(LED, 1);
              matrixLedStatus[col * 8 + row] = 1;
            }
          }
        }
      }
    }
  }
}

