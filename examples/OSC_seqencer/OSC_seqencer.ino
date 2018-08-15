/*
  ESP32 based OSC Controller for step sequencer
  2018/7/28
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
unsigned int sendPort = 9000;
unsigned int localPort = 9000;
const int PACKET_SIZE = 256;
char udpAddress[] = "192.168.6.255"; // broadcast
char pBuffer[PACKET_SIZE];



#define DEBUG
// #define LED_TEST

#define OSC_MESSAGE_HEADER "/4/multitoggle/"
#define OSC_MESSAGE_HEADER_LENGTH 15
#define OSC_MESSAGE_HEADER_SEND "/1/toggle%d"

#define SCAN_ROW_SIZE 8
#define SCAN_COL_SIZE 8
#define MAX_KEYS (SCAN_ROW_SIZE * SCAN_COL_SIZE)
#define SCAN_COUNT_TH 150  /* read key every SCAN_COUNT_TH loop */

/* Pin Assignment for LED/SW matrix */
int scanCol [SCAN_COL_SIZE] = {21, 22, 23, 25, 26, 27, 32, 33};
int scanRow [SCAN_ROW_SIZE] = {12, 13, 14, 15, 16, 17, 18, 19};
int ain = 35; // IO35 for speed control
int ledPin = 4;

static uint8_t scan_col = 0;
static uint16_t scan_wait_count = 1;
static uint16_t tempo = 100;
static uint8_t seqStep = 0;  // 0 to 15

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
#ifdef DEBUG
            Serial.print("Key ON ");
            Serial.println( note );
#endif
          } else {
            matrixLedStatus[note] = (matrixLedStatus[note] == 0) ? 1 : 0;  // flip status
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
        if (row == seqStep) {
          /* LED OFF */
          pinMode(scanRow[row], INPUT_PULLUP);
        } else {
          /* LED ON */
          pinMode(scanRow[row], OUTPUT);
          digitalWrite(scanRow[row], LOW);
        }
      } else {
        if (row == seqStep) {
          /* LED ON */
          pinMode(scanRow[row], OUTPUT);
          digitalWrite(scanRow[row], LOW);
        } else {
          /* LED OFF */
          pinMode(scanRow[row], INPUT_PULLUP);
        }
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
  sprintf(msg, OSC_MESSAGE_HEADER_SEND, note);
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

  // analog in port
  pinMode(ain, INPUT);

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
  static long cnt = tempo * 10;
  static int scnt = 10;
  /* maintain matrix and read key */
  scanMatrix();

  /* check display column */
  if ((--cnt) == 0) {
    int v = 4096 - analogRead(ain);
    tempo = map(v, 0, 4095, 10000, 30000);
    cnt = tempo*10;
    /* check matrix status where (col == seqStep) */
    for (int row = 0 ; row < SCAN_ROW_SIZE ; row++){
      uint8_t note = (row * SCAN_ROW_SIZE) + seqStep;
      if (matrixLedStatus[note] == 1) {
#ifdef DEBUG_DETAILS
        Serial.print("KEY=");
        Serial.println(note);
#endif
        addOscMessage(row+1, 0); // 1 to 8
      }
    }
    if (++seqStep > 7) {
      seqStep = 0;
    }
    /* check matrix status where (col == seqStep) */
    for (int row = 0 ; row < SCAN_ROW_SIZE ; row++){
      uint8_t note = (row * SCAN_ROW_SIZE) + seqStep;
      if (matrixLedStatus[note] == 1) {
#ifdef DEBUG_DETAILS
        Serial.print("KEY=");
        Serial.println(note);
#endif
        addOscMessage(row+1, 1); // 1 to 8
      }
    }
  }

  /* send out sequence, if anyt */
  if ((--scnt) <= 0) {
    scnt = 2;
    if ((idx = getNextOscMessageIndex()) != -1) {
      sendOscMessage(oscMessage[idx].note, oscMessage[idx].state);
    }
  }
}

