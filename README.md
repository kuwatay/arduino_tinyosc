
# Arduino TinyOSC 

This is a quick port of tiny osc ( https://github.com/mhroth/tinyosc ) to Arduino ESP8266/ ESP32 environment.

## Installation

1. Download this repository and extract zip file.
1. Move extracted folder to ~/Document/Arduino/Library folder. (or suitable places in your environmrnt)
1. Restart Arduino IDE and you are ready.


## Usage

Create a file named "wifi_config.h" which includes the following;

1. const char ssid[] = "YOUR_SSID";
1. const char pass[] = "YOUR_PASSWORD";
 
Look at examples folders in details.

## Environment Tested 

* Arduino IDE 1.8.5
* esp8266 by ESP8266 Community version 2.4.1
* Arduino core for ESP32 WiFi chip 
 at https://github.com/espressif/arduino-esp32

