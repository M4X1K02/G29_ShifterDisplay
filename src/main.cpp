// has to be included if using PlatformIO
#include <Arduino.h>
// include the MAX7219 library
#include "LedControl.h"
// libraries for esp-now
#include <ESP8266WiFi.h>
#include <espnow.h>
 


const int8_t up_pin = D1;
const int8_t down_pin = D2;

uint8 brightness = 5;

volatile long t0 = 0;
volatile bool pressed_up = false;

IRAM_ATTR void ISR0() {
  if (!pressed_up) {
    t0 = millis();
    pressed_up = true;
  }
}

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int16_t gear;
} struct_message;
struct_message Gear;
 
// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&Gear, incomingData, sizeof(Gear));
  Serial.print("Bytes received: ");
  Serial.print(len);
  Serial.print(": ");
  Serial.println(Gear.gear);
}


/*
 Now we need an LedControl instance to work with.

 pin 13 is connected to the DataIn 
 pin 14 is connected to the CLK 
 pin 15 is connected to LOAD 
 We have only a single MAX7219.
 check your gpio pin numbers for the microcontroller you are using
 */
//                  MOSI/DIN,CLK,SS,NUM
LedControl lc=LedControl(13, 14, 15, 1);

// get hex code from: https://xantorohara.github.io/led-matrix-editor/
const uint64_t IMAGES[] = {
  0x00317b4e4c7f7f00,
  0x7f7f0c18307f7f00,
  0x0001017f7f110100,
  0x0031794945672300,
  0x00367f4949632200,
  0x00047f7f24140c00,
  0x004e5f5151737200,
  0x00266f49497f3e00,
  0x0060785f47606000,
  0x00367f49497f3600,
  0x003e7f49497b3200
};
const int IMAGES_LEN = sizeof(IMAGES)/8;

// function for converting hex to binary and setting each pixel one by one.
void displayImage(uint64_t image) {
  for (int i = 0; i < 8; i++) {
    byte row = (image >> i * 8) & 0xFF;
    for (int j = 0; j < 8; j++) {
      //    addr,row,col,value
      lc.setLed(0, i, j, bitRead(row, j));
    }
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
   
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
   
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  /* Set the brightness (0-10) */
  lc.setIntensity(0,brightness);
  /* and clear the display (black) */
  lc.clearDisplay(0);

  attachInterrupt(digitalPinToInterrupt(up_pin), ISR0, FALLING);
}

// main loop
void loop() {

  if (pressed_up) {
    if ((millis()-t0) > 20) {
      if (digitalRead(up_pin) == LOW) {
        if (brightness==10) {
          brightness = 0;
        }
        lc.setIntensity(0,brightness++);
      }
      pressed_up = false;
    }
  }
  // draw number
  displayImage(IMAGES[Gear.gear+1]);

  // 2 millisecond delay
  delay(2);
}