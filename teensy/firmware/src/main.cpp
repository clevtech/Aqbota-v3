#include <Arduino.h>
#include "analogWrite.h"

struct Encoder {
  const uint8_t PIN;
  uint32_t rotations;
  double RPM;
  uint32_t last_time;
  bool pressed;
  uint32_t picks;
  double RPS;
};

Encoder encoder1 = {15, 0, 0, millis(), false, 24, 0};

void IRAM_ATTR isr() {
  encoder1.rotations += 1;
  encoder1.pressed = true;
}


void setup(){
    Serial.begin(115200);
    pinMode(encoder1.PIN, INPUT_PULLUP);
    attachInterrupt(encoder1.PIN, isr, RISING);
}


void loop(){
    int now = millis();
    if (now - encoder1.last_time >= 10000) {
        encoder1.RPS = (encoder1.rotations/encoder1.picks)*1000/double((now - encoder1.last_time));
        encoder1.RPM = encoder1.RPS*60;
        encoder1.last_time = now;
        encoder1.rotations = 0;
        Serial.printf("RPM is %f rpm\n", encoder1.RPM);
        Serial.printf("RPS is %f rps\n", encoder1.RPS);
    }
}
