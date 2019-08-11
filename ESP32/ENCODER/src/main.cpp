#include <Arduino.h>
#include "analogWrite.h"
#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "lino_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "lino_msgs/PID.h"
//header file for imu
#include "lino_msgs/Imu.h"

#include "lino_base_config.h"
#include "Motor.h"
#include "Kinematics.h"

const byte interruptPin = 15;
volatile int interruptCounter = 0;
int numberOfInterrupts = 0;
int last = millis();
int delta = 0;
int now = 0;
double RPM = 0;
int echo = 0;
 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  interruptCounter = interruptCounter + 1;
  portEXIT_CRITICAL_ISR(&mux);
}
 
void setup() {
    Serial.begin(115200);
    Serial.println("Monitoring interrupts: ");
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
 
}
 
void loop() {
 
    // if(interruptCounter>0){
        
    //     portENTER_CRITICAL(&mux);
    //     interruptCounter = interruptCounter-2;
    //     now = millis();
    //     delta = now - last;
    //     last = now;
    //     portEXIT_CRITICAL(&mux);

    //     numberOfInterrupts++;
    //     Serial.print("An interrupt has occurred. Total: ");
    //     Serial.println(numberOfInterrupts);

    //     if(delta > 100){
    //         RPM = (60000)/(4*delta);
    //         Serial.print("RPM is: ");
    //         Serial.println(RPM);
    //         Serial.println();
    //     }
    // }

    if(millis() - echo > 2000){
            echo = millis();
            portENTER_CRITICAL(&mux);
            numberOfInterrupts = interruptCounter;
            interruptCounter = 0;
            now = millis();
            delta = now - last;
            last = now;
            portEXIT_CRITICAL(&mux);


            Serial.print("An interrupt has occurred. Total: ");
            Serial.println(numberOfInterrupts);
            RPM = (numberOfInterrupts*60000)/(4*delta);
            Serial.print("RPM is: ");
            Serial.println(RPM);
            Serial.println();
        }
}


