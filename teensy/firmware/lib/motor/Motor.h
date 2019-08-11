#ifndef MOTOR_H
#define MOTOR_H

#include <analogWrite.h>
#include <Arduino.h>

class Controller
{
    public:
        enum driver {L298, BTS7960, ESC};
        Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB);
        void spin(int pwm);

    private:
        driver motor_driver_;
        int pwm_pin_;
        int motor_pinA_;
        int motor_pinB_;
};

#endif
