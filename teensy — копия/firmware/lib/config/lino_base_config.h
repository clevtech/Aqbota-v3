#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

//uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER      // 4WD robot
// #define LINO_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define LINO_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
// #define LINO_BASE MECANUM         // Mecanum drive robot

//uncomment the motor driver you're using
// #define USE_L298_DRIVER
#define USE_BTS7960_DRIVER
// #define USE_ESC

//uncomment the IMU you're using
// define USE_GY85_IMU
#define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

//// INTERNET data
#define SERVER_IP [192,168,1,149]
#define SERVER_PORT 11411

#define DEBUG 1

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

//define your robot' specs here
#define MAX_RPM 16                // motor's maximum RPM
#define COUNTS_PER_REV 8          // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.54       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.25   // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.0    // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN

/// ENCODER PINS
#define MOTOR1_ENCODER 14 
#define MOTOR2_ENCODER 11

//// MOTOR PINS
#ifdef USE_BTS7960_DRIVER
  #define MOTOR_DRIVER BTS7960  

  #define MOTOR1_PWM 1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 21
  #define MOTOR1_IN_B 20

  #define MOTOR2_PWM 8 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 5
  #define MOTOR2_IN_B 6

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif


#endif
