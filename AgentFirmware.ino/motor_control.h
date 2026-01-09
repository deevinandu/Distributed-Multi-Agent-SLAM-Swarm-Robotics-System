#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorController {
public:
    MotorController();

    // Initialize pins and PWM channels
    void init();

    // Drive motors with speeds from -255 to 255
    void drive(int left_speed, int right_speed);

    // Stop all motors
    void stop();

private:
    // Left Motor Pins
    const int ENA = 32;
    const int IN1 = 33;
    const int IN2 = 25;

    // Right Motor Pins
    // Right Motor Pins
    const int ENB = 14; // Corrected from 26
    const int IN3 = 27; // Swapped from 26
    const int IN4 = 26; // Swapped from 27

    // PWM Settings
    const int freq = 5000;
    const int resolution = 8;
    // Channels are no longer needed in ESP32 Core v3.0+ (auto-assigned)

    // Updated signature: removed channel parameter
    void setMotor(int speed, int enc_pin, int in1_pin, int in2_pin);
};

#endif // MOTOR_CONTROL_H
