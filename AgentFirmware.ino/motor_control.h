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
    // Left Motor Pins (Swapped with Right)
    const int ENA = 14;
    const int IN1 = 27;
    const int IN2 = 26;

    // Right Motor Pins (Swapped with Left)
    const int ENB = 32;
    const int IN3 = 33;
    const int IN4 = 25;

    // PWM Settings
    const int freq = 5000;
    const int resolution = 8;
    // Channels are no longer needed in ESP32 Core v3.0+ (auto-assigned)

    // Updated signature: removed channel parameter
    void setMotor(int speed, int enc_pin, int in1_pin, int in2_pin);
};

#endif // MOTOR_CONTROL_H
