#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorController {
public:
    MotorController();
    void init();
    void drive(int left_speed, int right_speed);
    void stop();

    // Diagnostic: tests each motor independently, prints result to Serial
    void testMotors();

private:
    // Left Motor  — ENA must be an OUTPUT-capable GPIO (NOT 34/35/36/39)
    const int ENB = 32;
    const int IN4 = 33;
    const int IN3 = 25;

    // Right Motor
    const int ENA = 14;
    const int IN2 = 26;
    const int IN1 = 27;

    const int freq       = 5000;
    const int resolution = 8;

    void setMotor(int speed, int ena_pin, int in1_pin, int in2_pin);
};

#endif // MOTOR_CONTROL_H
