#include "motor_control.h"
#include <Arduino.h>

MotorController::MotorController() {}

void MotorController::init() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // ESP32 Core v3.0+ API: ledcAttach(pin, freq, resolution)
    // Channels are auto-assigned internally
    ledcAttach(ENA, freq, resolution);
    ledcAttach(ENB, freq, resolution);

    stop();
}

void MotorController::drive(int left_speed, int right_speed) {
    setMotor(left_speed, ENA, IN1, IN2);
    setMotor(right_speed, ENB, IN3, IN4);
}

void MotorController::stop() {
    drive(0, 0);
}

void MotorController::setMotor(int speed, int ena_pin, int in1_pin, int in2_pin) {
    if (speed > 0) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
    } else if (speed < 0) {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
    } else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
    }

    int abs_speed = abs(speed);
    if (abs_speed > 255) abs_speed = 255;
    
    // v3.0+: ledcWrite takes PIN, not channel
    ledcWrite(ena_pin, abs_speed);
}
