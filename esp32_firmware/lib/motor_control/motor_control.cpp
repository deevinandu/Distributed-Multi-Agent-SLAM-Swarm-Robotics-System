#include "motor_control.h"

MotorController::MotorController() {}

void MotorController::init() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Setup PWM channels
    ledcSetup(left_channel, freq, resolution);
    ledcSetup(right_channel, freq, resolution);

    // Attach pins to channels
    ledcAttachPin(ENA, left_channel);
    ledcAttachPin(ENB, right_channel);

    stop();
}

void MotorController::drive(int left_speed, int right_speed) {
    setMotor(left_speed, ENA, IN1, IN2, left_channel);
    setMotor(right_speed, ENB, IN3, IN4, right_channel);
}

void MotorController::stop() {
    drive(0, 0);
}

void MotorController::setMotor(int speed, int ena_pin, int in1_pin, int in2_pin, int channel) {
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
    ledcWrite(channel, abs_speed);
}
