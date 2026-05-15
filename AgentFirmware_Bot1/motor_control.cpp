#include "motor_control.h"
#include <Arduino.h>

MotorController::MotorController() {}

void MotorController::init() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcAttach(ENA, freq, resolution);
    ledcAttach(ENB, freq, resolution);

    Serial.printf("[MOTOR] ENA=GPIO%d IN1=GPIO%d IN2=GPIO%d\n", ENA, IN1, IN2);
    Serial.printf("[MOTOR] ENB=GPIO%d IN3=GPIO%d IN4=GPIO%d\n", ENB, IN3, IN4);

    stop();
}

void MotorController::drive(int left_speed, int right_speed) {
    setMotor(left_speed,  ENB, IN3, IN4);  // ENB = physical left motor
    setMotor(right_speed, ENA, IN1, IN2);  // ENA = physical right motor
}

void MotorController::stop() {
    drive(0, 0);
}

void MotorController::testMotors() {
    Serial.println("[TEST] === MOTOR DIAGNOSTIC START ===");

    Serial.println("[TEST] LEFT motor FORWARD (1s)... watch which wheel spins");
    setMotor(180, ENA, IN1, IN2);
    setMotor(0,   ENB, IN3, IN4);
    delay(1000);
    stop(); delay(500);

    Serial.println("[TEST] RIGHT motor FORWARD (1s)... watch which wheel spins");
    setMotor(0,   ENA, IN1, IN2);
    setMotor(180, ENB, IN3, IN4);
    delay(1000);
    stop(); delay(500);

    Serial.println("[TEST] BOTH motors FORWARD (1s)...");
    drive(180, 180);
    delay(1000);
    stop();

    Serial.println("[TEST] === DONE. Note which wheel spun for each step ===");
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
    ledcWrite(ena_pin, abs_speed);
}
