# Distributed Multi-Agent SLAM & Swarm Robotics System

[![ESP32](https://img.shields.io/badge/Platform-ESP32-blue)](https://www.espressif.com/)
[![Python](https://img.shields.io/badge/Visualization-Python-green)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A distributed SLAM system using two ESP32 microcontrollers communicating wirelessly via **ESP-NOW** (V2V) and **UDP** for real-time 2D floor plan visualization. Designed for resource-constrained mobile robots with sensor fusion and autonomous navigation.

---

## System Architecture

```
┌─────────────────┐    ESP-NOW     ┌─────────────────┐     UDP      ┌─────────────────┐
│  Sensor Node    │ ────────────▶  │   Main Agent    │ ──────────▶  │  Laptop/PC      │
│  (ESP32 #2)     │   (20 Hz)      │   (ESP32 #1)    │  (10 Hz)     │  Visualization  │
├─────────────────┤                ├─────────────────┤              ├─────────────────┤
│ • HC-SR04       │                │ • MPU6050 (IMU) │              │ • Matplotlib    │
│ • Distance CM   │                │ • L298N Motors  │              │ • Real-time Map │
│                 │                │ • Servo Sweep   │              │ • Triangle Bot  │
│                 │                │ • EKF Fusion    │              │                 │
└─────────────────┘                └─────────────────┘              └─────────────────┘
```

---

## Features

- **V2V Communication**: ESP-NOW wireless link between sensor node and main agent
- **Sensor Fusion**: 6-state Extended Kalman Filter fusing IMU + wheel encoders
- **Survey Navigation**: Stop → Scan 180° → Turn to clearest path → Verify → Resume
- **Real-time Visualization**: Triangle robot marker rotates with IMU heading
- **Quasar-Lite Protocol**: 755-byte UDP packets with pose + 181-point scan data

---

## Hardware

### ESP32 #1: Main Agent
| Component | GPIO | Function |
|-----------|------|----------|
| L298N ENA/ENB | 27, 14 | Motor PWM |
| L298N IN1-IN4 | 26, 25, 12, 33 | Motor Direction |
| SG90 Servo | 13 | Ultrasonic sweep |
| MPU6050 | 21 (SDA), 22 (SCL) | IMU |
| Encoder | 34 | Wheel odometry |

### ESP32 #2: Sensor Node
| Component | GPIO | Function |
|-----------|------|----------|
| HC-SR04 TRIG | 5 | Ultrasonic trigger |
| HC-SR04 ECHO | 18 | Ultrasonic echo |

---

## Quick Start

### 1. Flash Sensor Node
```bash
# Open SensorNode/SensorNode.ino in Arduino IDE
# Select ESP32 board → Upload
# Serial Monitor should show: [V2V] Sending: XX.XX cm
```

### 2. Flash Main Agent
```bash
# Open AgentFirmware.ino/AgentFirmware.ino.ino
# Update WiFi credentials if needed
# Upload → Monitor shows: [DEBUG] V2V Distance: XX cm
```

### 3. Run Visualization
```bash
cd server_nodes
python live_topdown_mapper.py
# Triangle robot appears, rotates with heading
```

---

## Navigation Algorithm

```
┌─────────────────────────────────────┐
│ Move Forward (PWM 100, slow)        │
│           ↓                         │
│ Distance < 40cm? ─────────────────┐ │
│           ↓ No                    │ │
│    Continue forward           Yes ↓ │
│                          ┌────────┴─┤
│                          │ STOP     │
│                          │ Survey   │
│                          │ 0°→180°  │
│                          ├──────────┤
│                          │ Find max │
│                          │ distance │
│                          ├──────────┤
│                          │ Turn to  │
│                          │ best dir │
│                          ├──────────┤
│                          │ Verify   │
│                          │ forward  │
└──────────────────────────┴──────────┘
```

---

## Protocol: Quasar-Lite

### UDP Packet Structure (755 bytes)
```cpp
struct QuasarPacket {
    char magic[4];        // "QSRL"
    uint8_t agent_id;     // Robot ID
    float odom_x, odom_y; // Position (meters)
    float odom_yaw;       // Heading (radians)
    int32_t encoder_total;
    uint32_t v2v_count;   // V2V packets received
    uint16_t scan_count;  // 181
    float ranges[181];    // Distance per degree
};
```
**Python unpack**: `'<4sBfffiIH181f'`

---

## File Structure

```
├── AgentFirmware.ino/
│   ├── AgentFirmware.ino.ino  # Main navigation controller
│   ├── ekf.h / ekf.cpp        # Extended Kalman Filter
│   └── motor_control.h/.cpp   # L298N motor driver
├── SensorNode/
│   └── SensorNode.ino         # Ultrasonic V2V broadcaster
├── server_nodes/
│   ├── live_topdown_mapper.py # Real-time 2D visualization
│   └── logs/                  # Scan data logs
└── CONTEXT.md                 # Detailed project context
```

---

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `STOP_THRESHOLD` | 40 cm | Survey trigger distance |
| `SLOW_MOTOR_SPEED` | 100 | Forward PWM (0-255) |
| `SERVO_SURVEY_DELAY` | 40 ms | Sweep speed per degree |
| UDP Port | 8888 | Visualization port |
| V2V Rate | 20 Hz | ESP-NOW update rate |

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| V2V Link DOWN | Both ESPs must connect to same WiFi for channel sync |
| Robot spins randomly | Check V2V packet count increasing on Serial |
| Sensor returns 400cm | Check HC-SR04 wiring, needs stable 5V |
| Triangle doesn't rotate | Verify IMU calibration, check `robot_yaw` in packets |

---

## License

MIT License - See [LICENSE](LICENSE)
