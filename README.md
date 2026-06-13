# Distributed SLAM for a Swarm of Resource-Constrained Robots

[![Language](https://img.shields.io/badge/Language-C%2B%2B%20%26%20Python-yellowgreen)](#)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A lightweight, custom-built system for collaborative 2D spatial mapping using a swarm of inexpensive, ESP32-powered mobile robots. This project serves as the basis for a Bachelor's thesis in Electronics and Telecommunications Engineering.

Instead of relying on heavy frameworks like ROS 2, this project uses a highly optimized, raw UDP architecture to offload mapping computation to a central Python server. This allows resource-constrained microcontrollers to act as a coordinated swarm.

---

## 1. Core Architecture

The system avoids the overhead of standard SLAM algorithms (like `slam_toolbox`) running locally on the robots. Instead, it uses a **distributed approach**:
- **Agent Robots (ESP32):** Handle real-time motor control, ultrasonic wall-following, odometry estimation, and direct vehicle-to-vehicle communication.
- **Central Server (Python):** Subscribes to lightweight telemetry streams from all bots, maintains a global occupancy grid, renders the live map, handles SLAM loop closures, and coordinates the swarm by assigning territories.

## 2. Key Features

*   **Raw UDP Telemetry:** Bots stream a custom binary `QuasarPacket` (containing odometry, 4-way ultrasonic distances, and geometric landmark signatures) directly to the server, ensuring extremely low latency.
*   **Territory Avoidance:** The server dynamically calculates exploration bounding boxes and sends `ZONE` packets back to the bots. If a bot's trajectory enters another bot's territory, its local navigation overrides the wall-following and forces it to turn away, preventing collisions and redundant mapping.
*   **Custom Central Mapper:** A Python-based Pygame server (`dual_bot_mapper.py`) that processes telemetry, maintains an occupancy grid, renders the live point-cloud map, and handles path tracking.
*   **ESP-NOW V2V Communication:** Bots initialize ESP-NOW for direct vehicle-to-vehicle radio awareness, completely independent of the central WiFi router.
*   **Return-to-Home Logic:** Bots can detect when they have explored their environment and autonomously trigger a return-to-base sequence.

## 3. Hardware Stack

*   **Microcontroller:** ESP32
*   **Sensors:** 4x Ultrasonic Sensors (Front, Left, Back, Right)
*   **Odometry:** Wheel Encoders & IMU (MPU6050/9250)

## 4. Repository Structure

```
├── AgentFirmware_Bot1/       # C++ firmware for Bot 1 (Left-wall follower)
├── AgentFirmware_Bot2/       # C++ firmware for Bot 2 (Right-wall follower)
├── server_nodes/             # Central Server Python Nodes
│   └── dual_bot_mapper.py    # Custom Pygame Map Rendering & Swarm Coordination
├── simulation_tools/         # Offline developer utilities
│   ├── generate_fake_dual_session.py
│   ├── playback_dual_session.py
│   └── README.md             
```

## 5. How to Run

### Prerequisites
*   A PC with Python 3 and `pygame` installed.
*   ESP32 boards configured with Arduino IDE or PlatformIO.

### Steps
1.  **Configure Network:** Ensure your PC and the ESP32s are on the same WiFi network (or mobile hotspot). Update the `ssid` and `password` inside the `.ino` files.
2.  **Flash the Firmware:** 
    * Flash `AgentFirmware_Bot1.ino` to the first ESP32.
    * Flash `AgentFirmware_Bot2.ino` to the second ESP32.
3.  **Launch the Server:**
    ```bash
    python server_nodes/dual_bot_mapper.py
    ```
4.  **Deploy Swarm:** Power on the bots. They will automatically begin their wall-following routines and stream telemetry to the server, generating the map in real-time.
