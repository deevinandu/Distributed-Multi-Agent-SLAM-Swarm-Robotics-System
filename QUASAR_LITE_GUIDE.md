# Quasar-Lite: Real-Time Telemetry Guide

This guide explains how to operate the real-time mapping system and provides a technical breakdown of the **Quasar-Lite** protocol for your project documentation or presentation.

---

## 1. How to Receive Real-Time Packets & See the Map

To see the map being created in real-time while your robot is moving, follow these steps:

### **Step A: Start the Receiver (The Brain)**
Open a terminal on your PC and run:
```bash
python server_nodes/room_mapper.py
```
*   **What it does**: This script "listens" on UDP Port **8888**. It waits for the robot to shout its data across the WiFi network.
*   **The Window**: A Radar-style (Polar) window will open. As the robot scans, you will see a blue "sweep" line and red dots appear where obstacles are detected.

### **Step B: Turn on the Robot**
When the ESP32 boots up, it will connect to your WiFi and start sending packets. If the connection is successful, you will see:
1.  **Terminal**: `Agent 1 | Points: 181 | Closest: 0.45m` (This proves data is arriving).
2.  **Window**: The blue lines will jump and adjust as the servo moves.

---

## 2. Explaining "Quasar-Lite" (The Protocol)

When someone asks you, *"How does your robot talk to the computer?"*, you can explain **Quasar-Lite** with these three key points:

### **A. It is a "Binary Packet" Protocol**
Instead of sending slow, heavy text (like JSON or XML), Quasar-Lite sends raw **Binary Data**. 
*   **Why?**: It is incredibly fast. A packet that would take 2,000 characters in text only takes **743 bytes** in Quasar-Lite.
*   **Reliability**: It uses **UDP (User Datagram Protocol)**. In robotics, we prefer losing a packet occasionally over having the robot "lag" while waiting for a confirmation.

### **B. The Packet Structure (743 Bytes)**
Each packet is a "snapshot" of the robot's reality:
*   **Magic Header (4 bytes)**: `QSRL` — The password that identifies the data.
*   **Agent Identity (1 byte)**: Tells the server *which* robot sent the data (crucial for swarms).
*   **State Vector (12 bytes)**: The $(X, Y, Yaw)$ coordinates calculated by the EKF.
*   **Payload (724 bytes)**: 181 ultrasonic distance values (one for every degree).

### **C. The "Wavelet" Pedigree**
Quasar-Lite is a streamlined version of the full **Quasar Protocol**. It is designed for "High-Frequency Telemetry"—meaning the robot can update the server 10-20 times per second without choking the WiFi network.

---

## 3. The 1.2m "Trust Filter" (The Logic)
You can highlight this specific feature in your explanation:
> "Our system includes a **Trust-Zone Filter**. While the sensor can detect objects far away, we only 'ink' them into the map if they are within **1.2 meters**. This ensures that reflections and noise from distant walls don't ruin the accuracy of the room map."

---

### **Quick Checklist for Success:**
1.  **WiFi**: Ensure your PC IP address still starts with `10.57.0`.
2.  **Firewall**: If no data appears, ensure Windows Firewall isn't blocking Python.
3.  **Visualizer**: Use `room_mapper.py` for live scans, and `generate_topdown_map.py` to see the final floor plan after you stop the robot.
