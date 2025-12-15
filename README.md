# Distributed SLAM for a Swarm of Resource-Constrained Robots

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Language](https://img.shields.io/badge/Language-C%2B%2B%20%26%20Python-yellowgreen)](https://www.ros.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A complete, end-to-end ROS 2 system for collaborative 3D mapping using a swarm of resource-constrained, ESP32-powered mobile robots that offload heavy SLAM computation to a central server. This project serves as the basis for my Bachelor's thesis in Electronics and Telecommunications Engineering.

<!-- ![SYSTEM_ARCHITECTURE_DIAGRAM](path/to/your/diagram.png) -->
<!-- *(Note: Please create a system architecture diagram and replace the path above)* -->

---

## 1. Core Problem

Standard SLAM algorithms like `slam_toolbox` are computationally intensive and require significant memory, making them impossible to run directly on low-cost microcontrollers like the ESP32. This project solves that problem by creating a **distributed architecture** where resource-constrained "agent" robots handle real-time state estimation, while a powerful central server manages the computationally expensive tasks of map generation and multi-robot map merging.

This enables large-scale, collaborative mapping using swarms of simple, inexpensive robots.

## 2. Key Features

*   **Firmware-Level State Estimation:** A robust, multi-rate **Extended Kalman Filter (EKF)** runs on each ESP32, fusing high-frequency IMU data with lower-frequency wheel odometry to provide a smooth, real-time state estimate.
*   **Efficient Agent-to-Server Communication:** Utilizes **Micro-ROS over Wi-Fi** to stream `nav_msgs/Odometry` and `sensor_msgs/LaserScan` data reliably to the central ROS 2 network.
*   **Parallelized SLAM:** The central server runs a separate instance of the `slam_toolbox` node for each agent, allowing for simultaneous localization and local map generation for the entire swarm.
*   **Robust Map Merging:** A central Python node subscribes to all local maps and uses an **Iterative Closest Point (ICP)** algorithm from the Open3D library to align and stitch them into a single, unified global map.
*   **Validation & Stability:** The map merging pipeline includes a **fitness check** to reject bad ICP alignments, preventing the corruption of the global map and ensuring robustness.

## 3. System Architecture

The system is composed of two main components:

#### a) ESP32 Agent Firmware (`esp32_firmware/`)
*   **Language:** C++ (ESP-IDF & PlatformIO)
*   A real-time, multi-rate application with two main timers:
    *   A **100 Hz timer** reads the IMU and runs the EKF's `predict` step.
    *   A **10-20 Hz timer** reads wheel encoders, runs the EKF's `update` step, and publishes the final fused odometry and raw laser scan via Micro-ROS.

#### b) Central Server Workspace (`server_nodes/`)
*   **Language:** Python
*   A ROS 2 launch file starts multiple instances of `slam_toolbox` and the custom `map_merger` node.
*   The **Map Merger Node** subscribes to each agent's `/map` topic, converts the `OccupancyGrid` messages to point clouds, performs ICP registration, and publishes the final `nav_msgs/OccupancyGrid` on the `/global_map` topic.

## 4. Tech Stack & Key Libraries

*   **Frameworks:** ROS 2 (Humble/Iron), Micro-ROS, PlatformIO, ESP-IDF
*   **Languages:** C++17, Python 3
*   **Core Libraries:**
    *   **C++ (Agent):** Eigen (for EKF math), `rclc`
    *   **Python (Server):** `rclpy`, Open3D (for ICP), NumPy
*   **ROS 2 Packages:** `slam_toolbox`, `nav_msgs`, `sensor_msgs`

## 5. Repository Structure

```
├── esp32_firmware/           # PlatformIO project for the ESP32 agent
│   ├── src/
│   │   ├── main.cpp          # Main Micro-ROS node logic
│   ├── lib/
│   │   ├── ekf/
│   │   │   ├── ekf.cpp       # EKF implementation
│   │   │   └── ekf.h         # EKF header
│   └── platformio.ini        # Project configuration
└── server_nodes/             # Central Server Python Nodes
    └── map_merger.py         # Custom Map Merging Implementation
```

## 6. How to Run

### Prerequisites
*   A PC with ROS 2 (Humble or Iron) installed.
*   An ESP32 board configured with PlatformIO.
*   A running Micro-ROS agent on the PC.

### Steps
1.  **Start the Micro-ROS Agent:**
    ```bash
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    ```
2.  **Build and Flash the Firmware:**
    *   Open the `esp32_firmware/` directory in VSCode with PlatformIO.
    *   Update `WIFI_SSID`, `WIFI_PASSWORD`, and `agent_ip` in `main.cpp`.
    *   Build and upload the firmware to the ESP32.
3.  **Build and Launch the Server:**
    ```bash
    # From the ros2_ws/ directory
    colcon build
    source install/setup.bash
    ros2 run <pkg_name> map_merger
    ```
4.  **Visualize:**
    *   Open RViz and subscribe to the `/agent_1/map`, `/agent_2/map`, and `/global_map` topics to see the system in action.
