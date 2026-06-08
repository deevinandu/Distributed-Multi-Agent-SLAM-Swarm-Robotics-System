# Multi-Agent SLAM & Swarm Deployment Guide

This document explains the architecture of our dual-bot distributed SLAM swarm system, including the server-side SLAM engine, frontier-based exploration, heartbeat failover, and exactly how to deploy the system onto your physical robots.

## 1. System Overview

The system consists of **three main components**:
1. **Bot 1 (Agent 1):** ESP32 robot that defaults to **left-wall-following** but can switch to server-directed frontier waypoints (`GO_TO_TARGET` mode). Broadcasts 4-sensor readings + landmark signatures to the server.
2. **Bot 2 (Agent 2):** ESP32 robot that defaults to **right-wall-following** with the same frontier/target capabilities. Uses different trigger GPIO pins and listens on port `8889`.
3. **Mission Control Server (`dual_bot_mapper.py`):** A Python + PyGame application running on your PC that:
   - Receives telemetry from both bots via UDP
   - Runs **Pose-Graph SLAM** with landmark-based loop closure
   - Maintains an **Occupancy Grid** with Bresenham ray-casting
   - Detects **Frontiers** (boundaries of explored/unexplored space) and assigns them to bots
   - Manages **Territory Zones** to prevent redundant mapping
   - Monitors bot **Heartbeats** and handles failures gracefully
   - Renders everything in a real-time PyGame dashboard

---

## 2. Hardware Differences Between the Bots

Both bots share the exact same motor, encoder, and IMU wiring. They only differ in their **Ultrasonic Trigger Pin** assignments:

| Component | Bot 1 (AGENT_ID=1) | Bot 2 (AGENT_ID=2) |
| :--- | :--- | :--- |
| **Front Trigger** | GPIO 2 | GPIO 16 |
| **Left Trigger** | GPIO 4 | GPIO 17 |
| **Back Trigger** | GPIO 5 | GPIO 5 |
| **Right Trigger** | GPIO 13 | GPIO 13 |
| **Local UDP Port** | `8888` | `8889` |

---

## 3. Prerequisites

- **PC:** Python 3.10+ with `pygame` and `numpy` installed:
  ```bash
  pip install pygame numpy
  ```
- **Arduino IDE:** With ESP32 board support + libraries: `Adafruit_MPU6050`, `Eigen`
- **Network:** All devices (PC + both bots) on the same Wi-Fi network

---

## 4. How to Deploy (Step-by-Step)

### Step 4.1: Flash Bot 1
1. Open `AgentFirmware_Bot1/AgentFirmware_Bot1.ino` in the Arduino IDE.
2. Connect **Bot 1** via USB. Select its corresponding COM port.
3. Ensure your Wi-Fi credentials (`WIFI_SSID` and `WIFI_PASSWORD`) and the `agent_ip` (your PC's broadcast address, e.g. `x.x.x.255`) are correct.
4. Click **Upload**. Wait for the "Done uploading" message.
5. Disconnect Bot 1.

### Step 4.2: Flash Bot 2
1. Open `AgentFirmware_Bot2/AgentFirmware_Bot2.ino` in the Arduino IDE.
2. Connect **Bot 2** via USB. Select its corresponding COM port.
3. Verify your Wi-Fi credentials are correct exactly as in Bot 1.
4. Click **Upload**. Wait for the "Done uploading" message.
5. Disconnect Bot 2.

### Step 4.3: Set Up the Physical Environment
1. Place Bot 1 at your desired origin point.
2. Place Bot 2 exactly next to it, at a known distance along the **X-axis** (e.g., exactly 0.5 meters to the right of Bot 1).
3. Both robots should be facing the same direction.

### Step 4.4: Run the Mission Control Server
1. Open your terminal on your PC.
2. Navigate to the `server_nodes` directory.
3. Run the dual mapper, providing the initial physical separation:
   ```bash
   python dual_bot_mapper.py --separation 0.5
   ```
4. A dark-themed PyGame map window will open. Controls:
   - **Scroll** to zoom in/out
   - **Click + drag** to pan the view
   - **Close window** or `Ctrl+C` to stop

### Step 4.5: Start the Mission
1. Power on Bot 1 using its battery switch.
2. Power on Bot 2 using its battery switch.
3. Both bots will connect to Wi-Fi, run their boot sequences (sensor self-tests, MPU calibration), and wait for `STARTUP_DELAY_SEC` (default 5 seconds).
4. Both bots will begin moving. The PyGame dashboard will show:
   - Both robot positions (cyan/magenta triangles)
   - Point cloud map building in real-time
   - Territory zone overlays
   - Frontier targets (yellow diamonds) with assignment lines
   - SLAM loop closure connections (green lines)

---

## 5. Navigation Modes

Each bot has 5 navigation states:

| State | Description |
| :--- | :--- |
| `FOLLOW` | Default wall-following (left for Bot1, right for Bot2) |
| `CORNER_ROUND` | Drives straight to physically clear a corner gap |
| `TURN_TO_WALL` | Turns in 15° steps to re-acquire the wall |
| `AVOID_FRONT` | Turns away from front obstacles in 15° steps |
| `GO_TO_TARGET` | Drives toward a server-assigned frontier waypoint |

Wall-following is the **fallback** when no frontier target is assigned by the server.

---

## 6. How Territory Sharing & Frontier Allocation Works

### Territory Zones (Overlap Prevention)
1. Both bots send `QSRL` telemetry packets with position + 4 sensor distances + landmark type.
2. The server computes a bounding box of each bot's mapped area.
3. Every 2 seconds, Bot 1 receives Bot 2's bounding box (and vice versa) as a `ZONE` packet.
4. If a bot's 30cm look-ahead point falls inside the forbidden zone, it executes a forced 30° turn away.

### Frontier-Based Exploration
1. The server maintains a unified **Occupancy Grid** (5cm resolution) using Bresenham ray-casting.
2. **Frontier cells** (FREE cells adjacent to UNKNOWN cells) are detected and clustered.
3. Every 3 seconds, the server assigns each bot to the nearest unvisited frontier cluster.
4. A `TARG` packet with `(target_x, target_y)` is sent to each bot.
5. The bot switches to `GO_TO_TARGET` mode and drives toward the assigned frontier.

### Heartbeat Failover
- If no packet is received from a bot for **5 seconds**, it is declared **OFFLINE**.
- The server lifts the forbidden zone for the dead bot, allowing the surviving bot to map the entire area.
- If the dead bot comes back online, normal territory sharing resumes.

---

## 7. SLAM & Loop Closure

The server runs **Pose-Graph SLAM** with landmark-based loop closure:
- **Landmarks** are geometric signatures detected from the 4 sensors (corners, corridors, dead-ends, open areas).
- When a bot revisits a location near a previously detected matching landmark, a **loop closure** is triggered.
- The server applies a drift correction to the bot's odometry, reducing accumulated dead-reckoning error.
- **Cross-bot closures** are supported: Bot 1's landmarks can trigger closures for Bot 2.

---

## 8. Quasar-Lite v2 Protocol

| Packet | Direction | Magic | Size | Fields |
| :--- | :--- | :--- | :--- | :--- |
| QuasarPacket v2 | Bot → Server | `QSRL` | 42B | agent_id, odom_x/y/yaw, encoder, v2v, dist_front/left/back/right, landmark_type |
| ZonePacket | Server → Bot | `ZONE` | 20B | min_x, min_y, max_x, max_y |
| TargetPacket | Server → Bot | `TARG` | 12B | target_x, target_y |

---

## 9. Troubleshooting

**Map only shows one robot:**
* Check that both bots are turned on.
* Ensure both bots have the *exact* same WiFi SSID, Password, and `agent_ip`.
* Check the terminal output of `dual_bot_mapper.py` for heartbeat OFFLINE messages.

**One robot is drifting or turning too widely:**
* Adjust `LEFT_SPEED_BOOST` in that specific bot's firmware. If it curves left, increase the boost (e.g., `1.15`). If it curves right, decrease it (e.g., `0.95`).

**Bots are mapping over each other excessively:**
* In `dual_bot_mapper.py`, change `ZONE_UPDATE_INTERVAL` from 2.0 to 1.0 seconds.
* In the firmware `isInForbiddenZone()`, increase the `margin` (e.g. `0.20` to `0.40`).

**PyGame window doesn't open:**
* Ensure `pygame` is installed: `pip install pygame`
* On headless servers, PyGame requires a display. Use `--no-gui` mode (not yet implemented) or SSH with X forwarding.

**Frontier targets not being assigned:**
* Bots need to have explored enough to create FREE cells in the occupancy grid.
* Frontiers only appear at the boundary between FREE and UNKNOWN cells.
* Check terminal output for `[TARGET]` log messages.
