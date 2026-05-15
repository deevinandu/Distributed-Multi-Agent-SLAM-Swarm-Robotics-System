# Multi-Agent SLAM & Swarm Deployment Guide

This document explains the architecture of our dual-bot swarm system, how the territory avoidance works, and exactly how to deploy the system onto your physical robots.

## 1. System Overview

The system consists of **three main components**:
1. **Bot 1 (Agent 1):** ESP32 robot hugging the **left wall**, mapping its surroundings, and dodging Bot 2's territory.
2. **Bot 2 (Agent 2):** ESP32 robot hugging the **right wall** (operating mostly like Bot 1 but tracing the opposite wall), initialized with a different starting X-coordinate offset.
3. **Dual-Bot Mapper (Server):** A Python script running on your PC that listens to both bots simultaneously, plots their positions and sensors on a single map, and arbitrates their territories.

---

## 2. Hardware Differences Between the Bots

Both bots share the exact same motor, encoder, and IMU wiring. They only differ in their **Ultrasonic Trigger Pin** assignments to accommodate structural/wiring constraints:

| Component | Bot 1 (AGENT_ID=1) | Bot 2 (AGENT_ID=2) |
| :--- | :--- | :--- |
| **Front Trigger** | GPIO 2 | GPIO 16 |
| **Left Trigger** | GPIO 4 | GPIO 17 |
| **Back Trigger** | GPIO 5 | GPIO 5 |
| **Right Trigger** | GPIO 13 | GPIO 13 |
| **Local Port** | `8888` | `8889` |

---

## 3. How to Deploy (Step-by-Step)

### Step 3.1: Flash Bot 1
1. Open `AgentFirmware_Bot1/AgentFirmware_Bot1.ino` in the Arduino IDE.
2. Connect **Bot 1** via USB. Select its corresponding COM port.
3. Ensure your Wi-Fi credentials (`WIFI_SSID` and `WIFI_PASSWORD`) and the `agent_ip` (your PC's IPv4 address or `x.x.x.255` broadcast) are correct.
4. Click **Upload**. Wait for the "Done uploading" message.
5. Disconnect Bot 1.

### Step 3.2: Flash Bot 2
1. Open `AgentFirmware_Bot2/AgentFirmware_Bot2.ino` in the Arduino IDE.
2. Connect **Bot 2** via USB. Select its corresponding COM port.
3. Verify your Wi-Fi credentials are correct exactly as in Bot 1.
4. Click **Upload**. Wait for the "Done uploading" message.
5. Disconnect Bot 2.

### Step 3.3: Set Up the physical environment
1. Place Bot 1 at your desired origin point.
2. Place Bot 2 exactly next to it, at a known distance along the **X-axis** (e.g., exactly 0.5 meters to the right of Bot 1).
3. Both robots should be facing the same direction.

### Step 3.4: Run the Central Server
1. Open your terminal on your PC.
2. Navigate to the `server_nodes` directory.
3. Run the dual mapper, providing the initial physical separation (e.g., 0.5 meters):
   ```bash
   python dual_bot_mapper.py --separation 0.5
   ```
4. A dark-themed map window will open. You will see Bot 1 Start (Cyan) at `(0,0)` and Bot2 Start (Magenta) at `(0.5, 0)`.

### Step 3.5: Start the Mission
1. Power on Bot 1 using its battery switch.
2. Power on Bot 2 using its battery switch.
3. Both bots will connect to Wi-Fi, run their boot sequences (sensor self-tests, MPU calibration), and wait for `STARTUP_DELAY_SEC` (default 5 seconds).
4. Both bots will begin moving simultaneously. The map on your PC will update in real time with both robots' trajectories and point clouds.

---

## 4. How the "Territory Sharing" Works

To prevent the robots from needlessly re-mapping the exact same areas, we implemented a **Dynamic Bounding Box Avoidance System**.

1. **Telemetry Upload:** Both bots continuously send UDP `QSRL` packets containing their position `(x, y, yaw)` and 4 sensor readings to the Python Server.
2. **Server Grid Bounding:** The Python server computes a bounding box `[min_x, min_y, max_x, max_y]` of all points mapped by Bot 1. It computes a separate bounding box for all points mapped by Bot 2.
3. **Zone Broadcast:** Every 2.0 seconds, the Python Server sends a `ZONE` packet to Bot 1 containing Bot 2's bounding box. It sends Bot 1's bounding box to Bot 2.
4. **Collision Avoidance:** During navigation, every time a bot stops to make a decision, it calculates its "look-ahead" point (where it will be if it drives 30cm forward). 
5. **Evasive Action:** If that look-ahead point falls inside the other bot's `ZONE`, the bot overrides its normal wall-following algorithm and executes a forced **30° Turn** (Bot 1 turns Right, Bot 2 turns Left) to bounce away toward the center of the room and explore fresh territory.

---

## 5. Troubleshooting Checklists

**Map only shows one robot:**
* Check that both bots are turned on.
* Ensure both bots have the *exact* same WiFi SSID, Password, and `agent_ip`.
* Check the terminal output of `dual_bot_mapper.py` to see if it lists packets dropping from a specific agent.

**One robot is drifting or turning too widely:**
* Adjust `LEFT_SPEED_BOOST` in that specific bot's firmware. If it curves left, increase the boost (e.g., `1.15`). If it curves right, decrease the boost (e.g., `0.95`).

**Bots are mapping over each other excessively:**
* In `dual_bot_mapper.py`, change `ZONE_UPDATE_INTERVAL` from 2.0 to 1.0 seconds for faster territory updates.
* In the Bot firmware `isInForbiddenZone()`, increase the `margin` (e.g. `0.20` to `0.40`) to make the territory bounds feel physically "larger" to the navigating bot.
