# Simulation and Testing Tools

This directory contains offline developer tools and utilities for testing the Swarm SLAM map and GUI without requiring active physical robots.

Having these scripts allows developers to debug the `pygame` rendering logic, verify multi-agent mapping behavior, and test the server UI seamlessly using pre-recorded or synthetically generated data.

## Contents

- **`generate_fake_dual_session.py`**: Generates synthetic log files (.csv) that simulate two bots moving around a room and reporting telemetry/ultrasonic data.
- **`playback_dual_session.py`**: A simulator script that reads CSV logs (either real recorded logs or synthetically generated ones) and fires them via UDP packets to the main server, tricking the server into thinking live bots are running.
- **`render_bedroom_map.py`**: A script to visually render a static layout of an environment for debugging coordinate spaces and ultrasonic boundaries.
- **`send_test_zone.py`**: A developer utility that sends a mock `ZONE` territory packet via UDP to a bot to verify its avoidance logic is triggering correctly.
