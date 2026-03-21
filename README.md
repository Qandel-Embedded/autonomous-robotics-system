# Autonomous Robotics System

[![CI](https://github.com/Qandel-Embedded/autonomous-robotics-system/actions/workflows/ci.yml/badge.svg)](https://github.com/Qandel-Embedded/autonomous-robotics-system/actions)
[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://python.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

ROS-based autonomous robot with custom PCB, Kalman-filter sensor fusion, A* path planning, and PID motor control.

## Architecture
```
sensors → KalmanFilter (fusion) → AStarPlanner → PIDController → motors
```

## Stack
| Layer | Technology |
|-------|-----------|
| Platform | Raspberry Pi 4 + STM32 co-processor |
| Framework | ROS Noetic |
| Sensors | LiDAR, IMU, encoders, camera |
| Communication | CAN Bus, SPI, I2C |
| PCB | Custom 4-layer (KiCad) |

## Quick Start
```bash
git clone https://github.com/Qandel-Embedded/autonomous-robotics-system
cd autonomous-robotics-system
pip install -r requirements.txt
python src/main.py
```

## Run Tests
```bash
pytest tests/ -v
```

## Results
- Navigation accuracy: < 5 cm deviation
- Path planning: < 20 ms on Pi 4
- Sensor fusion rate: 100 Hz

---
**Portfolio:** https://ahmedqandel.com | Available for hire on Upwork
