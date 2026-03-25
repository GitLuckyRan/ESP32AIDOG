# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-based quadruped robot controller (SpotMicro-style). Runs a 50 Hz FreeRTOS control loop that converts body motion commands into 12 servo joint angles via inverse kinematics. An ESP32-CAM module sends commands (WALK/STAND/LIE) over UART; sensors (MPU6050 IMU, QMC5883L compass, dual ultrasonic) provide feedback.

## Build System

PlatformIO with Arduino framework targeting NodeMCU-32S (ESP32).

```bash
# Build
pio run

# Upload to board
pio run -t upload

# Serial monitor (115200 baud)
pio device monitor
```

## Architecture

### Control Loop (50 Hz on FreeRTOS core 1)

```
SpotControlLoopEntry() — every 20ms:
  CAM.readCamSerial()        → parse WALK/STAND/LIE commands
  peripherals.update()       → poll IMU, compass, sonar
  motionService.update()     → state machine + inverse kinematics
  servoController.update()   → lerp angles → PWM via PCA9685
```

The main Arduino `loop()` runs at lower priority: sends IMU data to camera module and checks battery safety (1s interval).

### Motion State Machine

`MotionService` manages transitions between three `MotionState` subclasses:

- **RestState** — body lowered to minimum height
- **StandState** — mid-height ready position
- **WalkState** — Bezier-curve gait with two modes: TROT (diagonal pairs) and CRAWL (sequential)

States implement `step(body_state, dt)` which updates foot positions. The kinematics solver then computes 12 joint angles (3 DOF × 4 legs: hip rotation, knee pitch, ankle pitch).

### Kinematics

Analytical 3-DOF inverse kinematics per leg. Key dimensions: coxa 60.5mm, femur 111.2mm, tibia 118.5mm, body 207.5×78mm. Default standing height ~105mm.

### Hardware Wiring

| Peripheral | Interface | Pins |
|---|---|---|
| PCA9685 (12 servos) | I2C 0x40 | SDA=18, SCL=19 |
| MPU6050 IMU | I2C 0x68 | SDA=18, SCL=19 |
| QMC5883L compass | I2C 0x0D | SDA=18, SCL=19 |
| Ultrasonic L | GPIO | Trig=33, Echo=34 |
| Ultrasonic R | GPIO | Trig=32, Echo=35 |
| ESP32-CAM UART | UART1 | RX=5, TX=16 |
| Battery voltage | ADC | GPIO 36 |
| Battery current | ADC | GPIO 39 |
| Relay cutoff | Digital | GPIO 27 |

### Key Libraries

- `lib/Kinematics/` — inverse kinematics solver and math utilities
- `lib/Motion/` — MotionService state machine, gait timing
- `lib/peripherals/` — sensor abstraction (IMU, compass, sonar), servo controller, battery monitor
- `lib/CamSerial/` — ESP32-CAM UART protocol
- `include/motion_states/` — state pattern: MotionState base class + REST/STAND/WALK implementations

### Servo Configuration

12 servos on PCA9685 at 50Hz PWM. Each servo has center PWM offset (280–320), direction multiplier (±1), and degree-to-PWM conversion factor (2.3–2.5). PWM clamped to [120, 500]. Angles are smoothed via `lerp(current, target, 0.5)`.

### Battery Safety

2S LiPo with voltage divider (5.463×) and current shunt (0.066 V/A). EMA-filtered readings. Safety relay triggers on overcurrent (>25A) or undervoltage (<4.4V).
