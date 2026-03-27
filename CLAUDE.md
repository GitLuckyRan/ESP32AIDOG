# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-based quadruped robot controller (SpotMicro-style). Runs a 50 Hz FreeRTOS control loop that converts body motion commands into 12 servo joint angles via inverse kinematics. An ESP32-CAM module sends commands (WALK/STAND/LIE) over UART; sensors (MPU6050 IMU, QMC5883L compass, dual ultrasonic) provide feedback. Code comments are primarily in Chinese.

## Build System

PlatformIO with Arduino framework targeting NodeMCU-32S (ESP32). Built with C++17 (`-std=gnu++17`). Debug prints are disabled by default (`DBG_PRINT_DISABLE=1`).

```bash
# Build
pio run

# Upload to board
pio run -t upload

# Serial monitor (115200 baud)
pio device monitor

# Clean build
pio run -t clean
```

No test framework is configured; validation is done on hardware via serial monitor.

## Architecture

### Dual-Core FreeRTOS Split

- **Core 1** (priority 5): `SpotControlLoopEntry` — real-time 50 Hz control loop (commands, sensors, IK, servos)
- **Core 0** (default): Arduino `loop()` — 1 Hz telemetry (IMU data to CAM, battery monitoring/safety)

Only `src/main.cpp` is active source. `src/ca.cpp` is a commented-out battery test file.

### Control Loop Pipeline (20ms cycle)

```
SpotControlLoopEntry() — every 20ms:
  CAM.readCamSerial()        → parse UART string commands
  CAM.SwitchCamMode()        → translate to WALK/STAND/REST strings
  motionService.handleMode() → trigger state transitions
  peripherals.update()       → poll IMU, compass, sonar
  motionService.update()     → state.step() updates body_state, then IK solver computes 12 angles
  servoController.update()   → lerp angles → PWM via PCA9685
```

`WARN_IF_SLOW()` macro flags iterations exceeding the 20ms budget.

### Central Data Flow

`body_state_t` is the main data structure flowing through the pipeline:
- Contains 6-DOF body pose (omega/phi/psi/xm/ym/zm) + 4x4 foot position matrix
- Each `MotionState::step()` modifies `body_state` (via `target_body_state` + lerp smoothing)
- `Kinematics::calculate_inverse_kinematics(body_state)` → 12 joint angles
- `MotionService::update_angles()` applies per-leg direction multipliers (`dir[12]`)

### Motion State Machine

`MotionService` manages transitions between three `MotionState` subclasses via `handleMode(cmd)`:

- **RestState** — body lowered to minimum height
- **StandState** — mid-height ready position
- **WalkState** — Bezier-curve gait with two modes: TROT (diagonal pairs, `speed_factor=2`) and CRAWL (sequential legs, `speed_factor=0.5`, with body-shift centroid tracking)

Command string mapping: "WALK" → WalkState, "STAND" → StandState, "REST"/"LIE" → RestState. States only transition when the mode actually changes.

States use `lerpToBody()` for smooth pose transitions with optional IMU compensation (low-pass filtered roll/pitch offsets at 0.15 blend factor).

### Leg Ordering and Angle Layout

Legs are indexed: FL=0, FR=1, RL=2, RR=3. Each leg has 3 DOF (hip rotation, knee pitch, ankle pitch), giving a flat `float[12]` array: `[FL_hip, FL_knee, FL_ankle, FR_hip, ...]`.

Mount offsets use body dimensions L=207.5mm, W=78mm. FR and RR legs (odd indices) have inverted local X in the IK solver.

### Kinematics

Analytical 3-DOF inverse kinematics per leg. Key dimensions: coxa 60.5mm, femur 111.2mm, tibia 118.5mm. Default standing height ~105mm. The IK solver short-circuits if `body_state` hasn't changed (equality check with epsilon tolerance).

### Servo Configuration

12 servos on PCA9685 at 50Hz PWM. The `defaults[12]` array in `servo_controller.h` is the primary tuning point — each servo has:
- `center_pwm` (270–322): zero-position PWM offset
- `direction` (±1): rotation direction
- `center_angle`: angle offset in degrees
- `conversion` (2.33–2.48): degrees-to-PWM factor

PWM output is clamped to [120, 500]. Angles are smoothed via `lerp(current, target, 0.5)` before PWM conversion.

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

- `lib/Kinematics/` — IK solver, `body_state_t`, `KinConfig` constants
- `lib/Motion/` — `MotionService` state machine, gait timing, math/lerp utilities
- `lib/peripherals/` — sensor abstraction (IMU, compass, sonar), `ServoController`, `BATTERY` module (C-style API)
- `lib/CamSerial/` — ESP32-CAM UART protocol (command receive + telemetry send)
- `include/motion_states/` — state pattern: `MotionState` base class + REST/STAND/WALK implementations

### Battery Safety

2S LiPo with voltage divider (5.463x) and current shunt (0.066 V/A). EMA-filtered readings. Safety relay triggers on overcurrent (>25A) or undervoltage (<4.4V). Battery module uses C-style global functions (`BATTERY_Init`, `BATTERY_Update`, `BATTERY_GetVoltage`, etc.).
