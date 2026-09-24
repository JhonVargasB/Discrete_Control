# SCARA Robot — Discrete LQI Control

Implementation of a **2-DOF planar SCARA robot** controlled with a discrete **LQI position controller** running on a **Seeed Studio XIAO ESP32-S3**.

The system receives Cartesian references from MATLAB, computes inverse kinematics on the microcontroller and controls two DC motors with encoders in real time.

## Main features

- 2R planar SCARA kinematics
- Closed-form inverse kinematics
- Discrete LQI position control
- Encoder feedback
- DC motor actuation through a **TB6612** dual H-bridge
- PWM control
- FreeRTOS-based firmware
- USB communication between MATLAB and ESP32
- Real-time telemetry and visualization
- Joint-limit checking
- Saturation and anti-windup mechanisms

## System overview

The host computer sends a Cartesian target `(x, y)` to the ESP32.

The microcontroller then:

1. Validates the requested Cartesian position.
2. Computes the corresponding joint references using inverse kinematics.
3. Runs one LQI controller for each motor.
4. Reads encoder feedback.
5. Applies PWM commands through the TB6612.
6. Returns telemetry to MATLAB for visualization and analysis.

## Hardware

- Seeed Studio XIAO ESP32-S3
- 2 DC motors with encoders
- TB6612 dual H-bridge
- Two-link planar robot arm
- Host computer running MATLAB

The current arm dimensions documented in the project are:

- Link 1: **8.3 cm**
- Link 2: **10.1 cm**

## Control

The motor model used in the project is represented in discrete time and augmented with an integral state for zero steady-state error.

The repository contains several firmware iterations related to:

- PWM testing
- Recursive least squares / identification experiments
- LQI implementation
- Integrated two-axis control

## Repository structure

```text
Discrete_Control/
├── main/        # ESP-IDF firmware and control experiments
├── matlab/      # MATLAB communication and visualization tools
├── informe/     # LaTeX technical report and figures
├── components/  # ESP-IDF components
└── tools/
```

## Tools and technologies

- C
- ESP-IDF
- FreeRTOS
- MATLAB
- Discrete-time control
- LQI
- Robot kinematics
- PWM
- Quadrature encoders

## Technical report

A detailed report is available in:

`informe/informe_scara.tex`

It documents the robot geometry, inverse kinematics, discrete model, LQI controller and implementation architecture.

---

**Author:** Jhon Vargas  
Electronic Engineering student
