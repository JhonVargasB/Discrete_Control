# SCARA Robot — Discrete LQI Control

Implementation of a **2-DOF planar SCARA robot** controlled with a discrete **LQI position controller** running on a **Seeed Studio XIAO ESP32-S3**.

The system receives Cartesian references from MATLAB, computes inverse kinematics on the microcontroller and controls two DC motors with encoder feedback in real time.

## Main features

- 2R planar SCARA kinematics
- Closed-form inverse kinematics
- **Discrete LQI position control for both joints**
- Encoder feedback
- DC motor actuation through a **TB6612** dual H-bridge
- PWM control
- FreeRTOS-based firmware
- USB communication between MATLAB and ESP32
- Real-time telemetry and visualization
- Joint-limit checking
- Saturation and anti-windup mechanisms
- Velocity filtering and smooth command transitions

## System overview

The host computer sends a Cartesian target `(x, y)` to the ESP32.

The microcontroller then:

1. Validates the requested Cartesian position.
2. Computes the corresponding joint references using inverse kinematics.
3. Runs one discrete LQI controller for each motor.
4. Reads encoder feedback.
5. Estimates joint position and velocity.
6. Applies PWM commands through the TB6612.
7. Returns telemetry to MATLAB for visualization and analysis.

## Hardware

- Seeed Studio XIAO ESP32-S3
- 2 DC motors with encoders
- TB6612 dual H-bridge
- Two-link planar robot arm
- Host computer running MATLAB

The arm dimensions used in the project are:

- Link 1: **$a_1 = 8.3\,\mathrm{cm}$**
- Link 2: **$a_2 = 10.1\,\mathrm{cm}$**

The encoder resolution used by the firmware is approximately **8341 counts per output-shaft revolution**, giving an angular resolution of:


$$
\Delta\theta = \frac{2\pi}{8341}
\approx 7.53\times10^{-4}\;\text{rad}
\approx 0.0432^\circ.

$$

---

# Robot kinematics

For a planar 2R manipulator, the Cartesian position of the end effector is


$$
p_x = a_1\cos(\theta_1)+a_2\cos(\theta_1+\theta_2)

$$


$$
p_y = a_1\sin(\theta_1)+a_2\sin(\theta_1+\theta_2)

$$

where:

- $a_1,a_2$ are the link lengths,
- $\theta_1,\theta_2$ are the joint angles,
- $p_x,p_y$ are the desired Cartesian coordinates.

## Inverse kinematics

Given a requested point $(p_x,p_y)$, the second joint angle can be obtained from


$$
c_2 =
\frac{p_x^2+p_y^2-a_1^2-a_2^2}
{2a_1a_2}

$$

with


$$
s_2 = \pm\sqrt{1-c_2^2}

$$

and therefore


$$
\theta_2 = \operatorname{atan2}(s_2,c_2).

$$

The first joint angle is then


$$
\theta_1 =
\operatorname{atan2}(p_y,p_x)
-
\operatorname{atan2}
\left(
a_2\sin\theta_2,
a_1+a_2\cos\theta_2
\right).

$$

The two possible signs of $s_2$ correspond to the two classical **elbow-up / elbow-down** configurations. The firmware checks the resulting solution against the allowed joint limits before commanding the motors.

---

# Discrete motor model

Each joint is driven by a DC motor with encoder feedback.

The identified discrete velocity model used in the project is


$$
\omega[k+1] = a\,\omega[k] + b\,u[k]

$$

with


$$
a = 0.7068,
\qquad
b = 3.0825.

$$

The angular position evolves according to


$$
\theta[k+1] = \theta[k] + T_s\omega[k]

$$

with sampling period


$$
T_s = 20\;\text{ms}.

$$

Defining the state vector


$$
x[k] =
\begin{bmatrix}
\theta[k]\\
\omega[k]
\end{bmatrix},

$$

the plant can be written as


$$
x[k+1] = Gx[k] + Hu[k]

$$


$$
y[k]=Cx[k]

$$

where


$$
G =
\begin{bmatrix}
1 & T_s\\
0 & a
\end{bmatrix},
\qquad
H =
\begin{bmatrix}
0\\
b
\end{bmatrix},
\qquad
C =
\begin{bmatrix}
1 & 0
\end{bmatrix}.

$$

Here:

- $\theta$: measured joint position,
- $\omega$: estimated angular velocity,
- $u$: normalized motor command,
- $y$: controlled output, which is the angular position.

---

# LQI controller

The objective is to make the measured joint angle follow a desired reference $r[k]$ while eliminating steady-state position error.

The tracking error is


$$
e[k] = r[k]-y[k].

$$

To introduce integral action, an additional state is accumulated:


$$
v[k+1] = v[k] + e[k+1].

$$

The augmented state is therefore


$$
\xi[k] =
\begin{bmatrix}
\theta[k]\\
\omega[k]\\
v[k]
\end{bmatrix}.

$$

The augmented discrete model used for the LQI design is


$$
\xi[k+1]
=
\begin{bmatrix}
G & 0\\
-CG & 1
\end{bmatrix}
\xi[k]
+
\begin{bmatrix}
H\\
-CH
\end{bmatrix}
u[k]
+
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}
r[k+1].

$$

The implemented feedback law is


$$
u[k]
=
-K_{\theta}\theta[k]
-K_{\omega}\omega[k]
+K_i v[k].

$$

The controller gains currently documented in the project are


$$
K_{\theta}=8.5621,
\qquad
K_{\omega}=0.3637,
\qquad
K_i=1.9635.

$$

The same controller structure is applied independently to both SCARA joints.

## What each term does

### Position feedback


$$
-K_{\theta}\theta[k]

$$

penalizes deviation of the angular position from the desired operating point.

### Velocity feedback


$$
-K_{\omega}\omega[k]

$$

adds damping and helps reduce oscillatory behavior.

### Integral action


$$
+K_i v[k]

$$

accumulates tracking error and allows the controller to remove residual steady-state error caused by friction, model mismatch or external disturbances.

## Practical implementation

The theoretical LQI law is complemented in firmware with practical constraints:

- control saturation,
- anti-windup logic,
- velocity filtering,
- gradual reference / command changes,
- encoder-based position feedback.

The normalized control signal is approximately limited to


$$
u \in [-0.15,\;0.15]

$$

before being translated into the PWM and motor-direction commands sent to the TB6612.

---

# Control architecture

```text
MATLAB
   │
   │  Cartesian reference (px, py)
   ▼
ESP32-S3
   │
   ├── Inverse kinematics
   │       │
   │       ├── θ1_ref
   │       └── θ2_ref
   │
   ├── LQI Controller — Joint 1
   │       └── PWM / Direction
   │
   ├── LQI Controller — Joint 2
   │       └── PWM / Direction
   │
   ▼
TB6612 Dual H-Bridge
   │
   ▼
DC Motors + Encoders
   │
   └────────── feedback ──────────► ESP32-S3
```

---

## Repository structure

```text
Discrete_Control/
├── main/        # ESP-IDF firmware and control experiments
├── matlab/      # MATLAB communication and visualization tools
├── informe/     # LaTeX technical report and figures
├── components/  # ESP-IDF components
└── tools/
```

Relevant firmware iterations include:

- `main/main_lqi.c`
- `main/main_lqi2.c`
- `main/main2_lqi.c`
- `main/main_rls.c`
- `main/main_pwm.c`

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

A more detailed derivation and implementation description is available in:

`informe/informe_scara.tex`

It documents the robot geometry, inverse kinematics, discrete motor model, LQI controller and implementation architecture.

---

**Author:** Jhon Vargas  
Electronic Engineering student
