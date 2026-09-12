🛸 ESP32 Bare-Metal Flight Controller (F450 Platform)

**> 📦 **Firmware Source Code:** [`ESP32_Flight_Controller.ino`](./ESP32_Flight_Controller.ino)**

![Language](https://img.shields.io/badge/Language-C%2B%2B-00599C?style=flat-square&logo=c%2B%2B)
![Hardware](https://img.shields.io/badge/Hardware-ESP32%20%7C%20MPU6050-red?style=flat-square)
![Loop Rate](https://img.shields.io/badge/Loop%20Rate-250Hz%20(4ms)-brightgreen?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)

<p align="center">
  <img src="https://github.com/user-attachments/assets/e05c7859-3ff7-4cea-91d6-589dc4be4942" alt="F450 ESP32 Flight Controller Hardware" width="650">
</p>
A lightweight, high-reliability custom flight controller firmware engineered from scratch in C/C++ for an F450 quadcopter...


A lightweight, high-reliability custom flight controller firmware engineered from scratch in C/C++ for an F450 quadcopter. Bypassing bulky third-party flight stacks, this firmware operates directly on the ESP32 hardware timers to achieve deterministic 250 Hz real-time attitude estimation, multi-axis PID stabilization, and 16-bit hardware-level PWM motor actuation.

---

 📐 System Architecture & Control Pipeline

The firmware operates on a non-blocking, microsecond-accurate loop:

```text
+-------------------------------------------------------------+
|                     250 Hz Master Loop                      |
|                                                             |
|  +-------------+       +-------------------+                |
|  |   MPU6050   | ----> |   Complementary   | ---> Roll/Pitch|
|  |  (400kHz)   |       |   Filter (α=0.98) |      Attitude  |
|  +-------------+       +-------------------+          |     |
|                                                       v     |
|  +-------------+       +-------------------+       +------+ |
|  | PPM Rx Dec. | ----> |   Decoupled PID   | ----> |Mixer | |
|  |  (Interrupt)|       |   Stabilization   |       +------+ |
|  +-------------+       +-------------------+          |     |
|                                                       v     |
|                                        +------------------+ |
|                                        | Hardware LEDC PWM| |
|                                        | (50Hz / 16-bit)  | |
|                                        +------------------+ |
+-------------------------------------------------------------+



⚡ Key Engineering Features
Deterministic 250 Hz Loop: Precise micros() tracking ensures minimal jitter and exact numerical integration for attitude kinematics.

Sensor Fusion (Complementary Filter): Blends high-frequency rate-gyro data with low-frequency accelerometer tilt over high-speed 400 kHz I2C.

Angle PID Stabilization:

Proportional (P): Fast dynamic response to angular displacement.

Integral (I) with Anti-Windup: Actively clamped while on the ground / low-throttle to prevent tip-over upon spool-up.

Derivative (D): Direct angular velocity dampening to eliminate oscillations.

Bare-Metal Motor Driving: Utilizes ESP32's internal LEDC hardware timers (16-bit resolution, 50 Hz base rate) to generate stable ESC control signals without software-timing overhead.

Fail-Safe & Safety Logic:

Automated emergency engine shutdown if roll or pitch angle exceeds 45 degrees.

Zero-throttle requirement for arming switch transition (AUX1 / CH5).




🔌 Hardware Configuration & Pinout

| Peripheral | Component / Signal | ESP32 Pin | Interface / Protocol |
| :--- | :--- | :--- | :--- |
| **IMU** | MPU-6050 SDA | `GPIO 21` | I2C (400 kHz Fast Mode) |
| **IMU** | MPU-6050 SCL | `GPIO 22` | I2C (400 kHz Fast Mode) |
| **Receiver** | PPM Signal Out | `GPIO 32` | Hardware Edge Interrupt |
| **Motor 1** | Rear Left (CCW) | `GPIO 27` | LEDC PWM (CH0) |
| **Motor 2** | Front Left (CW) | `GPIO 25` | LEDC PWM (CH1) |
| **Motor 3** | Front Right (CCW) | `GPIO 4` | LEDC PWM (CH2) |
| **Motor 4** | Rear Right (CW) | `GPIO 14` | LEDC PWM (CH3) |




🚀 Getting Started
1-Connect hardware according to the pinout table above.

2-Open ESP32_Flight_Controller.ino in Arduino IDE.

3-Ensure the ESP32 board package is installed.

4-Select ESP32 Dev Module under Tools > Board.

5-Keep the quadcopter stationary on a level surface during power-up for automatic IMU gyro calibration.

6-Arm via the designated transmitter switch (CH5) with throttle at minimum.
