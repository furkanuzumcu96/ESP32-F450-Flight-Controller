# 🛸 ESP32 Bare-Metal Flight Controller (F450 Platform)

![Language](https://img.shields.io/badge/Language-C%2B%2B-00599C?style=flat-square&logo=c%2B%2B)
![Hardware](https://img.shields.io/badge/Hardware-ESP32%20%7C%20MPU6050-red?style=flat-square)
![Loop Rate](https://img.shields.io/badge/Loop%20Rate-250Hz%20(4ms)-brightgreen?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)

> 📦 **Firmware Source Code:** [`ESP32_Flight_Controller.ino`](./ESP32_Flight_Controller.ino)

<p align="center">
  <img src="https://github.com/user-attachments/assets/e05c7859-3ff7-4cea-91d6-589dc4be4942" width="650" />
</p>

A lightweight, high-reliability custom flight controller firmware engineered from scratch in C/C++ for an F450 quadcopter. Bypassing bulky third-party flight stacks, this firmware operates directly on the ESP32 hardware timers to achieve deterministic 250 Hz real-time attitude estimation, multi-axis PID stabilization, and 16-bit hardware-level PWM motor actuation.

---

## 📐 System Architecture & Control Pipeline

The firmware operates on a non-blocking, microsecond-accurate deterministic loop:

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
```

---

## ⚡ Key Engineering Features

* **Deterministic 250 Hz Loop:** Precise `micros()` tracking ensures minimal timing jitter and exact numerical integration for attitude kinematics.
* **Sensor Fusion (Complementary Filter):** Blends high-frequency rate-gyro data with low-frequency accelerometer tilt over high-speed 400 kHz I2C ($α = 0.98$).
* **Angle PID Stabilization:**
  * **Proportional (P):** Fast dynamic response to angular displacement.
  * **Integral (I) with Anti-Windup:** Clamped on ground/low-throttle to eliminate spool-up tip-over.
  * **Derivative (D):** Direct angular rate dampening against oscillations.
* **Bare-Metal Motor Driving:** Direct ESP32 LEDC hardware timers (16-bit, 50 Hz base) producing clean ESC actuation without software timing overhead.
* **Failsafe & Arming Safety:**
  * Auto-disarm cut-off if roll/pitch exceeds 45 degrees.
  * Zero-throttle requirement for arming switch transition (CH5).

---

## 🔌 Hardware Configuration & Pinout

| Peripheral | Component / Signal | ESP32 Pin | Interface / Protocol |
| :--- | :--- | :--- | :--- |
| **IMU** | MPU-6050 SDA | `GPIO 21` | I2C (400 kHz Fast Mode) |
| **IMU** | MPU-6050 SCL | `GPIO 22` | I2C (400 kHz Fast Mode) |
| **Receiver** | PPM Signal Out | `GPIO 32` | Hardware Edge Interrupt |
| **Motor 1** | Rear Left (CCW) | `GPIO 27` | LEDC PWM (CH0) |
| **Motor 2** | Front Left (CW) | `GPIO 25` | LEDC PWM (CH1) |
| **Motor 3** | Front Right (CCW) | `GPIO 4` | LEDC PWM (CH2) |
| **Motor 4** | Rear Right (CW) | `GPIO 14` | LEDC PWM (CH3) |

### 📐 Wiring & Interconnection Diagram

```mermaid
flowchart LR
    subgraph SENSORS ["📡 Sensors & Radio"]
        direction TB
        MPU["MPU-6050 (IMU)"] -- "SDA (GPIO 21)<br/>SCL (GPIO 22)" --> ESP
        RX["FlySky Rx (PPM)"] -- "Signal (GPIO 32)" --> ESP
    end

    ESP{{"⚡ ESP32 Core<br/>(250 Hz Loop)"}}

    subgraph ACTUATORS ["🚁 ESCs & Motors"]
        direction TB
        ESP -- "GPIO 27 (CH0)" --> M1["ESC 1 (Rear-L CCW)"]
        ESP -- "GPIO 25 (CH1)" --> M2["ESC 2 (Front-L CW)"]
        ESP -- "GPIO 04 (CH2)" --> M3["ESC 3 (Front-R CCW)"]
        ESP -- "GPIO 14 (CH3)" --> M4["ESC 4 (Rear-R CW)"]
    end

    subgraph POWER ["🔋 Power Bus"]
        direction LR
        BATT["LiPo 3S/4S"] --> PDB["Power Dist. Board (PDB)"]
        PDB -. "5V / GND" .-> ESP
        PDB -. "VBat" .-> ACTUATORS
    end
```

## 🚀 Getting Started

1. Connect hardware according to the pinout table above.
2. Open `ESP32_Flight_Controller.ino` in Arduino IDE.
3. Ensure the ESP32 board package is installed.
4. Select **ESP32 Dev Module** under **Tools > Board**.
5. Keep the quadcopter stationary on a level surface during power-up for automatic IMU gyro calibration.
6. Arm via the designated transmitter switch (CH5) with throttle at minimum.
