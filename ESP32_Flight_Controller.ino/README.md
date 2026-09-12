# ESP32 Custom Flight Controller for F450 Quadcopter

High-performance, bare-metal flight controller firmware developed for F450 quadcopters using ESP32 & MPU6050. Features a deterministic 250Hz loop, Complementary Filter sensor fusion, discrete PID stabilization with ground anti-windup, and hardware-level 16-bit PWM motor driving.

## 🚀 Features
- **250 Hz Fixed Control Loop:** Deterministic execution interval tracking via microsecond timer.
- **Sensor Fusion:** Optimized Complementary Filter blending Gyroscope and Accelerometer data.
- **Custom Angle PID Controller:** Independent Roll, Pitch, and Yaw rate stabilization.
- **Ground Anti-Windup:** Disables integral accumulation before takeoff to prevent tipping.
- **Hardware-Level PWM:** Direct 50 Hz, 16-bit LEDC pulse generation bypassing third-party servo libraries.
- **Safety Mechanisms:** 
  - Automated 45° angle emergency engine cutoff.
  - Zero-throttle safety lock and dedicated ARM switch logic.

## 🛠 Hardware Configuration & Pinout
- **MCU:** ESP32 NodeMCU
- **IMU:** MPU6050 (I2C: GPIO 21 - SDA, GPIO 22 - SCL)
- **Receiver:** PPM Receiver on GPIO 32
- **Motors:**
  - **M1 (Rear Left):** GPIO 27
  - **M2 (Front Left):** GPIO 25
  - **M3 (Front Right):** GPIO 4
  - **M4 (Rear Right):** GPIO 14
